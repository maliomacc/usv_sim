// =============================================================================
// SensorFusionNode.cpp — 2.5D Camera + 2D LiDAR Sensor Fusion
// Package: usv_sensor_fusion | TEKNOFEST USV
//
// v3 Changes:
//   - Resilience: publishes yaw with distance=-1.0 when both sensors fail
//   - lidar_max_valid default raised to 15.0m (simulation scale)
//   - Throttled warning when running without any depth fallback
// =============================================================================

#include "usv_sensor_fusion/SensorFusionNode.hpp"

#include <cstring>
#include <cmath>
#include <limits>

namespace usv_sensor_fusion
{

// =============================================================================
// Constructor
// =============================================================================
SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions & options)
: Node("sensor_fusion_node", options)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  this->declare_parameter<double>("camera_hfov_deg",   110.0);
  this->declare_parameter<double>("image_width_px",   1280.0);
  this->declare_parameter<double>("lidar_min_valid",     0.2);
  this->declare_parameter<double>("lidar_max_valid",    15.0);  // raised: sim scale
  this->declare_parameter<std::string>("depth_encoding", "32FC1");

  camera_hfov_rad_ = this->get_parameter("camera_hfov_deg").as_double() * M_PI / 180.0;
  image_width_px_  = this->get_parameter("image_width_px").as_double();
  image_height_px_ = image_width_px_ * 9.0 / 16.0;   // 720px for 1280px width
  lidar_min_valid_ = this->get_parameter("lidar_min_valid").as_double();
  lidar_max_valid_ = this->get_parameter("lidar_max_valid").as_double();
  depth_encoding_  = this->get_parameter("depth_encoding").as_string();

  RCLCPP_INFO(this->get_logger(),
    "[SensorFusion] Init | HFOV=%.1f° | img=%.0fx%.0f | lidar=[%.2f, %.2f]m",
    camera_hfov_rad_ * 180.0 / M_PI,
    image_width_px_, image_height_px_,
    lidar_min_valid_, lidar_max_valid_);

  // ── QoS Profiles ──────────────────────────────────────────────────────────
  // /kamikaze_target: RELIABLE, must match Python publisher
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  // Sensor data: BEST_EFFORT (cache latest, no guarantee needed)
  auto sensor_qos   = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  // ── Subscriptions ─────────────────────────────────────────────────────────
  sub_target_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/kamikaze_target", reliable_qos,
    std::bind(&SensorFusionNode::onTarget, this, std::placeholders::_1));

  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan/filtered", sensor_qos,
    std::bind(&SensorFusionNode::onScan, this, std::placeholders::_1));

  // ZED depth: /roboboat/sensors/camera/depth (bridged from Ignition rgbd_camera)
  sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/depth", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&SensorFusionNode::onDepth, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────────────────
  pub_fusion_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/fusion/target", reliable_qos);

  RCLCPP_INFO(this->get_logger(),
    "[SensorFusion] Ready. /kamikaze_target(RELIABLE) | /scan/filtered | /zed/depth");
}

// =============================================================================
// Sensor Cache Callbacks
// =============================================================================
void SensorFusionNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

void SensorFusionNode::onDepth(const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_depth_ = msg;
}

// =============================================================================
// Main Fusion Callback
// =============================================================================
void SensorFusionNode::onTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // ── Step 1: Reconstruct pixel coordinates ─────────────────────────────────
  // kamikaze_control.py: point.x = cx/width, point.y = cy/height (normalised)
  const double bbox_cx = msg->point.x * image_width_px_;
  const double bbox_cy = msg->point.y * image_height_px_;

  // ── Step 2: Pixel → horizontal yaw angle ─────────────────────────────────
  const double yaw_rad = pixelToYaw(bbox_cx);

  // ── Step 3: LiDAR primary distance ───────────────────────────────────────
  float distance = std::numeric_limits<float>::quiet_NaN();
  float source   = 0.0f;  // 0=LiDAR, 1=ZED, -1=angle-only

  if (last_scan_) {
    const int idx = yawToScanIndex(yaw_rad, *last_scan_);
    if (idx >= 0 && idx < static_cast<int>(last_scan_->ranges.size())) {
      const float r = last_scan_->ranges[static_cast<std::size_t>(idx)];
      if (isValidRange(r)) {
        distance = r;
      }
    }
  }

  // ── Step 4: ZED depth fallback ────────────────────────────────────────────
  if (std::isnan(distance) || std::isinf(distance)) {

    if (last_depth_) {
      source = 1.0f;
      const int px = static_cast<int>(std::round(bbox_cx));
      const int py = static_cast<int>(std::round(bbox_cy));
      const float dv = sampleDepthImage(*last_depth_, px, py);

      if (!std::isnan(dv) && !std::isinf(dv) && dv > 0.01f) {
        distance = dv;
        RCLCPP_DEBUG(this->get_logger(),
          "[SensorFusion] ZED fallback: %.2fm (yaw=%.1f°)",
          distance, yaw_rad * 180.0 / M_PI);
      } else {
        // ZED returned invalid value → fall through to angle-only below
        distance = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  // ── Step 5: Angle-only fallback — ALWAYS publish, never silently drop ─────
  // If BOTH LiDAR and ZED depth are invalid/unavailable, we still publish
  // with distance = -1.0 as a "target visible, distance unknown" signal.
  // This prevents downstream nodes from losing track of the target angle.
  if (std::isnan(distance) || std::isinf(distance)) {
    distance = -1.0f;
    source   = -1.0f;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[SensorFusion] No range data available (LiDAR invalid, ZED %s). "
      "Publishing angle-only: yaw=%.1f° dist=-1.0",
      last_depth_ ? "invalid" : "not connected",
      yaw_rad * 180.0 / M_PI);
  }

  // ── Step 6: Publish fused result ─────────────────────────────────────────
  geometry_msgs::msg::PointStamped out;
  out.header  = msg->header;                        // preserve camera timestamp
  out.point.x = static_cast<double>(distance);      // [m], -1.0 = angle-only
  out.point.y = yaw_rad;                            // horizontal yaw [rad]
  out.point.z = static_cast<double>(source);        // 0=LiDAR, 1=ZED, -1=none

  pub_fusion_->publish(out);
}

// =============================================================================
// Helpers
// =============================================================================

double SensorFusionNode::pixelToYaw(double pixel_x) const
{
  const double normalised = (pixel_x / image_width_px_) - 0.5;
  return normalised * camera_hfov_rad_;
}

int SensorFusionNode::yawToScanIndex(
  double yaw_rad,
  const sensor_msgs::msg::LaserScan & scan) const
{
  // Camera yaw: CW-positive → negate to convert to CCW (LaserScan convention)
  const double scan_angle = -yaw_rad;
  if (scan_angle < scan.angle_min || scan_angle > scan.angle_max) {
    return -1;
  }
  const int idx = static_cast<int>(
    std::round((scan_angle - scan.angle_min) / scan.angle_increment));
  const int max_idx = static_cast<int>(scan.ranges.size()) - 1;
  return std::max(0, std::min(idx, max_idx));
}

bool SensorFusionNode::isValidRange(float range) const
{
  if (std::isnan(range) || std::isinf(range)) return false;
  return (range >= static_cast<float>(lidar_min_valid_) &&
          range <= static_cast<float>(lidar_max_valid_));
}

float SensorFusionNode::sampleDepthImage(
  const sensor_msgs::msg::Image & img,
  int px, int py) const
{
  if (px < 0 || py < 0 ||
      px >= static_cast<int>(img.width) ||
      py >= static_cast<int>(img.height)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  if (depth_encoding_ == "32FC1") {
    const std::size_t offset =
      static_cast<std::size_t>(py) * img.step +
      static_cast<std::size_t>(px) * sizeof(float);
    float val = 0.0f;
    std::memcpy(&val, &img.data[offset], sizeof(float));
    return val;
  }
  RCLCPP_WARN_ONCE(this->get_logger(),
    "[SensorFusion] Unsupported depth encoding: '%s'. Expected '32FC1'.",
    depth_encoding_.c_str());
  return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace usv_sensor_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_sensor_fusion::SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}
