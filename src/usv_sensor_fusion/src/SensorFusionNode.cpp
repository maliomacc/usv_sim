// =============================================================================
// SensorFusionNode.cpp — 2.5D Camera + 2D LiDAR Sensor Fusion  v4
// Package: usv_sensor_fusion | TEKNOFEST USV
//
// v4 Changes (2026-05-03):
//   - Angular Frustum LiDAR: bbox genişliğinden açı aralığı → min mesafe
//   - Windowed ZED Depth: 11×11 piksel pencere + medyan (single-pixel kaldırıldı)
//   - Çapraz Doğrulama: Δ eşiği ile ağırlıklı birleştirme veya yakın kazanır
//   - 1D Kalman Filtresi: [mesafe, hız] takibi, adaptif R gürültüsü
// =============================================================================

#include "usv_sensor_fusion/SensorFusionNode.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <limits>
#include <vector>

namespace usv_sensor_fusion
{

// =============================================================================
// Constructor
// =============================================================================
SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions & options)
: Node("sensor_fusion_node", options),
  kf_d_(0.0), kf_vd_(0.0),
  kf_P00_(1.0), kf_P01_(0.0), kf_P10_(0.0), kf_P11_(1.0),
  kf_initialized_(false), kf_last_t_(0.0)
{
  // ── Parametreler ─────────────────────────────────────────────────────────
  this->declare_parameter<double>("camera_hfov_deg",    110.0);
  this->declare_parameter<double>("image_width_px",    1280.0);
  this->declare_parameter<double>("lidar_min_valid",      0.2);
  this->declare_parameter<double>("lidar_max_valid",     15.0);
  this->declare_parameter<double>("frustum_buffer_deg",   0.5);
  this->declare_parameter<double>("crossval_threshold",   0.4);
  this->declare_parameter<double>("kalman_reset_sec",     2.0);
  this->declare_parameter<double>("kalman_reset_jump",    3.0);
  this->declare_parameter<std::string>("depth_encoding", "32FC1");
  this->declare_parameter<std::string>("depth_topic",
    "/zed/zed_node/depth/depth_registered");

  camera_hfov_rad_   = this->get_parameter("camera_hfov_deg").as_double()    * M_PI / 180.0;
  image_width_px_    = this->get_parameter("image_width_px").as_double();
  image_height_px_   = image_width_px_ * 9.0 / 16.0;
  lidar_min_valid_   = this->get_parameter("lidar_min_valid").as_double();
  lidar_max_valid_   = this->get_parameter("lidar_max_valid").as_double();
  frustum_buf_rad_   = this->get_parameter("frustum_buffer_deg").as_double() * M_PI / 180.0;
  crossval_thresh_   = this->get_parameter("crossval_threshold").as_double();
  kalman_reset_sec_  = this->get_parameter("kalman_reset_sec").as_double();
  kalman_reset_jump_ = this->get_parameter("kalman_reset_jump").as_double();
  depth_encoding_    = this->get_parameter("depth_encoding").as_string();
  depth_topic_       = this->get_parameter("depth_topic").as_string();

  RCLCPP_INFO(this->get_logger(),
    "[SensorFusion v4] HFOV=%.1f° | img=%.0fx%.0f | lidar=[%.2f,%.2f]m "
    "| frustum_buf=%.1f° | crossval=%.2fm",
    camera_hfov_rad_ * 180.0 / M_PI, image_width_px_, image_height_px_,
    lidar_min_valid_, lidar_max_valid_,
    this->get_parameter("frustum_buffer_deg").as_double(),
    crossval_thresh_);

  // ── QoS ──────────────────────────────────────────────────────────────────
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto sensor_qos   = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  // ── Abonelikler ───────────────────────────────────────────────────────────
  sub_target_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/kamikaze_target", reliable_qos,
    std::bind(&SensorFusionNode::onTarget, this, std::placeholders::_1));

  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan/filtered", sensor_qos,
    std::bind(&SensorFusionNode::onScan, this, std::placeholders::_1));

  sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&SensorFusionNode::onDepth, this, std::placeholders::_1));

  // ── Yayıncı ───────────────────────────────────────────────────────────────
  pub_fusion_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/fusion/target", reliable_qos);

  RCLCPP_INFO(this->get_logger(),
    "[SensorFusion v4] Hazır → Frustum-LiDAR | Windowed-ZED | Çapraz-Doğrulama | Kalman");
}

// =============================================================================
// Sensör önbellek callback'leri
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
// Ana füzyon callback'i
// =============================================================================
void SensorFusionNode::onTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const double now_sec = this->get_clock()->now().seconds();

  // ── 1. Mesaj decode ───────────────────────────────────────────────────────
  // point.x = cx_norm [0..1]        merkez piksel X normalize
  // point.y = cy_norm [0..1]        merkez piksel Y normalize
  // point.z = bbox_width_norm [0..1] kutu genişliği normalize
  const double cx_norm     = msg->point.x;
  const double cy_norm     = msg->point.y;
  const double bbox_w_norm = std::max(0.01, msg->point.z);  // minimum %1 genişlik

  const double cx_px = cx_norm * image_width_px_;
  const double cy_px = cy_norm * image_height_px_;

  // ── 2. Kamera açısı (yaw) ──────────────────────────────────────────────────
  const double yaw_rad = pixelToYaw(cx_px);

  // ── 3. Frustum sınırları ───────────────────────────────────────────────────
  const double half_fov = (bbox_w_norm / 2.0) * camera_hfov_rad_;
  const double theta_L  = yaw_rad - half_fov - frustum_buf_rad_;
  const double theta_R  = yaw_rad + half_fov + frustum_buf_rad_;

  // ── 4. LiDAR frustum mesafesi ──────────────────────────────────────────────
  float d_lidar = std::numeric_limits<float>::quiet_NaN();
  if (last_scan_) {
    d_lidar = getFrustumDistance(theta_L, theta_R, *last_scan_);
  }

  // ── 5. ZED pencereli medyan derinlik ──────────────────────────────────────
  float d_zed = std::numeric_limits<float>::quiet_NaN();
  if (last_depth_) {
    d_zed = sampleDepthWindow(*last_depth_,
              static_cast<int>(std::round(cx_px)),
              static_cast<int>(std::round(cy_px)));
  }

  // ── 6. Çapraz doğrulama → birleşik mesafe ─────────────────────────────────
  const bool lidar_ok = std::isfinite(d_lidar);
  const bool zed_ok   = std::isfinite(d_zed);

  float  d_fused = std::numeric_limits<float>::quiet_NaN();
  float  source  = -1.0f;
  double R_noise = 100.0;

  if (lidar_ok && zed_ok) {
    const float delta = std::fabs(d_lidar - d_zed);

    if (delta < static_cast<float>(crossval_thresh_)) {
      // Uyuşma: mesafeye göre ağırlıklı ortalama
      // LiDAR 8m'den sonra zayıflar; ZED yakında daha güçlü
      const double w_l = std::exp(-static_cast<double>(d_lidar) / 8.0);
      const double w_z = 1.0 - w_l;
      d_fused = static_cast<float>(w_l * d_lidar + w_z * d_zed);
      source  = 0.5f;
      R_noise = 0.02;
    } else {
      // Anlaşmazlık: gerçek yüzeyi gören sensör daha yakın değer verir
      if (d_lidar <= d_zed) {
        d_fused = d_lidar;  source = 0.0f;  R_noise = 0.04;
      } else {
        d_fused = d_zed;    source = 1.0f;  R_noise = 0.09;
      }
      RCLCPP_DEBUG(this->get_logger(),
        "[SensorFusion] Anlaşmazlık: LiDAR=%.2fm ZED=%.2fm Δ=%.2fm → yakın kazanır",
        d_lidar, d_zed, delta);
    }
  } else if (lidar_ok) {
    d_fused = d_lidar;  source = 0.0f;  R_noise = 0.04;
  } else if (zed_ok) {
    d_fused = d_zed;    source = 1.0f;  R_noise = 0.09;
  }
  // else: d_fused = NaN → Kalman yalnızca tahmin adımı çalışır

  // ── 7. Kalman filtresi ────────────────────────────────────────────────────
  const bool has_meas = std::isfinite(d_fused);

  if (!kf_initialized_) {
    if (!has_meas) {
      // Hiç ölçüm yok → açı-yalnız yayınla, başlatmayı bekle
      publishResult(msg, -1.0f, static_cast<float>(yaw_rad), -1.0f);
      return;
    }
    kalmanReset(static_cast<double>(d_fused), R_noise);
    kf_last_t_ = now_sec;
  } else {
    const double age = now_sec - kf_last_t_;

    if (age > kalman_reset_sec_) {
      RCLCPP_WARN(this->get_logger(),
        "[SensorFusion] Kalman bayat (%.1fs > %.1fs) → sıfırlanıyor",
        age, kalman_reset_sec_);
      if (!has_meas) {
        publishResult(msg, -1.0f, static_cast<float>(yaw_rad), -1.0f);
        kf_initialized_ = false;
        return;
      }
      kalmanReset(static_cast<double>(d_fused), R_noise);
    } else {
      const double dt = std::max(0.005, std::min(1.0, age));
      kalmanPredict(dt);

      if (has_meas) {
        // Büyük sıçrama → track kaybı, yeniden başlat
        if (std::fabs(kf_d_ - static_cast<double>(d_fused)) > kalman_reset_jump_) {
          RCLCPP_WARN(this->get_logger(),
            "[SensorFusion] Track sıçraması %.2fm → %.2fm (> %.1fm) → sıfırlanıyor",
            kf_d_, d_fused, kalman_reset_jump_);
          kalmanReset(static_cast<double>(d_fused), R_noise);
        } else {
          kalmanUpdate(static_cast<double>(d_fused), R_noise);
        }
      }
      // has_meas=false → tahmin adımı yeterli, update atla
    }
    kf_last_t_ = now_sec;
  }

  RCLCPP_DEBUG(this->get_logger(),
    "[SensorFusion] L=%.2f Z=%.2f fused=%.2f | kf_d=%.2f kf_vd=%.3f",
    lidar_ok ? d_lidar : -1.f,
    zed_ok   ? d_zed   : -1.f,
    has_meas ? d_fused : -1.f,
    kf_d_, kf_vd_);

  publishResult(msg,
    static_cast<float>(kf_d_),
    static_cast<float>(yaw_rad),
    source);
}

// =============================================================================
// Füzyon yardımcıları
// =============================================================================

float SensorFusionNode::getFrustumDistance(
  double theta_left, double theta_right,
  const sensor_msgs::msg::LaserScan & scan) const
{
  if (scan.ranges.empty() || scan.angle_increment <= 0.0f) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Kamera açısı CW-pozitif → LaserScan CCW-pozitif (yön tersi)
  const double scan_L = -theta_right;
  const double scan_R = -theta_left;

  const int n   = static_cast<int>(scan.ranges.size());
  const int i_L = static_cast<int>(
    std::floor((scan_L - static_cast<double>(scan.angle_min))
               / static_cast<double>(scan.angle_increment)));
  const int i_R = static_cast<int>(
    std::ceil ((scan_R - static_cast<double>(scan.angle_min))
               / static_cast<double>(scan.angle_increment)));

  const int start = std::max(0,     i_L);
  const int end   = std::min(n - 1, i_R);

  if (start > end) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  float best = std::numeric_limits<float>::quiet_NaN();
  for (int i = start; i <= end; ++i) {
    const float r = scan.ranges[static_cast<std::size_t>(i)];
    if (isValidRange(r)) {
      best = std::isnan(best) ? r : std::min(best, r);
    }
  }
  return best;
}

float SensorFusionNode::sampleDepthWindow(
  const sensor_msgs::msg::Image & img,
  int cx_px, int cy_px, int win) const
{
  if (depth_encoding_ != "32FC1") {
    RCLCPP_WARN_ONCE(this->get_logger(),
      "[SensorFusion] Desteklenmeyen depth encoding: '%s' (beklenen '32FC1')",
      depth_encoding_.c_str());
    return std::numeric_limits<float>::quiet_NaN();
  }

  const int w = static_cast<int>(img.width);
  const int h = static_cast<int>(img.height);

  std::vector<float> valid;
  valid.reserve(static_cast<std::size_t>((2*win+1) * (2*win+1)));

  for (int dy = -win; dy <= win; ++dy) {
    const int py = cy_px + dy;
    if (py < 0 || py >= h) continue;

    for (int dx = -win; dx <= win; ++dx) {
      const int px = cx_px + dx;
      if (px < 0 || px >= w) continue;

      const std::size_t off =
        static_cast<std::size_t>(py) * img.step +
        static_cast<std::size_t>(px) * sizeof(float);

      float v = 0.0f;
      std::memcpy(&v, &img.data[off], sizeof(float));

      if (std::isfinite(v) && v > 0.3f && v < 20.0f) {
        valid.push_back(v);
      }
    }
  }

  // En az 3 geçerli piksel şartı — güvenilirlik garantisi
  if (valid.size() < 3) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  // O(n) medyan, std::sort'tan hızlı
  auto mid = valid.begin() + static_cast<std::ptrdiff_t>(valid.size() / 2);
  std::nth_element(valid.begin(), mid, valid.end());
  return *mid;
}

double SensorFusionNode::pixelToYaw(double pixel_x) const
{
  return ((pixel_x / image_width_px_) - 0.5) * camera_hfov_rad_;
}

bool SensorFusionNode::isValidRange(float range) const
{
  if (std::isnan(range) || std::isinf(range)) return false;
  return range >= static_cast<float>(lidar_min_valid_) &&
         range <= static_cast<float>(lidar_max_valid_);
}

void SensorFusionNode::publishResult(
  const geometry_msgs::msg::PointStamped::SharedPtr & src,
  float dist, float yaw, float source) const
{
  geometry_msgs::msg::PointStamped out;
  out.header  = src->header;
  out.point.x = static_cast<double>(dist);
  out.point.y = static_cast<double>(yaw);
  out.point.z = static_cast<double>(source);
  pub_fusion_->publish(out);
}

// =============================================================================
// Kalman filtresi
// =============================================================================

void SensorFusionNode::kalmanReset(double d0, double R0)
{
  kf_d_          = d0;
  kf_vd_         = 0.0;
  kf_P00_        = R0;
  kf_P01_        = 0.0;
  kf_P10_        = 0.0;
  kf_P11_        = 0.1;
  kf_initialized_ = true;
  RCLCPP_INFO(this->get_logger(),
    "[SensorFusion] Kalman başlatıldı: d0=%.2fm", d0);
}

void SensorFusionNode::kalmanPredict(double dt)
{
  // Hareket modeli: sabit hız
  // F = [[1, dt], [0, 1]]
  // x_pred = F × x
  const double d_new  = kf_d_  + kf_vd_ * dt;
  const double vd_new = kf_vd_;

  // P_pred = F × P × F^T + Q,  Q = diag(q_d=0.001, q_v=0.01)
  const double dt2 = dt * dt;
  const double P00 = kf_P00_ + 2.0*dt*kf_P10_ + dt2*kf_P11_ + 0.001;
  const double P01 = kf_P01_ + dt * kf_P11_;
  const double P10 = kf_P10_ + dt * kf_P11_;
  const double P11 = kf_P11_ + 0.01;

  kf_d_   = d_new;   kf_vd_  = vd_new;
  kf_P00_ = P00;     kf_P01_ = P01;
  kf_P10_ = P10;     kf_P11_ = P11;
}

void SensorFusionNode::kalmanUpdate(double z, double R)
{
  // H = [1, 0] → ölçüm: sadece mesafe
  // Orijinal P değerlerini kaydet (sıralamaya duyarlı güncelleme)
  const double P00 = kf_P00_, P01 = kf_P01_;
  const double P10 = kf_P10_, P11 = kf_P11_;

  const double S   = P00 + R;           // inovasyon kovaryansı
  if (S < 1e-9) return;                 // sayısal güvence

  const double K0  = P00 / S;           // mesafe için Kalman kazancı
  const double K1  = P10 / S;           // hız için Kalman kazancı
  const double inn = z - kf_d_;         // inovasyon (ölçüm − tahmin)

  kf_d_  += K0 * inn;
  kf_vd_ += K1 * inn;

  // P_new = (I − K×H) × P,  (I−KH) = [[1−K0, 0], [−K1, 1]]
  const double rS  = R / S;             // = 1 − K0
  kf_P00_ = P00 * rS;
  kf_P01_ = P01 * rS;
  kf_P10_ = P10 - K1 * P00;            // = P10 × R/S
  kf_P11_ = P11 - K1 * P01;
}

}  // namespace usv_sensor_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_sensor_fusion::SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}
