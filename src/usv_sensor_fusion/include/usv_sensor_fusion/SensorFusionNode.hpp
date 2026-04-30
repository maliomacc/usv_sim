// =============================================================================
// SensorFusionNode.hpp — 2.5D Camera + 2D LiDAR Sensor Fusion
// Package: usv_sensor_fusion | TEKNOFEST USV
// =============================================================================
// Architecture:
//   Inputs:
//     /kamikaze_target  (geometry_msgs/PointStamped) — normalized pixel coords
//                         point.x = cx_norm [0..1]
//                         point.y = cy_norm [0..1]
//                         point.z = buoy area [px²] (unused for fusion)
//     /scan/filtered    (sensor_msgs/LaserScan)       — RPLidar A1M8 filtered
//     /zed/depth        (sensor_msgs/Image)            — ZED 32FC1 depth map
//
//   Output:
//     /fusion/target    (geometry_msgs/PointStamped)
//                         point.x = target distance [m]
//                         point.y = horizontal yaw angle [rad]
//                         point.z = source: 0.0=LiDAR, 1.0=ZED depth fallback
//
// QoS Note:
//   /kamikaze_target is published RELIABLE from Python (kamikaze_control.py).
//   We must match with a RELIABLE subscription or the connection will be refused.
// =============================================================================

#pragma once

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace usv_sensor_fusion
{

class SensorFusionNode : public rclcpp::Node
{
public:
  explicit SensorFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Parameters ──────────────────────────────────────────────────────────
  double camera_hfov_rad_;    // Camera horizontal FOV [rad]
  double image_width_px_;     // Camera image width [px]
  double image_height_px_;    // Camera image height [px]  (= width * 9/16 for 16:9)
  double lidar_min_valid_;    // LiDAR valid range lower bound [m]
  double lidar_max_valid_;    // LiDAR valid range upper bound [m]
  std::string depth_encoding_;
  std::string depth_topic_;

  // ── Cached sensor data ───────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  sensor_msgs::msg::Image::SharedPtr     last_depth_;

  // ── Subscriptions ────────────────────────────────────────────────────────
  // /kamikaze_target: RELIABLE QoS to match kamikaze_control.py publisher
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_depth_;

  // ── Publisher ────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_fusion_;

  // ── Callbacks ────────────────────────────────────────────────────────────
  void onTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg);

  // ── Helpers ──────────────────────────────────────────────────────────────
  double pixelToYaw(double pixel_x) const;
  int    yawToScanIndex(double yaw_rad, const sensor_msgs::msg::LaserScan & scan) const;
  bool   isValidRange(float range) const;
  float  sampleDepthImage(
    const sensor_msgs::msg::Image & depth_img,
    int pixel_x, int pixel_y) const;
};

}  // namespace usv_sensor_fusion
