// =============================================================================
// SensorFusionNode.hpp — 2.5D Camera + 2D LiDAR Sensor Fusion  v4
// Package: usv_sensor_fusion | TEKNOFEST USV
//
// Pipeline (per /kamikaze_target):
//   1. Angular Frustum  — bbox genişliğinden [θ_L, θ_R] aralığı → min LiDAR mesafe
//   2. Windowed ZED     — 11×11 piksel pencere medyan derinlik
//   3. Çapraz Doğrulama — Δ < 0.4m → ağırlıklı ortalama; Δ ≥ 0.4m → yakın kazanır
//   4. 1D Kalman        — [mesafe, yaklaşma_hızı] durum tahmini + pürüzsüzleştirme
//
// /kamikaze_target (geometry_msgs/PointStamped):
//   point.x = cx_norm         [0..1]   merkez piksel X normalize
//   point.y = cy_norm         [0..1]   merkez piksel Y normalize
//   point.z = bbox_width_norm [0..1]   kutu genişliği normalize  ← v4'te değişti
//
// /fusion/target (geometry_msgs/PointStamped):
//   point.x = distance [m]   Kalman çıkışı (-1.0 = sadece açı)
//   point.y = yaw [rad]      kamera açısından ölçülen yatay açı
//   point.z = source         0.0=LiDAR 0.5=birleşik 1.0=ZED -1.0=açı-yalnız
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
  // ── Parametreler ────────────────────────────────────────────────────────────
  double camera_hfov_rad_;
  double image_width_px_;
  double image_height_px_;
  double lidar_min_valid_;
  double lidar_max_valid_;
  double frustum_buf_rad_;    // frustum'a eklenen açısal tolerans [rad]
  double crossval_thresh_;    // LiDAR-ZED anlaşmazlık eşiği [m]
  double kalman_reset_sec_;   // bu kadar sessizlik → Kalman sıfırla [s]
  double kalman_reset_jump_;  // bu kadar sıçrama → Kalman sıfırla [m]
  std::string depth_encoding_;
  std::string depth_topic_;

  // ── Sensör önbelleği ────────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  sensor_msgs::msg::Image::SharedPtr     last_depth_;

  // ── Kalman filtresi durumu ───────────────────────────────────────────────────
  // Durum: x = [d, vd]   d=mesafe[m], vd=değişim_hızı[m/s]
  double kf_d_;                          // mesafe tahmini
  double kf_vd_;                         // hız tahmini
  double kf_P00_, kf_P01_, kf_P10_, kf_P11_;  // 2×2 kovaryans matrisi
  bool   kf_initialized_;
  double kf_last_t_;                     // son güncelleme zamanı [s]

  // ── Abonelikler ─────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_depth_;

  // ── Yayıncı ─────────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_fusion_;

  // ── Callback'ler ────────────────────────────────────────────────────────────
  void onTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onScan  (const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onDepth (const sensor_msgs::msg::Image::SharedPtr msg);

  // ── Füzyon yardımcıları ─────────────────────────────────────────────────────
  float  getFrustumDistance(double theta_left, double theta_right,
                             const sensor_msgs::msg::LaserScan & scan) const;
  float  sampleDepthWindow (const sensor_msgs::msg::Image & img,
                             int cx_px, int cy_px, int win = 5) const;
  double pixelToYaw        (double pixel_x) const;
  bool   isValidRange      (float range) const;

  // ── Kalman yardımcıları ─────────────────────────────────────────────────────
  void kalmanReset  (double d0, double R0);
  void kalmanPredict(double dt);
  void kalmanUpdate (double z, double R);

  // ── Yayın yardımcısı ────────────────────────────────────────────────────────
  void publishResult(const geometry_msgs::msg::PointStamped::SharedPtr & src,
                     float dist, float yaw, float source) const;
};

}  // namespace usv_sensor_fusion
