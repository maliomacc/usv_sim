# STI USV — Geliştirme ve Eksikler Raporu
**Tarih:** 2026-05-03 | **Standart:** Profesyonel ROS 2 otonomi projesi kriteri

---

## Özet Puan Tablosu

| Kategori | Durum |
|----------|-------|
| Temel görev mantığı | ✅ İşlevsel |
| Hata yönetimi / watchdog | ⚠ Kısmi |
| Thread güvenliği | ⚠ Eksik |
| Sensör füzyonu kalitesi | ⚠ Temel düzey |
| Donanım geçişi hazırlığı | ❌ Kritik boşluklar |
| Test / CI altyapısı | ❌ Yok |

---

## KRİTİK HATALAR (Gerçek donanımda çöker)

### C-1: KamikazeControl — Thread Güvensiz Sensör Erişimi
**Dosya:** `workspace_nav/scripts/kamikaze_control.py:344-348`

`_latest_depth`, `_latest_scan`, `_latest_conf` değişkenleri Thread B (sensor CB) tarafından yazılır, Thread D (20Hz publish loop) tarafından okunur. Numpy dizileri Python'da atomik değildir; kısmi yazım sırasında okuma olursa **boyut uyumsuzluğu → crash**.

```python
# Mevcut (güvensiz):
depth_img = self._latest_depth   # Thread D, Thread B yazarken okuyabilir

# Düzeltme:
self._sensor_lock = threading.Lock()
with self._sensor_lock:
    depth_img = self._latest_depth
```

---

### C-2: SensorFusionNode — Tek Piksel Derinlik Örneklemesi
**Dosya:** `usv_sensor_fusion/src/SensorFusionNode.cpp:195-216`

`sampleDepthImage()` tek pikselden değer alıyor. Python'daki `_zed_depth_at_pixel()` ise 11×11 pencere + medyan filtresi + güven maskesi kullanıyor. Gerçek ZED kamerası özellikle suyun yüzeyi gibi speküler yüzeylerde NaN/Inf değer üretiyor; tek piksel örnekleme mesafe kaybına → kamikaze mod körleşmesine yol açar.

```cpp
// Düzeltme: pencereli medyan örnekleme ekle (Python versiyonundaki mantığı port et)
float sampleDepthWindow(const Image& img, int px, int py, int win=5) const;
```

---

### C-3: Sabit Kodlanmış GPS Kovaryansı
**Dosya:** `workspace_ros/scripts/gps_covariance_repub.py:27-33`

```python
hdop = 3.0  # Her zaman 3.0 — gerçek HDOP'u yoksayıyor
var = (hdop * 1.5) ** 2  # = 20.25 m²
```

MAVROS zaten `/mavros/global_position/global` içinde gerçek HDOP'u yayınlar. EKF bu sabit kovaryansı inanır; açık alanda GPS iyiyken de zayıf güven verir → EKF ağırlıklaması yanlış. Gerçek donanımda GPS kayması artar.

**Düzeltme:** `msg.position_covariance` değerlerini MAVROS'tan geldikleri gibi bırak; sadece `frame_id` ve `covariance_type` düzelt.

---

### C-4: Yanlış LiDAR Açı Dönüşümü (SensorFusionNode)
**Dosya:** `usv_sensor_fusion/src/SensorFusionNode.cpp:177`

```cpp
const double scan_angle = -yaw_rad;  // kamera CW → LaserScan CCW dönüşümü
```

Bu dönüşüm, LiDAR'ın tam olarak kameraya göre hizalı (0° offset, aynı Z ekseninde) monte edildiğini varsayar. Gerçek teknede LiDAR çoğunlukla kameradan farklı yükseklikte/açıda monte edilir. TF ağacındaki `lidar_link→camera_link` dönüşümü kullanılmıyor. Sahada yanlış açı eşleşmesi → mesafe ölçümü yanlış.

---

## ORTA ÖNEMLİ EKSİKLER

### O-1: PID Hız Profili — Basamak Fonksiyonu
**Dosya:** `scripts/mission_manager.py:1101-1110`

```python
if heading_deg > 45:  target_vx = 0.2
elif heading_deg > 25: target_vx = 0.8
elif heading_deg > 10: target_vx = 1.5
else:                  target_vx = PID_MAX_SPEED
```

Bu basamak geçişleri, belirli açı değerlerinde ani hız değişimi üretir. Mevcut LPF (`alpha=0.10`) bu süreksizliği yeterince yumuşatmıyor (zaman sabiti ≈ 10 adım × 0.05s = 0.5s). Güçlü akıntıda ani hız kesmeleri osilasyona neden olur.

**Düzeltme:** `target_vx = PID_MAX_SPEED * exp(-k * |heading_deg|)` gibi sürekli bir profil.

---

### O-2: MissionManager — İçe Aktarma (import) Döngü İçinde
**Dosya:** `scripts/mission_manager.py:765, 1159, 1248, 1277`

```python
def _gate_center_cb(self, msg):
    import time as _time        # 20 Hz'de her çağrıda tekrar edilir

def _run_parkur2_mppi(self):
    import time as _tnow        # 20 Hz kontrol döngüsünde
    from geometry_msgs.msg import Twist as _Twist  # 20 Hz kontrol döngüsünde
```

Python'da modül önbelleğe alındığı için performans etkisi minimal; ancak bu pattern kodu gizler ve statik analiz araçlarını yanıltır. Tüm importlar dosya üstüne taşınmalı.

---

### O-3: `_yellow_visible_cb` Boş
**Dosya:** `scripts/mission_manager.py:737-738`

```python
def _yellow_visible_cb(self, msg) -> None:
    pass
```

`/yellow_visible` topic'ine abone olunuyor, gelen veri hiç kullanılmıyor. Eski tasarım artığı. Abonelik kaldırılmalı veya anlamlı bir işlev verilmeli.

---

### O-4: EKF GPS Entegrasyonu Eksik
**Dosya:** `workspace_nav/config/ekf_fusion.yaml`

EKF yalnızca ZED odom + IMU fuse ediyor. `navsat_transform_node` var (`navsat.yaml`) ancak GPS düzeltmesi EKF girdisi olarak yapılandırılmamış. Bu durumda uzun süreli görevlerde ZED odometri kayması GPS ile düzeltilemez. Gerçek su testinde 100m+ koşularda konumsal hata birikir.

**Düzeltme:** `ekf_fusion.yaml`'a `odom1: /odometry/gps` girişi ekle (navsat_transform_node çıkışı).

---

### O-5: GateFusion Standart Sapma Hesabı
**Dosya:** `scripts/mission_manager.py:412-423`

```python
var_x = sum((x - mean_x) ** 2 for x in xs) / (n - 1)
var_y = sum((y - mean_y) ** 2 for y in ys) / (n - 1)
return math.sqrt(var_x + var_y)  # 2D bileşik std
```

`sqrt(var_x + var_y)` matematiksel olarak "2D pozisyon belirsizliği" (mahalanobis mesafesinin basitleştirilmiş hali) sayılabilir, ancak bu, X ve Y'nin eşit ağırlıklı ve bağımsız olduğunu varsayar. Eğik açıdan gelen kapı tespitlerinde X-Y korelasyonu var; bu formül eşik değeriyle tutarsız sonuç üretebilir. Daha sağlam yöntem: `max(std_x, std_y)` veya `hypot(std_x, std_y) / sqrt(2)`.

---

### O-6: Nav2 Collision Monitor Bağlantısı Belirsiz
**Dosya:** `config/nav2_params_usv_pure.yaml:432-433`

```yaml
cmd_vel_in_topic: "cmd_vel_smoothed"
cmd_vel_out_topic: "cmd_vel_cm_out"
```

Collision Monitor `cmd_vel_cm_out` yayınlıyor; MAVROS köprüsü ise `/cmd_vel` dinliyor. Bu iki topic bağlanmadığı sürece Collision Monitor **etkisiz**. Launch dosyasında remapping veya köprü konfigürasyonu gerekli.

---

### O-7: PID İntegral Anti-Windup Eksik
**Dosya:** `scripts/mission_manager.py:1091-1092`

```python
self._pid_integral = max(-0.5, min(0.5, self._pid_integral + steering_err * PID_DT))
```

İntegral doyum sınırı var (±0.5) ama gerçek anti-windup yok. Tekne bir engel nedeniyle istenen rotaya dönemezken integratör dolmaya devam eder; engel kalktığında aşırı dönüş (windup) oluşur. Düzeltme: sadece çıktı doyumda olmadığında integratörü güncelle.

---

### O-8: Kamikaze Zamanlayıcı Karışıklığı
**Dosya:** `scripts/mission_manager.py:1419, 1452`

Kamikaze döngüsünde `lost_time` hesabında `self.get_clock().now().nanoseconds / 1e9` kullanılıyor (ROS duvar saati), ancak `_kamikaze_last_seen` set edilirken aynı yöntem kullanılıyor — tutarlı. Ancak `_fusion_last_seen` staleness kontrolü de aynı kaynaktan geliyor. Simülasyon ile gerçek donanım arasında saat farkı olabilir; `use_sim_time` flag'i dikkatli yönetilmeli.

---

## KÜÇÜK EKSİKLER / İYİLEŞTİRME ALANLARI

### K-1: MPPI Parametre Uygulaması — Retry Yok
**Dosya:** `scripts/mission_manager.py:222-229`

```python
if not self._client.service_is_ready():
    self._log.warn('... servis hazır değil — Nav2 başladıktan sonra otomatik uygulanacak.')
    return  # Hiç retry yok
```

Servis hazır değilse parametre değişikliği sessizce iptal edilir. Slalom moduna geçiş anında Nav2 henüz başlamadıysa default Sprint parametreleriyle çalışmaya devam eder. Basit bir retry queue eklenebilir.

---

### K-2: `local_goal_bridge.py` Entegrasyonu Belirsiz
**Dosya:** `scripts/local_goal_bridge.py`

`/usv_local_goal` topic'i sistemin hiçbir yerinde yayınlanmıyor. Bu düğüm launch dosyalarına eklenmemiş gibi görünüyor; ölü kod olabilir veya gelecek entegrasyon için placeholder.

---

### K-3: KamikazeControl Model Yolu Sabit Kodlanmış
**Dosya:** `scripts/kamikaze_control.py:101`

```python
self.declare_parameter('model_path', '/home/seatech/models/buoy.engine')
```

Parametre olarak tanımlanmış, iyi. Ancak TRT `.engine` modeli Jetson mimarisine özgü; farklı bir Jetson veya PC'de çalışmaz. Launch dosyasında açıkça belirtilmeli.

---

### K-4: Test Altyapısı Yok
Tüm kod tabanında unit test veya integration test dosyası **bulunmuyor**. Minimum şunlar eklenebilir:
- `GateFusionHandler._compute_std()` için birim test
- PID döngüsü için hedef yakınsama simülasyonu
- GPS dönüşüm callback'i için mock servisle entegrasyon testi

---

### K-5: Gerçek Donanım Darboğazları

| Bileşen | Risk | Tahmin |
|---------|------|--------|
| YOLOv8 TRT inference | Jetson Orin'de ~15-25ms; 20Hz döngüsü ile uyumlu ancak ısıl throttling tehlikesi | ⚠ Orta |
| MPPI batch=2000, iter=4 | CPU'da ~3-5ms; Jetson'da kabul edilebilir | ✅ Düşük |
| Nav2 global costmap 150×150@0.3m | 2Hz güncelleme: ~150ms hesaplama; dalga hareketi TF titremesiyle birleşince local plan sapabilir | ⚠ Orta |
| ZED derinlik + güven haritası @ 30fps | USB3 bant genişliği; Jetson'da OK; ek vibrasyon → NaN oranı artar | ⚠ Orta |
| EKF 30Hz + slam_toolbox + Nav2 | Eşzamanlı yük; SLAM'ın loop closure sırasında TF gecikmesi Nav2 plan sapmasına neden olabilir | ⚠ Orta |

---

## Simülasyon → Gerçek Donanım Geçişi Kontrol Listesi

- [ ] `use_sim_time: true` → `false` tüm YAML dosyalarında
- [ ] GPS kovaryansı gerçek HDOP'tan hesaplanacak şekilde güncelle (C-3)
- [ ] LiDAR-kamera açı ofseti TF ağacından alınacak şekilde düzelt (C-4)
- [ ] Thread güvenliği için sensör lock eklenmeli (C-1)
- [ ] SensorFusionNode pencereli derinlik örneklemesi (C-2)
- [ ] Collision Monitor topic remapping (O-6)
- [ ] EKF'e GPS girişi eklenmeli (O-4)
- [ ] Isıl yönetim: Jetson'da inference + Nav2 + SLAM eşzamanlı çalışma profili çıkarılmalı
- [ ] Motorlu gerçek testlerde PID Kp/Kd tekrar ayarlanmalı (simülasyon su direnci yok)
- [ ] MPPI obstacle critic ağırlıkları gerçek duba boyutlarıyla kalibre edilmeli
