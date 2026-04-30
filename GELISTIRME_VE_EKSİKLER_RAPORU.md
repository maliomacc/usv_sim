# STI USV — Geliştirme ve Eksikler Raporu
**Analiz Tarihi:** 2026-04-30  
**Standart:** Endüstriyel otonomi projesi + TEKNOFEST yarışma şartnameleri

---

## Özet Puan Tablosu

| Kategori | Puan | Durum |
|----------|------|-------|
| Algı (Perception) | 7/10 | İyi — çoklu sensör, yedek zinciri var |
| Lokalizasyon | 6/10 | Orta — GPS+IMU sağlam, SLAM bağlantısı gevşek |
| Planlama & Kontrol | 6/10 | Orta — MPPI iyi, PID ilkel |
| Hata Yönetimi | 5/10 | Zayıf — respawn var ama watchdog eksik |
| Kod Kalitesi | 6/10 | Orta — kod tekrarı yüksek |
| Test Edilebilirlik | 7/10 | İyi — standalone düğümler var |

---

## 1. KRİTİK SORUNLAR (Yarışmayı Doğrudan Etkiler)

### 1.1 MAVROS Köprüsü Hız Kırpması
**Dosya:** `workspace_ros/workspace_ros/cmd_vel_to_mavros.py`  
**Sorun:** `max_speed=1.0 m/s` ile başlatılıyor (`usv_autonomy.launch.py`), ama `mission_manager` Aşama 3 Faz 2'de `v = base_speed × 5.0 = 7.5 m/s` gönderiyor. `cmd_vel_to_mavros`'un PWM ölçekleme referansı `max_speed` olduğundan bu değer doğru PWM çıkışına dönüşmeyebilir.  
**Önerilen Düzeltme:** `usv_autonomy.launch.py`'de `max_speed` parametresini en azından `base_speed` ile aynı hizaya getir ya da MAVROS köprüsünde kırpma yerine `clamp` + log ekle.

### 1.2 GateFusion — Kodun İki Yerde Tekrarlanması
**Dosyalar:** `scripts/mission_manager.py` + `workspace_nav_entry/parkur2_standalone.py`  
**Sorun:** `GateFusionHandler` sınıfı iki ayrı dosyada neredeyse aynı şekilde bulunuyor. Bir parametreyi değiştirdiğinde iki yerde düzeltmen gerekiyor; biri unutulduğunda davranış uyuşmazlığı çıkar.  
**Önerilen Düzeltme:** `GateFusionHandler`'ı `scripts/gate_fusion.py`'ye taşı ve her iki dosyadan import et.

### 1.3 Stage 1 PID — Akıntı/Sürükleme Etkisine Karşı Savunmasız
**Dosya:** `scripts/mission_manager.py` — `_run_stage1_pid()`  
**Sorun:** Saf oransal (P) kontrolcü; integral terimi (I=0) yok. Denizde sürekli yan akıntı varsa araç hiç ulaşamayacağı stabil bir hata noktasına girer. Kd terimi kod içinde var ama pratik etkisi deniz koşullarında sınırlı.  
**Önerilen Düzeltme:** Küçük integral terimi (Ki=0.05–0.1, wind-up sınırı ile) ekle. Ya da daha iyi: Aşama 1 için de Nav2/MPPI'yi kullan, PID'i kaldır.

### 1.4 SLAM → Nav2 TF Zinciri Doğrulanmamış
**Sorun:** `slam_toolbox` `map→odom` TF'ini yayınlar. Eğer SLAM başlatılmadan `usv_autonomy.launch.py` çalıştırılırsa Nav2 `map` çerçevesini bulamaz ve tüm planlama sessizce başarısız olur. Herhangi bir hata mesajı veya acil durum koşulu yok.  
**Önerilen Düzeltme:** `mission_manager` başlangıcında `map→base_link` TF varlığını 5 saniye boyunca kontrol eden basit bir bekleme döngüsü ekle.

---

## 2. ORTA ÖNEMLİ EKSİKLER

### 2.1 Sağlık İzleme (Health Monitoring) Yok
**Sorun:** Hiçbir düğüm sensör akışlarının kesildiğini tespit etmiyor:
- ZED kamera çöküyor → `kamikaze_control` yeni görüntü alamıyor ama hata vermiyor
- LiDAR bağlantısı kopuyor → `sensor_fusion_node` sessizce angle-only moduna geçiyor
- EKF yakınsama sağlayamazsa `/odometry/filtered` durur → mission_manager'daki 2s watchdog var ama log seviyesi sadece WARN

**Önerilen Düzeltme:** Minimal watchdog düğümü: kritik topic'lerin son mesaj zamanını izle, 3s susarsa `/mission_state`'e "SENSOR_FAIL" yaz ve cmd_vel=0 yayınla.

### 2.2 `static_transform.yaml` — Simülatör Çerçeve İsimleri
**Dosya:** `workspace_ros/config/static_transform.yaml`  
**Sorun:** Hedef çerçeveler `roboboat/base_link/sensor_lidar` şeklinde simülatör isimlendirmesi kullanıyor. Gerçek donanımda çerçeve isimleri farklıysa Nav2 costmap LiDAR verilerini `base_link`'e dönüştüremez, engeller haritada yanlış konuma yerleşir.  
**Önerilen Düzeltme:** Gerçek donanım frame isimlerini doğrula ve YAML'ı güncelle.

### 2.3 `kiss_icp.launch.py` — Simülatör Topic Kalıntısı
**Dosya:** `workspace_ros/launch/kiss_icp.launch.py`  
**Sorun:** `topic: '/roboboat/lidar/points'` simülatör topic'i. Gerçek donanımda bu topic yok → KISS-ICP başlangıçta subscriber oluşturur ama hiç veri almaz.  
**Önerilen Düzeltme:** `default_value='/velodyne_points'` veya gerçek 3D LiDAR topic'i ile güncelle.

### 2.4 `ekf_fusion.yaml` — Kullanılmayan Config Dosyası
**Dosya:** `workspace_nav/config/ekf_fusion.yaml`  
**Sorun:** ZED odomometrisini (görsel odometri) girdi olarak tanımlıyor, ama `localization.launch.py` bunu başlatmıyor — yalnızca GPS+IMU kullanıyor. Bu dosya hiçbir launch tarafından çağrılmıyor, ama silinmesi için de emin olmak lazım.

### 2.5 Nav2 `xy_goal_tolerance: 2.5 m` — Çok Geniş
**Dosya:** `workspace_nav/config/nav2_params_usv_pure.yaml`  
**Sorun:** 2.5 metre yarıçap kapı genişliğinden büyük olabilir. Nav2, araç kapıdan 2.5m önce olduğunda "başarılı" sayabilir ve erken Aşama 3'e geçebilir.  
**Önerilen Düzeltme:** `xy_goal_tolerance: 1.0` veya GateFusion kilidine bağlı dinamik tolerans kullan.

### 2.6 Kamikaze Kilit Mekanizması — 3s Sabit Gecikme
**Dosya:** `scripts/kamikaze_control.py`  
**Sorun:** İlk tespitten 3 saniye sonra kilit açılıyor. Eğer araç hızlı yaklaşıyorsa (Faz 1b v=4.5 m/s), 3 saniyede 13.5 metre gidilir — kilit açıldığında hedef zaten geçilmiş olabilir. Tam ters: araç yavaşsa kilit çok erken açılabilir ve yanlış yönde tam hız verilebilir.  
**Önerilen Düzeltme:** Kilit koşulunu zaman yerine mesafeye bağla: `dist < 2.0m AND conf_frames > 10`.

---

## 3. DARBOĞAZLAR (Gerçek Donanımda Ortaya Çıkacak)

### 3.1 YOLO + ZED Derinlik — CPU/GPU Bant Genişliği
**Sorun:** Jetson Orin NX 8GB'de:
- ZED SDK 1280×720@30fps → ~110 MB/s ham veri
- YOLOv8 TRT ~18–22ms çıkarım süresi
- `sensor_fusion_node` aynı anda `/scan/filtered` işliyor

Thread-D 20 Hz yayın döngüsünde depth görüntüsünü kopyalıyor (`last_depth_ = msg`). Büyük mesaj paylaşım kilidi olmadan bu potansiyel olarak race condition yaratır (Python GIL korur ama C++ paylaşımı `onDepth` ve `onTarget` arasında güvenli).  
**Durum:** C++ tarafında `last_depth_` std::shared_ptr ataması atomik değil ama rclcpp callback queue serileştirir — şu an güvenli.

### 3.2 Nav2 MPPI — Jetson'da CPU Yükü
**Sorun:** `batch_size: 2000`, `iteration_count: 4` → 8000 traj. evaluasyonu / kontrol döngüsü. Çevrimiçi benchmark: Orin NX'de ~45–60ms MPPI döngüsü = sadece ~17 Hz. `controller_frequency: 20.0` ile hafif uyuşmazlık.  
**Önerilen Düzeltme:** `batch_size: 1000` ile test yap; performans yeterliyse düşük tut.

### 3.3 LiDAR Tarama Frekansı vs. Hareket Hızı
**Sorun:** RPLidar A1M8 @ 5.5 Hz. Araç Faz 2'de 4.5 m/s gidiyorsa, iki tarama arasında 0.8m hareket eder — engel tespiti ciddi ölü zona sahip.  
**Önerilen Düzeltme:** RPLidar'ı en yüksek hız moduna al (10 Hz); ya da hız limitini MPPI'dan 2.0 m/s ile kısıtla.

### 3.4 GPS Başlangıç Yakınsaması
**Sorun:** `navsat_transform_node` parametresi `delay: 1.0s`. IMU ve GPS aynı anda başlarsa, ilk saniyede GPS/IMU zaman damgası uyuşmazlığı yanlış başlangıç pozisyonu verebilir. Araç bu hatalı pozisyon ile WP1'e giderse yanlış yönü görebilir.  
**Önerilen Düzeltme:** mission_manager başlangıcında `/odometry/filtered` kovaryans izini (covariance[0]) izle; değer makul olana kadar (örn. < 5.0 m²) bekle.

---

## 4. ALGORİTMA İYİLEŞTİRME ÖNERİLERİ

### 4.1 Aşama 1 için Saf Takip Rotası (Pure Pursuit) Alternatifi
Mevcut PID sürekli en yakın WP'yi hedefliyor. "Pure Pursuit" algoritması lookahead mesafesi ile çok daha yumuşak rota izler, aşım yapmaz. Kod değişikliği: ~50 satır.

### 4.2 GateFusion — Ağırlıklı Ortalama (Temporal Weighting)
Mevcut: `mean(son N ölçüm)`. Öneri: Üstel ağırlıklı ortalama (EMA, α=0.7). Eski ölçümlerin etkisi üssel azalır → dinamik sahnede daha hızlı adaptasyon.

### 4.3 Kamikaze Faz Geçişi — Mesafeye Dayalı P Kazancı
Mevcut: `ω = kp_yaw × err × 8.0` (sabit). Öneri: `ω = kp_yaw × err × (8.0 + 20.0 / max(dist, 0.5))` — yaklaşırken kazanç artar, çarpışma açısı hassasiyeti iyileşir.

### 4.4 Yedek Rota (Fallback Route) Tanımı Eksik
Aşama 1'de GPS kaybı, Aşama 2'de Nav2 planner başarısızlığı durumunda sistem ne yapacak? Şu an: sonsuz döngü veya sıfır hız. Yarışmada bu manuel müdahale gerektiriyor.  
**Öneri:** Her aşama için N saniye sonra önceki aşamaya geri dön veya güvenli duruş noktasına git mantığı.

---

## 5. KOD KALİTESİ VE SÜRDÜRÜLEBİLİRLİK

### 5.1 Sihirli Sayılar
Aşağıdaki sabitler doğrudan kodda yazılmış, parametre değil:

| Dosya | Sabit | Değer | Gerekçe |
|-------|-------|-------|---------|
| `mission_manager.py` | Faz 1a/1b hız | 0.2, v=base×3 | Tüm sürümler için sabit mi? |
| `mission_manager.py` | Gate consensus std | 1.0 m | Farklı sahalar için değişmeli |
| `kamikaze_control.py` | Lock timer | 3.0 s | Sahadaki hıza bağlı |
| `sensor_fusion.cpp` | Depth window | 11×11 px | Çözünürlüğe bağımlı |

### 5.2 Log Seviyeleri Tutarsız
Bazı kritik olaylar DEBUG, bazı rutin mesajlar WARN seviyesinde loglanıyor. Yarışmada log incelemesi zorlaşıyor.  
**Öneri:** Standart: bilgi=INFO, anormal ama kurtarılabilir=WARN, sistem durdurucu=ERROR.

### 5.3 `yolo_topic` Parametresi Artık Kullanılmıyor
`usv_autonomy.launch.py`'de `/yolo/detections` launch argümanı tanımlı ama hiçbir düğüme geçilmiyor — legacy kod.

---

## 6. EKSİK TEST SENARYOLARI

| Senaryo | Risk | Mevcut Durum |
|---------|------|--------------|
| ZED çökmesi sırasında Aşama 3 | Yüksek | Test edilmedi |
| GPS atlaması (5m sıçrama) sırasında Aşama 1 | Yüksek | Test edilmedi |
| MPPI plan bulamazsa (engel dolu costmap) | Orta | Test edilmedi |
| Yanlış sarı şamandıra tespiti (gürültü) | Orta | Test edilmedi |
| 2+ kırmızı şamandıra görünürse Kamikaze | Orta | Büyük alan seçme var |
| Araç sahadaki sınırın dışına çıkarsa | Orta | Sınır yok |

---

## 7. ACİL DÜZELTME ÖNCELİK LİSTESİ

| Öncelik | Sorun | Dosya | Tahmini Süre |
|---------|-------|-------|-------------|
| 🔴 P1 | MAVROS max_speed kırpması | `usv_autonomy.launch.py` | 5 dk |
| 🔴 P2 | static_transform gerçek frame isimleri | `static_transform.yaml` | 15 dk |
| 🟠 P3 | GateFusion kod tekrarı kaldır | mission_manager + parkur2 | 1 saat |
| 🟠 P4 | TF zincir başlangıç kontrolü | `mission_manager.py` | 30 dk |
| 🟡 P5 | xy_goal_tolerance daralt | `nav2_params_usv_pure.yaml` | 5 dk |
| 🟡 P6 | Kilit koşulunu mesafeye bağla | `kamikaze_control.py` | 20 dk |
| 🟡 P7 | Aşama 1 integral terimi ekle | `mission_manager.py` | 30 dk |

---

*Bu rapor kaynak kod statik analizi ile oluşturulmuştur. Dal: 2d-lidar-saha-testi*
