# Parkur 3 — Masa Başı Motor Testi Kılavuzu

**Mod:** `./start_all.sh saha_p3`  
**Amaç:** Tekneyi masaya sabitleyerek YOLO tespitini, faz geçişlerini ve motor tepkilerini gerçek donanımla doğrulamak.  
**Süre (ilk test):** ~45–60 dakika

---

## İçindekiler

1. [Bu Test Ne Doğrular?](#1-bu-test-ne-doğrular)
2. [Gereksinimler](#2-gereksinimler)
3. [Fiziksel Güvenlik Kurulumu](#3-fiziksel-güvenlik-kurulumu)
4. [Donanım Bağlantıları](#4-donanım-bağlantıları)
5. [Pixhawk Parametre Hazırlığı](#5-pixhawk-parametre-hazırlığı)
6. [Hız Parametresi Ayarı](#6-hız-parametresi-ayarı)
7. [Adım Adım Başlatma](#7-adım-adım-başlatma)
8. [Faz Davranışlarını Doğrulama](#8-faz-davranışlarını-doğrulama)
9. [İzleme Komutları](#9-izleme-komutları)
10. [Acil Durdurma](#10-acil-durdurma)
11. [Test Sonrası Kontrol Listesi](#11-test-sonrası-kontrol-listesi)
12. [Sık Karşılaşılan Sorunlar](#12-sık-karşılaşılan-sorunlar)

---

## 1. Bu Test Ne Doğrular?

| # | Test Konusu | Beklenen Sonuç |
|---|-------------|----------------|
| 1 | YOLO modeli (`son.engine`) çalışıyor mu? | `/kamikaze_target` topic'i 20 Hz yayın yapmalı |
| 2 | Faz 0 — Hedef yok → dönüş | `angular.z ≈ +1.5 rad/s`, `linear.x = 0` |
| 3 | Faz 1a — Hedef var, hizasız → tank dön | `linear.x = 0.2`, `angular.z` düzeltme |
| 4 | Faz 1b — Hedef ortalandı → hücum | `linear.x = base_speed × 3` |
| 5 | Faz 2 — 3 sn kilit → tam güç | `linear.x = base_speed × 5`, `/kamikaze_locked = true` |
| 6 | GUIDED+ARM kilidi çalışıyor mu? | GUIDED değilken `cmd_vel = 0` |
| 7 | MAVROS köprüsü motorlara ulaşıyor mu? | Gerçek pervane dönüşü |

---

## 2. Gereksinimler

### Donanım
- Jetson Orin NX (güçlü adaptörle, pil yeterli değil)
- Pixhawk (ArduRover firmware, USB veya UART)
- ZED Kamera (USB 3.0)
- ESC + motorlar bağlı ve test dublası (renkli top veya boya)

### Yazılım (çalışıyor olmalı)
```bash
# Workspace build edilmiş mi?
ls ~/sti_usv/install/setup.bash   # dosya yoksa: colcon build --symlink-install
```

### Masa üzerinde hazır olması gerekenler
- Kırmızı/yeşil/siyah renkli test dublası (en az 20 cm çaplı)
- Tekneyi sabitleyen bağ veya mengene
- 2. terminal (acil DISARM için)

---

## 3. Fiziksel Güvenlik Kurulumu

> ⚠️ **UYARI:** Motorlar gerçekten dönecek. Pervane varsa mutlaka koruyucu takın veya pervaneyi çıkarın.

### Tekneyi Sabitleme
```
[Masa]
  ├── Tekne gövdesi → ip/mengene ile masaya bağlı
  ├── Uzunluk: en az 40 cm bağ boşluğu bırak (ileri hareket simülasyonu)
  └── Yan bağ: teknenin yanlara kaymaması için 2 noktadan sabitle
```

**Minimum güvenli düzenek:**
1. Teknenin pruvasından ve kıçından birer ip
2. İpleri masanın kenarındaki mengenelere bağla
3. İpler gergin ama tel gibi değil — biraz esneklik bırak (10–15 cm)

### Pervane Kararı
| Durum | Öneri |
|-------|-------|
| İlk test | Pervaneyi çıkar, sadece ESC/motor sinyalini test et |
| Motor PWM doğrulaması | Pervane takılı, ama koruyucu kapak gerekli |
| Tam motor testi | Pervane takılı, tekne sıkıca bağlı, etrafta kimse yok |

---

## 4. Donanım Bağlantıları

```bash
# Kontrol et:
ls /dev/ttyACM*   # Pixhawk → /dev/ttyACM0
ls /dev/video*    # ZED → görünmeyebilir (SDK ile bağlanır)

# ZED bağlantısı USB 3.0 portunda mı?
lsusb | grep -i stereolabs
```

---

## 5. Pixhawk Parametre Hazırlığı

Masa başı test; kapalı alan (GPS yok), RC kumanda yok, GCS bağlantısı kesintili gibi koşulları içerir.
Varsayılan ArduRover parametreleriyle bu koşullarda **arm edilememe**, **beklenmedik failsafe** veya **otomatik disarm** sorunları yaşanır.
Aşağıdaki parametreler bu sorunları önlemek için masa başı testine özel ayarlanmalıdır.

---

### 5.1 Parametre Yedekleme (Önce Yap)

**QGroundControl ile:**
`Vehicle Setup → Parameters → Tools → Save to file`
→ `ardurover_saha_backup_YYYYMMDD.params` olarak kaydet

**MAVProxy ile:**
```bash
# MAVProxy bağlıyken:
param save ~/ardurover_saha_backup.parm
```

**MAVROS üzerinden (terminal):**
```bash
ros2 service call /mavros/param/pull std_srvs/srv/Empty
ros2 run mavros mavparam dump ~/ardurover_backup.yaml
```

---

### 5.2 Masa Başı İçin Değiştirilmesi Gereken Parametreler

#### Arming Kontrolleri

| Parametre | Sahada Değer | Masa Başı Değer | Açıklama |
|-----------|-------------|-----------------|----------|
| `ARMING_CHECK` | `1` (tümü) | `0` | Tüm arming ön koşullarını atla. GPS yok, barometresiz ortam için zorunlu. |
| `BRD_SAFETY_DEFLT` | `1` | `0` | Safety switch varsayılan pasif. Switch takılı değilse arm engelini kaldırır. |

> **Not:** `ARMING_CHECK = 0` tüm güvenlik kontrollerini devre dışı bırakır. Sadece masa başı testlerinde kullan, sahaya çıkmadan önce `1`'e geri al.

---

#### Failsafe Devre Dışı Bırakma

| Parametre | Sahada Değer | Masa Başı Değer | Açıklama |
|-----------|-------------|-----------------|----------|
| `FS_GCS_ENABLE` | `1` | `0` | GCS (QGC/MAVProxy) bağlantısı kopunca disarm/RTL yapmasın. MAVROS bağlantısı kesilirse tekne aniden duruyor. |
| `FS_GPS_ENABLE` | `1` | `0` | GPS sinyal kaybı failsafe'i. Kapalı alanda GPS fix yoktur, sürekli tetikler. |
| `FS_CRASH_CHECK` | `1` | `0` | Çarpışma/takla tespiti. Masa titreşimi veya ani hız değişimi bunu tetikleyebilir. |
| `FS_THR_ENABLE` | `1` | `0` | Throttle/RC failsafe. RC kumanda bağlı değilse arm olur olmaz failsafe girer. |

---

#### Otomatik Disarm

| Parametre | Sahada Değer | Masa Başı Değer | Açıklama |
|-----------|-------------|-----------------|----------|
| `DISARM_DELAY` | `10` | `0` | `0` = otomatik disarm yok. Varsayılan 10 sn'de araç hareketsizse disarm olur; test sırasında Faz 0'da (arama dönüşü) sorun çıkarır. |

---

#### Motor Güç Limiti (Güvenlik)

| Parametre | Sahada Değer | Masa Başı Değer | Açıklama |
|-----------|-------------|-----------------|----------|
| `THR_MAX` | `100` | `30` | Maksimum gaz yüzdesi. Masa başında %30 üzerini kilitle. Tekne bağı koparsa daha az tehlikeli. |
| `MOT_SPIN_ARM` | `0.1` | `0.0` | Arm edilince motorlar düşük güçte dönmeye başlar. Masa başında `0.0` yapılırsa arm anında motor ses çıkarmaz, komut gelince döner. |

---

#### GUIDED Mod Hız Limiti

| Parametre | Sahada Değer | Masa Başı Değer | Açıklama |
|-----------|-------------|-----------------|----------|
| `WP_SPEED` | `2.5` | `0.5` | GUIDED modda maksimum hedef hız (m/s). `cmd_vel`'den daha yüksek değer gelse bile bu değerde sınırlanır. İkinci bir güvenlik katmanı. |
| `ATC_STR_RAT_MAX` | `180` | `60` | Maksimum dönüş hız sınırı (deg/s). Masa başında ani dönüşleri sınırlar. |

---

### 5.3 Parametreleri Nasıl Ayarlarsın?

#### Yöntem A — MAVROS üzerinden (terminal, MAVROS çalışırken)

```bash
# Tek parametre set:
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'ARMING_CHECK', value: {integer_value: 0}}"

# Tüm masa başı parametrelerini sırayla:
for PARAM_VAL in \
  "ARMING_CHECK:0" \
  "BRD_SAFETY_DEFLT:0" \
  "FS_GCS_ENABLE:0" \
  "FS_GPS_ENABLE:0" \
  "FS_CRASH_CHECK:0" \
  "FS_THR_ENABLE:0" \
  "DISARM_DELAY:0" \
  "THR_MAX:30" \
  "MOT_SPIN_ARM:0.0" \
  "WP_SPEED:0.5" \
  "ATC_STR_RAT_MAX:60"
do
  PARAM=$(echo $PARAM_VAL | cut -d: -f1)
  VAL=$(echo $PARAM_VAL | cut -d: -f2)
  echo "Setting $PARAM = $VAL"
  ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
    "{param_id: '${PARAM}', value: {integer_value: ${VAL}}}" 2>/dev/null || \
  ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
    "{param_id: '${PARAM}', value: {double_value: ${VAL}}}" 2>/dev/null
  sleep 0.3
done
```

#### Yöntem B — MAVProxy üzerinden

```bash
# MAVProxy bağlantısı:
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200

# Parametre listesi yükle:
param load ~/sti_usv/src/usv_sim/config/masa_basi_test.parm

# Veya tek tek:
param set ARMING_CHECK 0
param set FS_GCS_ENABLE 0
param set FS_GPS_ENABLE 0
param set FS_CRASH_CHECK 0
param set FS_THR_ENABLE 0
param set DISARM_DELAY 0
param set THR_MAX 30
param set MOT_SPIN_ARM 0.0
param set WP_SPEED 0.5
param set ATC_STR_RAT_MAX 60
param set BRD_SAFETY_DEFLT 0

# Kalıcı kaydet:
param save
```

#### Yöntem C — Hazır `.parm` Dosyası ile

Aşağıdaki içeriği `~/sti_usv/src/usv_sim/config/masa_basi_test.parm` olarak kaydet:

```
# Masa Basi Test Parametreleri — ArduRover
# Sahaya cikmadan once geri yukle: saha_parametreleri.parm
ARMING_CHECK        0
BRD_SAFETY_DEFLT    0
FS_GCS_ENABLE       0
FS_GPS_ENABLE       0
FS_CRASH_CHECK      0
FS_THR_ENABLE       0
DISARM_DELAY        0
THR_MAX             30
MOT_SPIN_ARM        0.0
WP_SPEED            0.5
ATC_STR_RAT_MAX     60
```

MAVProxy ile yükle:
```bash
param load ~/sti_usv/src/usv_sim/config/masa_basi_test.parm
param save
```

---

### 5.4 Parametre Doğrulama

Parametreler yazıldı mı kontrol et:

```bash
# MAVROS üzerinden oku:
ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet \
  "{param_id: 'ARMING_CHECK'}"

# MAVProxy üzerinden:
param show ARMING_CHECK
param show FS_GCS_ENABLE
param show DISARM_DELAY
param show THR_MAX
```

---

### 5.5 Sahaya Dönmeden Önce Geri Al

> ⚠️ **KESİNLİKLE UNUTMA:** Bu parametrelerle sahaya çıkma. GPS failsafe kapalı, crash check kapalı, THR_MAX düşük.

```bash
# Yedek dosyadan geri yükle (MAVProxy):
param load ~/ardurover_saha_backup.parm
param save

# Veya kritik parametreleri elle geri al:
param set ARMING_CHECK 1
param set FS_GCS_ENABLE 1
param set FS_GPS_ENABLE 1
param set FS_CRASH_CHECK 1
param set FS_THR_ENABLE 1
param set DISARM_DELAY 10
param set THR_MAX 100
param set MOT_SPIN_ARM 0.1
param set WP_SPEED 2.5
param set ATC_STR_RAT_MAX 180
param set BRD_SAFETY_DEFLT 1
param save
```

---

### 5.6 Parametre Özet Tablosu

| Parametre | Sahada | Masa Başı | Değiştirme Nedeni |
|-----------|--------|-----------|-------------------|
| `ARMING_CHECK` | 1 | **0** | GPS yok, kapalı alan |
| `BRD_SAFETY_DEFLT` | 1 | **0** | Safety switch yok |
| `FS_GCS_ENABLE` | 1 | **0** | MAVROS kesintisi disarm yapmasın |
| `FS_GPS_ENABLE` | 1 | **0** | İç mekanda GPS fix yok |
| `FS_CRASH_CHECK` | 1 | **0** | Masa titreşimi tetiklemesin |
| `FS_THR_ENABLE` | 1 | **0** | RC yok |
| `DISARM_DELAY` | 10 | **0** | Test sırasında otomatik disarm olmasın |
| `THR_MAX` | 100 | **30** | Motor gücü sınırla |
| `MOT_SPIN_ARM` | 0.1 | **0.0** | Arm anında motor dönmesin |
| `WP_SPEED` | 2.5 | **0.5** | Guided max hız limiti |
| `ATC_STR_RAT_MAX` | 180 | **60** | Dönüş hız sınırı |

---

## 6. Hız Parametresi Ayarı (ROS Tarafı)

**Masa başı test için `base_speed` değerini düşür.** Varsayılan `1.5 m/s` masada tehlikelidir.

| Senaryo | `base_speed` | Faz-1 hızı | Faz-2 hızı |
|---------|--------------|-----------|-----------|
| Pervane yok (sadece ESC testi) | `1.5` (varsayılan) | 4.5 m/s | 7.5 m/s |
| Pervane var, ilk test | `0.3` | 0.9 m/s | 1.5 m/s |
| Pervane var, güvenli test | `0.5` | 1.5 m/s | 2.5 m/s |

```bash
# Düşük hızla başlat:
BASE_SPEED=0.3 ./start_all.sh saha_p3

# Veya belirli renk + hız:
BASE_SPEED=0.3 TARGET_COLOR=1 ./start_all.sh saha_p3
```

---

## 7. Adım Adım Başlatma

> **Ön koşul:** Bölüm 5'teki Pixhawk parametreleri ayarlanmış olmalı.

### Terminal 1 — MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
```

MAVROS bağlandı mı?
```bash
ros2 topic echo /mavros/state --once
# armed: false  mode: "MANUAL"  → bağlantı var
```

### Terminal 2 — ZED Kamera

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

ZED çalışıyor mu?
```bash
ros2 topic hz /zed/zed_node/rgb/image_rect_color
# 30 Hz civarı görünmeli
```

### Terminal 3 — saha_p3 Başlat

```bash
cd ~/sti_usv/src/usv_sim

# Kırmızı duba, düşük hız:
BASE_SPEED=0.3 TARGET_COLOR=0 ./start_all.sh saha_p3
```

Başarılı çıktı şu satırları içermeli:
```
Kamikaze Gözcü başlatılıyor (model: .../son.engine)...
  └─ Kamikaze Gözcü PID: XXXX
Parkur 3 Standalone başlatılıyor...
  └─ Parkur3 Standalone PID: XXXX
[SAHA] cmd_vel → MAVROS köprüsü başlatılıyor...
```

### Terminal 4 — Acil DISARM (hazır beklesin)

```bash
# Bu komutu kopyala, gerektiğinde Enter'a bas:
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

### Terminal 5 — İzleme

```bash
ros2 topic echo /cmd_vel
```

---

## Pixhawk'ı GUIDED Moda Alma ve ARM

> ⚠️ Bu adımdan sonra motorlar komutu kabul etmeye başlar.

### Adım 1: GUIDED Moda Al

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
```

Doğrula:
```bash
ros2 topic echo /mavros/state --once
# mode: "GUIDED"  ← bu görünmeli
```

### Adım 2: ARM Et

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

Doğrula:
```bash
ros2 topic echo /mavros/state --once
# armed: true  mode: "GUIDED"  ← her ikisi de görünmeli
```

### Adım 3: node'un fark ettiğini kontrol et

```bash
ros2 topic echo /mission_state
# "PARKUR3_STANDALONE" yayınlanıyor olmalı
```

---

## 8. Faz Davranışlarını Doğrulama

### Faz 0 — Hedef Yok (Arama)

**Koşul:** ZED kamerayı boş bir yöne doğrult (duba görünmesin).

**Beklenen `/cmd_vel`:**
```
linear:
  x: 0.0
angular:
  z: 1.5   ← sola dönüş
```

**Motor tepkisi:** Sol motor ileri, sağ motor geri (yerinde dönüş).

**Log:**
```
[KAMIKAZE] FAZ-0 ARAMA — 2.3s hedef yok, dönüyorum…
```

---

### Faz 1a — Hedef Görüldü, Hizalanıyor

**Koşul:** Kırmızı dubayı kameranın sağ köşesine tut.

**Beklenen `/cmd_vel`:**
```
linear:
  x: 0.2
angular:
  z: +2.x   ← sola dön (hedef sağda, sola düzelt)
```

**Log:**
```
[KAMIKAZE] FAZ-1 HİZALANIYOR — err=+0.28 ω=+2.45
```

> `err > 0.15` olduğu sürece bu fazda kalır.

---

### Faz 1b — Hedef Ortalandı, Hücum

**Koşul:** Dubayı kameranın tam ortasına getir (`err ≤ 0.15`).

**Beklenen `/cmd_vel`:**
```
linear:
  x: ~0.9   (base_speed=0.3 ile 0.3×3=0.9, min 1.5 yerine BASE_SPEED=0.3 ile max(1.5,0.9) → 1.5)
angular:
  z: küçük düzeltme
```

> **Not:** Kodda `max(1.5, adaptive_speed)` var. `base_speed=0.3` bile olsa `linear.x` minimum `1.5` m/s'dir.  
> Bunu aşmak için `BASE_SPEED=0.5` ve `max(1.5,...)` kısmını geçici olarak düşürmek gerekir.  
> Masa başı için pervane çıkarmayı tercih et.

---

### Faz 2 — Kilit (3 Saniye Sonra)

**Koşul:** Dubayı tam ortada tut, 3 saniye bekle.

**Beklenen `/kamikaze_locked`:**
```
data: true
```

**Beklenen `/cmd_vel`:**
```
linear:
  x: 1.5   (base_speed=0.3 × 5)
angular:
  z: ~0.0  (tam kilitliyse)
```

**Log:**
```
[KİLİT] /kamikaze_locked=True alındı — FAZ 2 başlıyor!
[KAMIKAZE] FAZ-2 KILL PHASE! err=-0.02 v=1.5m/s ω=-0.03rad/s
```

---

## 9. İzleme Komutları

```bash
# Faz izleme (hangi fazda olduğunu görmek için)
ros2 topic echo /mission_state

# cmd_vel (her faz geçişinde değişmeli)
ros2 topic echo /cmd_vel

# YOLO tespiti (x=0.5 = merkez, z = alan px²)
ros2 topic echo /kamikaze_target

# Kilit sinyali
ros2 topic echo /kamikaze_locked

# ArduRover durumu
ros2 topic echo /mavros/state

# YOLO 20 Hz çalışıyor mu?
ros2 topic hz /kamikaze_target

# Tüm topic'ler bir arada (tmux önerilir)
watch -n 0.5 "ros2 topic echo /cmd_vel --once 2>/dev/null | grep -E 'x:|z:'"
```

---

## 10. Acil Durdurma

**Öncelik sırası:**

### 1. Hızlı DISARM (Terminal 4'ten)
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

### 2. start_all.sh'yi durdur
```bash
# Terminal 3'te:
Ctrl + C
```
Cleanup fonksiyonu tüm node'ları öldürür: `kamikaze_control`, `parkur3_standalone`, `cmd_vel_to_mavros`.

### 3. Manuel mod (Pixhawk'ta RC varsa)
RC kumandadan MANUAL moduna geç → tüm otonom kontrol devre dışı.

### 4. Son çare
```bash
pkill -f "parkur3_standalone" && pkill -f "kamikaze_control" && pkill -f "cmd_vel_to_mavros"
```

---

## 11. Test Sonrası Kontrol Listesi

```
[ ] Faz 0 doğrulandı: hedef yokken angular.z = 1.5 rad/s
[ ] Faz 1a doğrulandı: hizalanma dönüşü görüldü
[ ] Faz 1b doğrulandı: ileri hareket başladı
[ ] Faz 2 doğrulandı: /kamikaze_locked=true alındı
[ ] GUIDED kilidi çalışıyor: GUIDED değilken cmd_vel = 0 görüldü
[ ] MAVROS köprüsü motorlara ulaştı (ESC sinyal aldı)
[ ] Acil DISARM test edildi
[ ] base_speed sahada kullanılacak değere geri yüklendi (1.5)

[ ] ── PIXHAWK PARAMETRELERİ GERİ ALINDI (SAHAYA ÇIKMADAN ÖNCE ZORUNLU) ──
[ ] ARMING_CHECK    → 1
[ ] BRD_SAFETY_DEFLT → 1
[ ] FS_GCS_ENABLE   → 1
[ ] FS_GPS_ENABLE   → 1
[ ] FS_CRASH_CHECK  → 1
[ ] FS_THR_ENABLE   → 1
[ ] DISARM_DELAY    → 10
[ ] THR_MAX         → 100
[ ] MOT_SPIN_ARM    → 0.1
[ ] WP_SPEED        → 2.5
[ ] ATC_STR_RAT_MAX → 180
[ ] param save yapıldı (kalıcı kaydedildi)
```

---

## 12. Sık Karşılaşılan Sorunlar

### cmd_vel geliyor ama motor dönmüyor

```bash
# MAVROS bağlantısı var mı?
ros2 topic echo /mavros/state --once
# → armed: false ise ARM edilmedi

# Köprü çalışıyor mu?
ros2 node list | grep cmd_vel_to_mavros
```

### YOLO modeli yüklenmiyor

```bash
# Model dosyası var mı?
ls ~/sti_usv/src/usv_sim/workspace_ros/models/son.engine

# kamikaze_control logu:
ros2 node info /kamikaze_control
```

`son.engine` yoksa → modeli doğru klasöre kopyala:
```bash
cp /path/to/son.engine ~/sti_usv/src/usv_sim/workspace_ros/models/
```

### /kamikaze_target yayınlanmıyor

```bash
ros2 topic hz /kamikaze_target
# 0 Hz → YOLO çalışmıyor

ros2 topic hz /zed/zed_node/rgb/image_rect_color
# 0 Hz → ZED bağlı değil veya driver çalışmıyor
```

### Hedef görülüyor ama Faz 2 gelmiyor

Kilit için duba **3 saniye kesintisiz** görünmeli. Kamera sallantılı veya tespit güven skoru düşük olabilir:
```bash
# Tespit kalitesini izle:
ros2 topic echo /kamikaze_target
# z değeri (alan px²) küçükse duba çok uzakta veya küçük
```

### "bypass_guided=True" hata mesajı çıkıyor ama bypass açılmadı

```bash
ros2 param get /parkur3_standalone bypass_guided_check
# false olmalı — motorları test ediyorsan True OLMAMALI
```

---

### Pixhawk ARM edilemiyor

```
PreArm: Need 3D Fix
PreArm: GPS speed error
PreArm: Compass not calibrated
```

**Çözüm:** `ARMING_CHECK = 0` parametresi yazılmamış.
```bash
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'ARMING_CHECK', value: {integer_value: 0}}"
```

---

### ARM edildi ama hemen disarm oluyor

**Sebep 1 — `DISARM_DELAY` hâlâ 10:**
```bash
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'DISARM_DELAY', value: {integer_value: 0}}"
```

**Sebep 2 — `FS_GCS_ENABLE` hâlâ 1:** MAVROS terminal kapanınca GCS bağlantısı kopuyor, failsafe disarm yapıyor.
```bash
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'FS_GCS_ENABLE', value: {integer_value: 0}}"
```

---

### GUIDED moda girilemiyor / hemen MANUAL'e dönüyor

**Sebep:** Bazı ArduRover sürümlerinde GPS fix olmadan GUIDED mod kabul edilmez.  
**Çözüm:** `FS_GPS_ENABLE = 0` yap ve `ARMING_CHECK = 0` ile birlikte dene.

```bash
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'FS_GPS_ENABLE', value: {integer_value: 0}}"
```

---

### cmd_vel geliyor, MAVROS'a ulaşıyor ama motor dönmüyor

**Sebep:** `MOT_SPIN_ARM = 0` ve `THR_MAX = 30` ayarında düşük cmd_vel PWM eşiğini aşamıyor olabilir.  
**Kontrol:** ESC'ye gelen PWM sinyalini ölç (servo tester veya osiloskop).  
**Geçici çözüm:** `THR_MAX = 50` yap, komutun yeterli PWM üretip üretmediğini test et.

---

### Masa titreşimi nedeniyle Faz 2 kilit bozuluyor

`FS_CRASH_CHECK = 1` iken masa titreşimi ArduRover'ı disarm edebilir.  
Parametrenin `0` olduğunu doğrula:
```bash
ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet \
  "{param_id: 'FS_CRASH_CHECK'}"
# integer_value: 0 görünmeli
```

---

## Hızlı Referans Kartı

```
TERMINAL DÜZENİ (tmux önerilir)
┌─────────────────┬─────────────────┐
│  T1: MAVROS     │  T2: ZED SDK    │
├─────────────────┼─────────────────┤
│  T3: start_all  │  T4: ACİL DISM  │
├─────────────────┴─────────────────┤
│  T5: ros2 topic echo /cmd_vel     │
└───────────────────────────────────┘

PIXHAWK HAZIRLIK SIRASI:
  param backup → masa_basi_test.parm yükle
  → ARMING_CHECK=0, FS_*=0, DISARM_DELAY=0, THR_MAX=30

ARM SIRASI:
  set_mode GUIDED → arming true → duba göster

STOP SIRASI:
  arming false → Ctrl+C (T3)

SAHAYA ÇIKMADAN ÖNCE:
  saha_backup.parm yükle → param save → tüm FS parametreleri 1'e aldın mı kontrol et
```
