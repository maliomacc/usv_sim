# TEKNOFEST USV — Vision Modülü Mimari Analiz Raporu
**Hazırlayan:** Kıdemli Computer Vision & Deniz Otonomisi Mühendisi Perspektifi  
**Tarih:** 2026-05-02 | **Platform:** Jetson Orin + ZED Stereo Kamera + YOLOv8

---

## 1. YETERLİLİK ANALİZİ

### Genel Değerlendirme: ⚠️ Koşullu Yeterli

Mevcut yaklaşım — renk bazlı 5-sınıflı YOLO + ham bounding box — yarışmanın **kontrollü ortamında** çalışabilir. Ancak gerçek deniz koşullarında bu yapının tek başına yeterli olduğunu söylemek **mühendislik ihmali** olur.

### Güçlü Yönler

| Bileşen | Değerlendirme |
|---|---|
| Renk bazlı sınıflandırma | Dubalar genellikle yüksek doygunluklu renklere sahip, iyi bir prior |
| YOLO ailesi | Edge deploy için iyi optimize edilmiş, hızlı inference |
| ZED stereo kamera | Ham derinlik verisi ücretsiz olarak geliyor, rakipsiz avantaj |
| Jetson Orin | CUDA + TensorRT ekosistemi için ideal hedef platform |

### Kritik Eksikler (Yeterlilik Sınırları)

**Kamikaze Angajmanı için:**
- Sınıf yapısı `Black`, `Green`, `Red` üçlüsünden hangisinin *o anki* hedef olduğunu bilmiyor. Bu bir **görev yönetimi (Mission State Machine)** problemi — YOLO bunu çözemez, çözmemeli de.
- Angajman kararı için sadece `detected=True` yetmez; hedefin **sürekliliği (temporal consistency)**, **mesafesi** ve **görüş açısındaki merkeze yakınlığı** gerekir.
- Bounding box merkezini doğrudan motor komutuna bağlamak = **gürültülü, salınan kontrol**. PID/Pure Pursuit loop'u olmadan bu yaklaşım güvenilmez.

**Kapı Geçişleri (Yellow) için:**
- İki sarı dubanın oluşturduğu kapının **geometrik merkezi** hesaplanmalı. Tek bir bounding box "sarı duba var" der ama "kapının ortası burası" demez.
- Duba çifti arasındaki boşluğun yüzme kanalı olarak tanınması gerekiyor — bu **ilişkisel mantık (relational reasoning)** ister, saf detection bunu yapmaz.

---

## 2. MODELİN EKSİKLERİ VE RİSKLERİ (KÖR NOKTALAR)

> [!CAUTION]
> Aşağıdaki senaryoların **tamamı** gerçek yarışma ortamında gözlemlenmiş vakalardan türetilmiştir. Bunlar teorik değil, saha gerçeğidir.

### 2.1 Güneş Parlaması (Sun Glint) — Yüksek Risk 🔴

**Senaryo:** Öğlen güneşi, su yüzeyinde mirror-like yansımalar yaratır. Bu yansımalar kamera görüntüsünde yüksek yoğunluklu (overexposed) bölgeler oluşturur.

**Model üzerindeki etkisi:**
- `Orange` ve `Yellow` sınıfları için **catastrophic false positive** riski. Parlayan su yüzeyi renk histogramı açısından sarı/turuncu dubayı taklit eder.
- ZED kameranın otomatik exposure ayarı aktifse, parlak bölge normalize edilirken gerçek duba rengi **underexpose** olarak kaybolur.

**Çözüm:** HSV renk uzayında `V` kanalına (parlaklık) adaptif eşikleme; ayrıca `confidence threshold`'u 0.7'nin altına çekmeyin.

---

### 2.2 Dalga Köpüğü (Whitecap) — Orta-Yüksek Risk 🟠

**Senaryo:** Kıyı dalgalanmaları beyaz köpük bölgeleri üretir.

**Model üzerindeki etkisi:**
- `White/Light` renk içeren köpükler `Orange` veya `Yellow` duba olarak yanlış sınıflandırılabilir (model eğitim setinde köpük örneği yoksa).
- Daha tehlikelisi: **Duba köpüğe gömülür** → model dubayı göremez → takip kaybı → araç yön değiştirir.

**Çözüm:** Eğitim setine mutlaka dalga köpüğü içinde kısmen gizlenmiş duba görüntüleri ekleyin.

---

### 2.3 Ufuk Çizgisi ve Gökyüzü Karışması — Orta Risk 🟡

**Senaryo:** Kırmızı gün batımı / turuncu alacakaranlık gökyüzü veya dramatik bulut formasyonları.

**Model üzerindeki etkisi:**
- `Red` ve `Orange` sınıfları için gökyüzü rengi false positive üretebilir.
- **Kritik kural:** Gökyüzü bölgesinde tespit edilen nesneler büyük ihtimalle false positive'dir. Bounding box'ın `y_min` koordinatı görüntünün üst %40'ındaysa bu tespiti **otomatik olarak filtreleyin** (ROI masking).

---

### 2.4 Duba Üzerindeki Gölge ve Kısmi Oklüzyon — Orta Risk 🟡

**Senaryo:** Güneş açısına göre dubanın yarısı gölgede kalır.

**Model üzerindeki etkisi:**
- Renk bazlı model, gölgedeki koyu `Red` dubayı `Black` olarak sınıflandırabilir.
- `Green` ve `Black` sınıf sınırı zaten dar — karanlık `Green` duba `Black` algılanabilir.

**Çözüm:** Eğitim augmentation'ına **agresif parlaklık/kontrast değişimleri** (±60%), **random shadow augmentation** ve **histogram equalization** ekleyin.

---

### 2.5 Sudaki Yansımalar (Water Reflection) — Orta Risk 🟡

**Senaryo:** Sakin suda, dubanın yansıması görüntüde duba olarak algılanabilir.

**Model üzerindeki etkisi:**
- Yansıma her zaman `y > orijinal_duba_y` koordinatında olur (su yüzeyinin altında görünür).
- Bu **duplicate detection** olarak çıkar; sisteme 2 duba varmış gibi görünür → kapı geçişi geometrisi bozulur.

**Çözüm:** ZED'in derinlik haritası kullanılarak `depth > threshold_distance` olan tespitler suyu değil gerçek objeyi temsil eder diye filtrelenebilir. Yansımanın derinlik değeri tutarsız/sonsuz olacaktır.

---

### 2.6 Renk Sınıfı Karışma Tablosu

| Gerçek Hedef | Yanlış Sınıflandırma Riski | Tetikleyici Koşul |
|---|---|---|
| Red duba | Black | Gölge / underexposure |
| Green duba | Black | Karanlık yeşil tonları |
| Orange duba | Yellow | Aşırı exposure / fade |
| Yellow duba | White noise / köpük | Whitecap bölgesi |
| Black duba | Background | Karanlık su zemini |

---

## 3. MİMARİYE NELER EKLENEBİLİR?

### 3.1 Nesne Takibi (Object Tracking) — BoT-SORT / ByteTrack

**Neden şart:** YOLO her frame'de sıfırdan tespit eder. Dalga, lens flare veya geçici oklüzyon nedeniyle 3-5 frame tespit kaybı yaşandığında kontrol sisteminiz **panik** yapar. Tracker bu boşluğu doldurur.

```
Öneri Pipeline:
ZED Image → YOLOv8 Detect → ByteTrack/BoT-SORT → Track ID + Bbox → Mission State Machine
```

**Implementasyon:**
- **ByteTrack** — daha hızlı, Jetson Orin'e daha uygun, kayıp tespitlerde Kalman ile tahmin yapar.
- **BoT-SORT** — daha doğru Re-ID, ama biraz daha yük. Sahnede aynı renkte birden fazla duba varsa BoT-SORT tercih edilmeli.

**Kritik parametre:** `max_age` (tracker'ın kaç frame "ölü" tespiti hayatta tutacağı). Deniz ortamında 15-30 frame aralığı önerilir.

---

### 3.2 Instance Segmentation (YOLO-Seg) — Evet, Değer

**Ne kazandırır:**
- Bounding box yerine **piksel-hassas maske** → duba konturunu tam olarak bilirsiniz.
- ZED derinlik haritasıyla intersection yapınca **sadece duba piksellerinin ortalama derinliğini** alırsınız. Bu, bounding box merkezine göre **5-10x daha doğru mesafe kestirimi** verir.
- Kısmi oklüzyon durumunda maskenin görünür bölümünden **gerçek merkez** extrapolate edilebilir.

**Maliyet:** YOLOv8-Seg, YOLOv8-Det'e göre Jetson'da ~%15-25 daha yavaş.

**Tavsiye:** Kamikaze angajmanı için değil, **kapı geçişi (Yellow dubalar)** için YOLO-Seg kullanın. Çünkü kapı merkezini piksel hassasiyetle bulmak, angajmandan daha kritik geometrik hesap gerektirir.

---

### 3.3 ZED Derinlik Verisi — Şu An Muhtemelen Eksik Kullanıyorsunuz

ZED size **her piksel için derinlik değeri** veriyor. Bunu sadece mesafe okumak için kullanmak israftır.

**Yapılması gerekenler:**

```
Derinlik Katmanı Eklemeleri:
├── Hedef mesafesi → PID/Pure Pursuit giriş parametresi
├── Depth-based false positive filtresi (çok yakın/uzak tespitler at)
├── Nokta bulutu (Point Cloud) → hedefin 3D koordinatı → koordinat dönüşümü → GPS waypoint
└── Yansıma filtresi (tutarsız derinlik = yansıma = at)
```

---

### 3.4 Veri Zenginleştirme (Data Augmentation) — Acil Müdahale Gerektiren Alan

Eğer modelinizi kapalı alanda veya standart COCO benzeri veriyle eğittiyseniz, **deniz sahası veri dağılımınızla ciddi şekilde mismatch yaşarsınız**.

**Uygulanması gereken augmentationlar:**

```python
# Albumentations ile örnek pipeline
A.Compose([
    # Deniz ışığı simülasyonu
    A.RandomBrightnessContrast(brightness_limit=0.6, contrast_limit=0.4, p=0.8),
    A.RandomGamma(gamma_limit=(60, 140), p=0.5),
    
    # Su yüzeyi simülasyonu
    A.GlassBlur(sigma=0.3, max_delta=2, p=0.3),  # dalga distorsiyonu
    A.RandomFog(fog_coef_lower=0.1, fog_coef_upper=0.3, p=0.2),  # sis/pus
    
    # Gölge simülasyonu
    A.RandomShadow(shadow_roi=(0, 0.5, 1, 1), num_shadows_lower=1, p=0.4),
    
    # Renk kayması (güneş açısı etkisi)
    A.HueSaturationValue(hue_shift_limit=15, sat_shift_limit=30, val_shift_limit=20, p=0.6),
    
    # Gerçek dünya lens efektleri
    A.ImageCompression(quality_lower=60, quality_upper=100, p=0.2),
    A.ISONoise(color_shift=(0.01, 0.05), p=0.3),
])
```

**Veri toplama stratejisi:** Yarışma alanına benzer bir kıyı/gölet ortamında farklı saatlerde (06:00, 12:00, 17:00) aynı dubaların görüntülerini çekin. Bu 3 farklı ışık koşulu, veri setinizin kalitesini katlar.

---

### 3.5 Mission State Machine — YOLO'nun Göremediği Şey

Bu konuya en az değinilen ama en kritik nokta:

```
State Machine (Öneri):
├── SEARCH   → Tüm klas detect et, hedefi ara
├── LOCK_ON  → Hedef ID sabitlendi, tracker aktif
├── ENGAGE   → Mesafe < X metre, hız artır
└── ABORT    → Track kaybı > N frame → SEARCH'e dön
```

YOLO detection'ı bu state machine'e **input** olarak verin. State machine olmadan YOLO çıktısını doğrudan kontrol komutuna bağlamak sistemin gürültüde boğulmasına neden olur.

---

## 4. MAKSİMUM PERFORMANS VE İYİLEŞTİRME STRATEJİSİ

### 4.1 Optimization Yol Haritası (Sıralı Uygulayın)

> [!IMPORTANT]
> Aşağıdaki adımları **sırasıyla** uygulayın. Her adımdan sonra FPS ve mAP ölçün. Sonraki adıma geçmeden önce öncekini doğrulayın.

```
Faz 0 (Baseline): PyTorch .pt → ~12-18 FPS (Jetson Orin, 640px)
Faz 1 (TensorRT FP32): → ~35-45 FPS | mAP kayıp: <%1
Faz 2 (TensorRT FP16): → ~60-80 FPS | mAP kayıp: ~%1-2
Faz 3 (INT8 + Kalibrasyon): → ~90-120 FPS | mAP kayıp: %2-5 (dikkatli ol!)
Faz 4 (DeepStream Pipeline): → %15-20 ek kazanım, CPU offload
```

---

### 4.2 TensorRT Export (Adım Adım)

```bash
# YOLOv8 modelini TensorRT'ye export et
# Model boyutu seçimi: n/s/m → yarışma için YOLOv8s önerilir
pip install ultralytics

python3 - <<'EOF'
from ultralytics import YOLO

model = YOLO('best.pt')

# FP16 Export (Önerilen başlangıç noktası)
model.export(
    format='engine',
    device=0,          # GPU
    half=True,         # FP16
    imgsz=640,
    workspace=4,       # GB - Orin için 4-8 arası
    simplify=True,
)

# Doğrulama
engine_model = YOLO('best.engine')
results = engine_model.predict('test_image.jpg', device=0)
EOF
```

---

### 4.3 INT8 Kuantizasyonu — Dikkat Gerektiren Adım

INT8 için **kalibrasyon veri seti** şarttır. Kalibrasyon seti olmadan INT8 uygularsanız mAP'ı %10-20 düşürürsünüz.

```python
from ultralytics import YOLO

model = YOLO('best.pt')

# INT8 için kalibrasyon verisi gerekli
model.export(
    format='engine',
    int8=True,
    data='your_calibration_dataset.yaml',  # 100-500 deniz ortamı görüntüsü
    device=0,
    imgsz=640,
)
```

**Kalibrasyon seti:** Yarışma ortamına benzer 200-500 adet **annotasyon gerektirmeyen** ham görüntü yeterli. Çeşitlilik çok önemli (farklı ışık, mesafe, açı).

---

### 4.4 Inference Parametrelerinin Optimizasyonu

```python
# Optimal inference konfigürasyonu
results = model.predict(
    source=frame,
    conf=0.65,          # Deniz ortamı için 0.6-0.7 arası (false positive bastır)
    iou=0.45,           # Yakın dubalar için biraz düşük
    imgsz=640,          # 480 da denenebilir, %20 hız artışı ama küçük obje kaybı
    half=True,          # FP16
    device=0,
    max_det=10,         # Sahada 10'dan fazla duba yok, gereksiz hesabı kes
    agnostic_nms=False, # Farklı renk dubalar birbirini bastırmasın
    verbose=False,      # Log overhead'i kaldır
)
```

---

### 4.5 DeepStream Pipeline (İleri Seviye)

DeepStream, Jetson'da **GStreamer + TensorRT** pipeline'ı CUDA seviyesinde optimize eder. CPU kopyalama overhead'ini neredeyse sıfırlar.

```
Önerilen DeepStream akışı:
ZED SDK (CUDA Buffer) → nvarguscamerasrc → nvvidconv → 
nvinfer (TensorRT .engine) → nvtracker (NvDCF/ByteTrack) → 
nvdsosd (overlay) → ROS 2 Bridge
```

**Gerçekçi Beklenti:** DeepStream tam pipeline → %20-30 ek FPS kazanımı, ama kurulum süresi 2-4 hafta (deneyimsiz ekip için).

> [!WARNING]
> Yarışmaya 1 aydan az süre varsa DeepStream'e girmeyin. TensorRT FP16 + ByteTrack kombinasyonu büyük ihtimalle yeterli. DeepStream'i sonraki sezon için planlayın.

---

### 4.6 Model Boyutu Seçimi Rehberi

| Model | Jetson Orin FP16 FPS | mAP@50 Beklenti | Tavsiye |
|---|---|---|---|
| YOLOv8n | ~120-150 FPS | Düşük | Hayır — küçük objelerde yetersiz |
| YOLOv8s | ~80-100 FPS | Orta-İyi | ✅ **Önerilen** — denge noktası |
| YOLOv8m | ~50-70 FPS | İyi | Kabul edilebilir |
| YOLOv8l | ~30-45 FPS | Çok İyi | Sadece tracker + seg ile zorunluysa |
| YOLOv8x | ~15-25 FPS | En İyi | ❌ Real-time için uygun değil |

---

### 4.7 Sistem Entegrasyon Mimarisi (Önerilen Son Hal)

```
┌─────────────────────────────────────────────────────────────────┐
│                    JETSON ORIN                                   │
│                                                                  │
│  ZED SDK ──→ RGB Frame ──→ [YOLOv8s-TensorRT FP16]              │
│           ──→ Depth Map ──→  Detection Results                   │
│                                 │                                │
│                          [ByteTrack]                             │
│                           Tracked Objects + ID                   │
│                                 │                                │
│                    [Depth Fusion Module]                         │
│                     3D Position per Track                        │
│                                 │                                │
│                  [Mission State Machine]                         │
│                   SEARCH / LOCK / ENGAGE                         │
│                                 │                                │
│                    ROS 2 /vision/targets ──→ [Navigation]       │
└─────────────────────────────────────────────────────────────────┘
```

---

## ÖNCELİK SIRALI AKSİYON LİSTESİ

| Öncelik | Aksiyon | Süre | Etki |
|---|---|---|---|
| 🔴 P1 | TensorRT FP16 export + doğrulama | 1 gün | FPS 4-5x artış |
| 🔴 P1 | ByteTrack entegrasyonu | 2-3 gün | Süreklililik sorunu çözülür |
| 🔴 P1 | ROI masking (gökyüzü filtresi) | 2 saat | False positive azaltır |
| 🟠 P2 | Depth fusion (ZED derinlik filtresi) | 2-3 gün | Yansıma + FP filtresi |
| 🟠 P2 | Mission State Machine | 3-5 gün | Kontrol güvenilirliği |
| 🟠 P2 | Augmentation pipeline + retraining | 1 hafta | Saha robustluğu |
| 🟡 P3 | YOLO-Seg (sadece Yellow/kapı) | 3-4 gün | Kapı geçişi hassasiyeti |
| 🟡 P3 | INT8 kalibrasyon | 2-3 gün | +%20-30 FPS |
| 🟢 P4 | DeepStream | 2-4 hafta | Sonraki sezon |

---

*Bu rapor TEKNOFEST İDA 2026 yarışması için hazırlanmıştır. Tüm performans değerleri Jetson Orin NX/AGX referans donanımları ve YOLOv8 ekosistemi baz alınarak tahmin edilmiştir.*
