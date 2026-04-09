● ---                                                                                                                                                         
  TEKNOFEST USV — Tam Sistem Rehberi                                                                                                                          
                                                                                                                                                              
  Genel Mimari                                                                                                                                                
                                                                                                                                                              
  ┌─────────────────────────────────────────────────────────────────┐                                                                                         
  │                    IGNITION GAZEBO (Simülasyon)                 │                                                                                         
  │  ┌──────────┐  ┌──────────────┐  ┌───────────┐  ┌──────────┐  │                                                                                           
  │  │ RPLidar  │  │  ZED 1.0     │  │   GPS     │  │   IMU    │  │                                                                                           
  │  │ A1M8 2D  │  │ rgbd_camera  │  │ (NavSat)  │  │          │  │                                                                                           
  │  └────┬─────┘  └──────┬───────┘  └─────┬─────┘  └────┬─────┘  │                                                                                           
  └───────┼────────────────┼────────────────┼──────────────┼────────┘                                                                                         
          │ ros_gz_bridge  │                │              │                                                                                                  
          ▼                ▼                ▼              ▼                                                                                                  
      /scan           /zed/depth/*     /roboboat/    /roboboat/                                                                                               
                      /zed/odom        sensors/gps   sensors/imu                                                                                              
          │                                                                                                                                                   
      laser_filters                       │              │                                                                                                    
          │                               └──────┬───────┘                                                                                                    
          ▼                                      ▼             
     /scan/filtered                       robot_localization (EKF)                                                                                            
          │                                      │                                                                                                            
          ├──────────────────────────────────────▼
          │                               /odometry/filtered                                                                                                  
          ▼                                      │             
     slam_toolbox (2D SLAM)                      │                                                                                                            
          │                                      │                                                                                                            
          ▼                                      ▼
        /map ←─────────────────────── TF: map→odom→base_link                                                                                                  
          │                                                                                                                                                   
          ▼
     ┌─────────────────────────────────────────────────┐                                                                                                      
     │              Nav2 Stack                         │                                                                                                      
     │  local_costmap: /scan/filtered + /zed/depth/points │
     │  global_costmap: /scan                          │                                                                                                      
     │  controller: RegulatedPurePursuit               │       
     └─────────────────────────────────────────────────┘                                                                                                      
          │                                                    
          ▼                                                                                                                                                   
     /cmd_vel → velocity_smoother → collision_monitor → thrusters                                                                                             
                                                                                                                                                              
     ┌─────────────────────────────────────────────────┐                                                                                                      
     │           Görev Katmanı                         │                                                                                                      
     │  mission_manager.py → PID | MPPI | Kamikaze     │                                                                                                      
     │  kamikaze_control.py → HSV + LiDAR + ZED depth  │                                                                                                      
     └─────────────────────────────────────────────────┘                                                                                                      
                                                                                                                                                              
  ---                                                                                                                                                         
  Parkur Görev Makinesi (3 Aşama)                              
                                                                                                                                                              
  ┌──────────┬─────────────────────┬───────────────────────────────────────────┬───────────────┐
  │  Aşama   │       Script        │                 Algoritma                 │ Bitiş Koşulu  │                                                              
  ├──────────┼─────────────────────┼───────────────────────────────────────────┼───────────────┤
  │ Parkur 1 │ mission_manager.py  │ PID + GPS waypoint                        │ WP4'e ulaş    │                                                              
  ├──────────┼─────────────────────┼───────────────────────────────────────────┼───────────────┤
  │ Parkur 2 │ kamikaze_control.py │ HSV sarı kapı tespiti + MPPI              │ WP5'e ≤3m     │                                                              
  ├──────────┼─────────────────────┼───────────────────────────────────────────┼───────────────┤                                                              
  │ Parkur 3 │ kamikaze_control.py │ HSV kırmızı/siyah duba + LiDAR+ZED mesafe │ Görev tamamla │                                                              
  └──────────┴─────────────────────┴───────────────────────────────────────────┴───────────────┘                                                              
                                                               
  ---                                                                                                                                                         
  Sensör Verileri — Nasıl Görüntülenir                         
                                                                                                                                                              
  Terminal ile Hz Kontrolü
                                                                                                                                                              
  # Her sensörü tek satırda kontrol et                         
  ros2 topic hz /scan                    # RPLidar ham  → ~7 Hz                                                                                               
  ros2 topic hz /scan/filtered           # Filtreli     → ~7 Hz                                                                                               
  ros2 topic hz /camera/image            # RGB kamera   → ~10 Hz                                                                                              
  ros2 topic hz /zed/depth/image         # ZED derinlik → ~5-10 Hz                                                                                            
  ros2 topic hz /zed/depth/points        # PointCloud2  → ~5-10 Hz                                                                                            
  ros2 topic hz /zed/depth/camera_info   # Depth CI     → ~5-10 Hz  ← yeni eklendi                                                                            
  ros2 topic hz /zed/odom                # Visual odom  → ~30 Hz                                                                                              
  ros2 topic hz /odometry/filtered       # EKF çıkışı  → ~10 Hz
  ros2 topic hz /map                     # SLAM haritası → ~0.5 Hz                                                                                            
  ros2 topic hz /cmd_vel                 # Hız komutu   → değişken                                                                                            
                                                                                                                                                              
  Topic İçeriği Okuma                                                                                                                                         
                                                                                                                                                              
  # Görev durumu (hangi parkurda?)                             
  ros2 topic echo /mission_state                                                                                                                              
                                                                                                                                                              
  # Bot GPS konumu                                                                                                                                            
  ros2 topic echo /roboboat/sensors/gps/navsat --once                                                                                                         
                                                                                                                                                              
  # EKF konumu (x,y,yaw)
  ros2 topic echo /odometry/filtered --once                                                                                                                   
                                                                                                                                                              
  # ZED odometry (ground truth konum)
  ros2 topic echo /zed/odom --once                                                                                                                            
                                                               
  # Kamikaze algılama sonucu                                                                                                                                  
  ros2 topic echo /kamikaze_target --once
  ros2 topic echo /kamikaze_locked --once      # true=hedef kilitlendi                                                                                        
  ros2 topic echo /yellow_visible --once      # true=sarı kapı görünüyor                                                                                      
                                                                                                                                                              
  # Thruster güçleri                                                                                                                                          
  ros2 topic echo /roboboat/thrusters/left/thrust --once                                                                                                      
  ros2 topic echo /roboboat/thrusters/right/thrust --once                                                                                                     
  
  ---                                                                                                                                                         
  RViz2 ile Görselleştirme                                     
                                                                                                                                                              
  Sistemi görsel takip için:
  rviz2 &                                                                                                                                                     
                                                               
  Eklenecek Display'ler ve topic'leri:                                                                                                                        
                                                                                                                                                              
  ┌──────────────┬─────────────────────────┬──────────────────────────────────────┐                                                                           
  │ Display Türü │          Topic          │             Ne Gösterir              │                                                                           
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Image        │ /camera/image           │ RGB kamera (HSV algılama buradan)    │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤
  │ Image        │ /zed/depth/image        │ ZED derinlik görüntüsü               │                                                                           
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ LaserScan    │ /scan                   │ Ham RPLidar taraması                 │                                                                           
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ LaserScan    │ /scan/filtered          │ Filtreli LiDAR                       │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Map          │ /map                    │ SLAM haritası                        │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Costmap      │ /local_costmap/costmap  │ Yerel engel haritası                 │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Costmap      │ /global_costmap/costmap │ Global rota haritası                 │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Odometry     │ /odometry/filtered      │ EKF konum+hız                        │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Odometry     │ /zed/odom               │ ZED visual odometry                  │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ PointCloud2  │ /zed/depth/points       │ ZED nokta bulutu                     │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ Path         │ /plan                   │ Nav2 planlanan rota                  │
  ├──────────────┼─────────────────────────┼──────────────────────────────────────┤                                                                           
  │ TF           │ —                       │ Tüm frame'ler (base_link, odom, map) │
  └──────────────┴─────────────────────────┴──────────────────────────────────────┘                                                                           
                                                               
  ▎ Not: /zed/depth/image görüntülerken Camera Info Topic alanını /zed/depth/camera_info yap (simülasyonu restart ettikten sonra gelecek).                    
                                                               
  ---                                                                                                                                                         
  Smart Fallback — Mesafe Zinciri                              
                                 
  kamikaze_control.py içinde hedef mesafesi şu sırayla hesaplanır:
                                                                                                                                                              
  1. Açı hesapla (piksel merkezinden)
        ↓                                                                                                                                                     
  2. LiDAR: o açıda /scan'dan mesafe oku (_safe_lidar_dist)    
        ↓ (LiDAR None döndürürse)                                                                                                                             
  3. ZED: derinlik görüntüsünden piksel mesafesi oku (_zed_depth_at_pixel)                                                                                    
        ↓ (ZED de None ise)                                                                                                                                   
  4. Fallback sabit mesafe (2.5m)                                                                                                                             
                                                                                                                                                              
  HUD'da hangi kaynağın kullanıldığı görünür:                                                                                                                 
  - Kaynak: LIDAR — LiDAR başarılı                                                                                                                            
  - Kaynak: ZED — LiDAR yoktu, ZED kullanıldı                                                                                                                 
  - ZED DEPTH: OK/YOK — ZED depth stream durumu                
                                                                                                                                                              
  ---                                                                                                                                                         
  Başlatma Modları                                                                                                                                            
                                                                                                                                                              
  cd ~/sti_usv/src/usv_sim                                     
                                                                                                                                                              
  ./start_all.sh          # Sadece Gazebo + sensörler                                                                                                         
  ./start_all.sh slam2d   # TAM SİSTEM ← yarışma için bu                                                                                                      
  ./start_all.sh mola     # MOLA SLAM (4D LiDAR için)                                                                                                         
  ./start_all.sh parkur   # Haritasız reaktif sürüş                                                                                                           
                                                                                                                                                              
  ---                                                                                                                                                         
  Nav2 Costmap Sensör Füzyonu                                                                                                                                 
                                                               
  local_costmap iki kaynaktan engel alır (nav2_params.yaml):
                                                                                                                                                              
  /scan/filtered    → LaserScan  → yatay düzlem engelleri
  /zed/depth/points → PointCloud2 → yüksek engeller (0.1m–2.5m arası)                                                                                         
                                                                                                                                                              
  Costmap'i terminal'de izlemek:                                                                                                                              
  # Costmap güncellenme hızı                                                                                                                                  
  ros2 topic hz /local_costmap/costmap                                                                                                                        
                                                                                                                                                              
  # Nav2 tüm node durumları
  ros2 node list | grep -E "nav2|slam|ekf"                                                                                                                    
                                                               
                                                               
                                                               
                                                                                                                                                                           
                                                               
😄️❯ hangi zed verileri füzyonda kullanılıyor sistemde bulunan füzyonların hepsinin tam olarak mantığını anlat lütfen  🥰️                                          
                                                                                                                   
  
  
                                                                                                                                               
● Artık tüm füzyon katmanlarının tam resmini görebildim. Sistemde 3 ayrı, iç içe füzyon var:                                                                  
                                                                                                                                                              
  ---                                                                                                                                                         
  Sistemdeki Tüm Füzyonlar                                                                                                                                    
                                                                                                                                                              
  FÜZYON 1 — Lokalizasyon: GPS + IMU → Konum Tahmini (EKF)                                                                                                    
                                                                                                                                                              
  Ne: robot_localization paketi, Extended Kalman Filter (EKF)                                                                                                 
  Dosya: workspace_ros/config/ekf.yaml                                                                                                                        
  Çıktı: /odometry/filtered + odom→base_link TF                                                                                                               
                                                                                                                                                              
  /gps/fixed_cov    ──► EKF ──► /odometry/filtered                                                                                                            
  /imu/fixed_cov    ──►     ──► odom→base_link TF                                                                                                             
  (GPS → navsat_transform → /odometry/gps → EKF girişi)                                                                                                     
                                                                                                                                                              
  EKF hangi bitleri kime güveniyor:                                                                                                                           
                                                                                                                                                              
  ┌──────────────────────┬──────────────────────────────────┬────────────────────────┐                                                                        
  │        Sensör        │        Güvenilen Alanlar         │    Göz Ardı Edilen     │                                                                      
  ├──────────────────────┼──────────────────────────────────┼────────────────────────┤                                                                        
  │ GPS (/odometry/gps)  │ X, Y pozisyon                    │ Z, yaw, hızlar         │                                                                      
  ├──────────────────────┼──────────────────────────────────┼────────────────────────┤
  │ IMU (/imu/fixed_cov) │ Roll, Pitch, Yaw + açısal hızlar │ X, Y, Z, lineer hızlar │                                                                        
  └──────────────────────┴──────────────────────────────────┴────────────────────────┘                                                                        
                                                                                                                                                              
  Mantık: GPS sana nerede olduğunu söyler ama titrer ve sürekli sıçrar. IMU sana hangi yönde baktığını söyler ama zamanla kayar (drift). EKF ikisini          
  istatistiksel olarak birleştirip ikisinin de zayıf olduğu noktaları kapatır.                                                                              
                                                                                                                                                              
  GPS → X,Y (konum doğru ama gürültülü)                                                                                                                       
  IMU → Yaw (yön doğru ama drifter)                                                                                                                           
  EKF → X,Y + Yaw (birlikte stabil)                                                                                                                           
                                                                                                                                                              
  ZED bu füzyonda var mı? ekf.yaml (simülasyon) → YOK. ekf_fusion.yaml (gerçek donanım) → VAR (ZED odom X,Y,Z + hız).                                         
                                                                                                                                                            
  ---                                                                                                                                                         
  FÜZYON 2 — Engel Haritası: LiDAR + ZED PointCloud → Costmap (Nav2)                                                                                        
                                                                                                                                                              
  Ne: Nav2 ObstacleLayer, iki sensörden gelen veriyi tek costmap'e yazar
  Dosya: workspace_nav/config/nav2_params.yaml                                                                                                                
  Çıktı: /local_costmap/costmap (Nav2 MPPI controller bunu kullanır)                                                                                        
                                                                                                                                                              
  /scan/filtered        ─► ObstacleLayer ─► /local_costmap/costmap                                                                                          
  /zed/depth/points     ─►               ─►                                                                                                                   
                                                                                                                                                              
  Her kaynağın rolü:                                                                                                                                          
                                                                                                                                                              
  ┌───────────────────┬─────────────┬──────────────────┬──────────────┬─────────────────────────────────────────────────┐                                     
  │      Kaynak       │  Veri Tipi  │ Yükseklik Dilimi │    Menzil    │                   Ne Yakalar                    │                                   
  ├───────────────────┼─────────────┼──────────────────┼──────────────┼─────────────────────────────────────────────────┤                                     
  │ /scan/filtered    │ LaserScan   │ 0.0 – 2.0 m      │ 0.1 – 10.0 m │ Yatay düzlemdeki tüm engeller (boya, şamandıra) │                                   
  ├───────────────────┼─────────────┼──────────────────┼──────────────┼─────────────────────────────────────────────────┤                                     
  │ /zed/depth/points │ PointCloud2 │ 0.1 – 2.5 m      │ 0.5 – 8.0 m  │ LiDAR'ın ıskaladığı yüksek/düşük engeller       │                                     
  └───────────────────┴─────────────┴──────────────────┴──────────────┴─────────────────────────────────────────────────┘                                     
                                                                                                                                                              
  Neden ikisine birden gerek var?                                                                                                                             
  RPLidar A1M8 sadece tek bir yatay düzlem tarar (~0°). Su yüzeyinde hafif yükselen bir platform veya alçak sarkan bir nesne LiDAR'ın gözünden kaçar. ZED
  depth cloud bu boşluğu 3B olarak doldurur.                                                                                                                  
                                                               
  Yan görünüm:                                                                                                                                                
            [Şamandıra üstü] ← ZED görür, LiDAR görmez         
  ──────────[LiDAR düzlemi]─────                                                                                                                              
            [Su yüzeyi]                                                                                                                                       
                                                                                                                                                              
  global_costmap'te ZED yok — global harita büyük menzil ister (300x300m, 0.5m çözünürlük), ZED sadece 8m menzilli. Oraya sadece /scan giriyor.               
                                                               
  ---                                                                                                                                                         
  FÜZYON 3 — Mesafe Ölçümü: LiDAR + ZED Depth → Hedef Mesafesi (Kamikaze Smart Fallback)
                                                                                                                                                              
  Ne: kamikaze_control.py içindeki kural tabanlı öncelik zinciri
  Dosya: workspace_nav/scripts/kamikaze_control.py                                                                                                            
  Çıktı: self._fusion_dist (HUD'da gösterilir, mission_manager kullanır)
                                                                                                                                                              
  Görüntüden hedef piksel koordinatı (cx_px, cy_px)            
             │                                                                                                                                                
             ▼ Step 1: Açı hesabı                              
      angle = (cx_norm - 0.5) × FOV_H_RAD                                                                                                                     
             │                                                                                                                                                
             ▼ Step 2: LiDAR önce dene
      _safe_lidar_dist(angle, window=±15°)                                                                                                                    
      → /scan topic'inden o açıdaki geçerli mesafelerin medyanı                                                                                               
             │                                                                                                                                                
      ┌──────┴───────┐                                                                                                                                        
     Var           Yok (engel yok / out-of-range)                                                                                                             
      │               │                                                                                                                                       
      ▼               ▼ Step 3: ZED fallback                                                                                                                  
    Kaynak:       _zed_depth_at_pixel(depth_img, cx_px, cy_px)                                                                                                
    LiDAR         → 1280×720 piksel koordinatını 320×180'e ölçekle                                                                                            
                  → 5px pencerede medyan al                                                                                                                   
                  │                                                                                                                                           
             ┌────┴────┐                                                                                                                                      
            Var       Yok                                                                                                                                     
             │         │                                       
             ▼         ▼                                                                                                                                      
          Kaynak:   dist = -1.0                                
           ZED      (bilinmiyor)                                                                                                                              
   
  _safe_lidar_dist nasıl çalışır:                                                                                                                             
                                                               
  # angle etrafında ±window indeks kadar genişlet                                                                                                             
  # NaN, 0, inf değerleri at                                                                                                                                  
  # Kalan değerlerin medyanını al (spike'lara karşı sağlam)                                                                                                   
                                                                                                                                                              
  _zed_depth_at_pixel nasıl çalışır:                                                                                                                          
                                                                                                                                                              
  # RGB koordinatı (1280×720) → depth koordinatına ölçekle (320×180, ÷4)                                                                                      
  cx_depth = cx_px * 320 / 1280   # = cx_px / 4                                                                                                               
  cy_depth = cy_px * 180 / 720    # = cy_px / 4                                                                                                               
                                                                                                                                                              
  # 5×5 piksel penceresindeki geçerli derinlik değerlerinin medyanı                                                                                           
  # Geçerli: 0.1m < d < 30.0m ve sonlu sayı                    
                                                                                                                                                              
  Bu füzyon Parkur 2 (kapı) ve Parkur 3 (duba) için ayrı ayrı çalışır:                                                                                        
                                                                                                                                                              
  ┌─────────────────────────┬───────────────────────────┬────────────────────────────────┐                                                                    
  │         Senaryo         │           LiDAR           │              ZED               │
  ├─────────────────────────┼───────────────────────────┼────────────────────────────────┤
  │ Tek boya (sol veya sağ) │ Sol buoy açısından mesafe │ Sol buoy pikselinden depth     │
  ├─────────────────────────┼───────────────────────────┼────────────────────────────────┤
  │ İki boya (kapı)         │ Her iki açının ortasından │ Kapı merkezi pikselinden depth │                                                                    
  ├─────────────────────────┼───────────────────────────┼────────────────────────────────┤                                                                    
  │ Kamikaze hedef          │ Hedef açısından mesafe    │ Hedef pikselinden depth        │                                                                    
  └─────────────────────────┴───────────────────────────┴────────────────────────────────┘                                                                    
                                                               
  ---                                                                                                                                                         
  Gerçek Donanımda Ek Füzyon (ekf_fusion.yaml)                 
                                                                                                                                                              
  Bu dosya simülasyonda kullanılmıyor ama gerçek tekne için hazır:
                                                                                                                                                              
  /zed/odom         → EKF: X,Y,Z + vx,vy,vz  (yön BİLEŞENİ YOK — drift eder)
  /mavros/imu/data  → EKF: Roll,Pitch,Yaw + açısal hızlar                                                                                                     
  ──────────────────────────────────────────────────────                                                                                                      
  Çıktı: /odometry/filtered (GPS olmadan da çalışır, kapalı ortam için)                                                                                       
                                                                                                                                                              
  ---                                                                                                                                                         
  Tüm Füzyonların Özet Şeması                                                                                                                                 
                                                                                                                                                              
  Sensörler                Füzyon                  Çıktı
  ─────────                ──────                  ─────                                                                                                      
  GPS ────────────┐                                            
                  ├──► EKF (robot_localization) ──► /odometry/filtered                                                                                        
  IMU ────────────┘                                 odom→base_link TF                                                                                         
                                                         │                                                                                                    
                                                         ▼                                                                                                    
  /scan/filtered ─────────┐                        Nav2 MPPI                                                                                                  
                           ├──► ObstacleLayer ──► Controller                                                                                                  
  /zed/depth/points ───────┘   (local_costmap)        │
                                                       ▼                                                                                                      
                                                /cmd_vel_smoothed
                                                       │                                                                                                      
  /scan ──────────────────► ObstacleLayer ─────────────┘                                                                                                      
                           (global_costmap)
                           (global plan için)                                                                                                                 
                                                               
  /scan (o açıda) ────────┐                                                                                                                                   
                           ├──► Smart Fallback ──► _fusion_dist (m)
  /zed/depth/image ────────┘   (kamikaze_control)  → HUD'da Kaynak: LIDAR/ZED                                                                                 
                                                                                                                                                              
  ---                                                                                                                                                         
  Takip Komutları                                                                                                                                             
                                                               
  # Füzyon 1 — EKF çıkışı
  ros2 topic echo /odometry/filtered --once                                                                                                                   
   
  # Füzyon 2 — Costmap güncelleniyor mu?                                                                                                                      
  ros2 topic hz /local_costmap/costmap                         
                                                                                                                                                              
  # Füzyon 2 — PointCloud geliyor mu?                                                                                                                         
  ros2 topic hz /zed/depth/points
                                                                                                                                                              
  # Füzyon 3 — HUD'da görünür (kamikaze_control kamera penceresi)                                                                                             
  # Veya:
  ros2 topic echo /kamikaze_target --once    
