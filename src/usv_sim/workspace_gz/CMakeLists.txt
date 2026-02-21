cmake_minimum_required(VERSION 3.8)
project(workspace_gz)

# Derleyici Ayarları
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# --- Ignition Fortress (Humble) Paketleri ---
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common4 REQUIRED COMPONENTS graphics)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-utils1 REQUIRED)
find_package(sdformat12 REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Kütüphaneler (plugins klasörüne bakacak şekilde ayarlandı) ---

# 1. Waves
add_library(Waves SHARED plugins/Wavefield.cc)
target_link_libraries(Waves PUBLIC
  ignition-common4::graphics
  ignition-gazebo6::core
  ignition-math6::ignition-math6
  Eigen3::Eigen
)

# 2. Base Scoring Plugin
add_library(ScoringPlugin SHARED plugins/ScoringPlugin.cc)
target_link_libraries(ScoringPlugin PUBLIC
  ignition-common4::graphics
  ignition-gazebo6::core
  ignition-math6::ignition-math6
)

# 3. Diğer Tüm Pluginler
set(VRX_PLUGINS
  SimpleHydrodynamics
  PolyhedraBuoyancyDrag
  Surface
  USVWind
  WaveVisual
  StationkeepingScoringPlugin
  WayfindingScoringPlugin
  AcousticPerceptionScoringPlugin
  AcousticTrackingScoringPlugin
  AcousticPingerPlugin
  BallShooterPlugin
  LightBuoyPlugin
  NavigationScoringPlugin
  GymkhanaScoringPlugin
  PerceptionScoringPlugin
  PlacardPlugin
  PublisherPlugin
  ScanDockScoringPlugin
  WildlifeScoringPlugin
)

foreach(PLUGIN ${VRX_PLUGINS})
  # Bazı pluginler yardımcı dosyalara ihtiyaç duyar
  set(PLUGIN_SRCS plugins/${PLUGIN}.cc)

  # Scoring pluginleri için WaypointMarkers.cc ekle
  if(${PLUGIN} MATCHES "ScoringPlugin" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/WaypointMarkers.cc")
     list(APPEND PLUGIN_SRCS plugins/WaypointMarkers.cc)
  endif()

  # PolyhedraBuoyancyDrag için ek dosyalar
  if(${PLUGIN} STREQUAL "PolyhedraBuoyancyDrag")
     list(APPEND PLUGIN_SRCS plugins/PolyhedronVolume.cc plugins/ShapeVolume.cc)
  endif()

  add_library(${PLUGIN} SHARED ${PLUGIN_SRCS})
  
  target_link_libraries(${PLUGIN} PUBLIC
    ignition-gazebo6::core
    ignition-plugin1::loader
    ignition-rendering6::ignition-rendering6
    ignition-sensors6::ignition-sensors6
    ignition-utils1::ignition-utils1
    ScoringPlugin
    Waves
    Eigen3::Eigen
  )
endforeach()

# --- Yükleme (Install) ---
install(TARGETS Waves ScoringPlugin ${VRX_PLUGINS}
  DESTINATION lib/${PROJECT_NAME}
)

# Modeller, Dünyalar ve Launch dosyalarını kopyala
install(DIRECTORY launch models worlds description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()