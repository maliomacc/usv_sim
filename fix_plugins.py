import os
import re

# Eklenti klasör yolu
plugin_dir = os.path.expanduser("~/sti_usv/src/usv_sim/workspace_gz/plugins")

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Temel isim alanı ve tip dönüşümleri
    content = content.replace("ignition::sim", "ignition::gazebo")
    content = content.replace("sim::Entity", "gazebo::Entity")
    content = content.replace("sim::kNullEntity", "gazebo::v6::kNullEntity")
    content = content.replace("GZ_PROFILE", "IGN_PROFILE")
    content = content.replace("GZ_RTOD", "IGN_RTOD")
    content = content.replace("GZ_DTOR", "IGN_DTOR")
    content = content.replace("GZ_ADD_PLUGIN_ALIAS", "IGNITION_ADD_PLUGIN")
    
    # 2. KRİTİK: std::get_if hatasını kesin çöz (long unsigned int -> unsigned int)
    content = content.replace("std::get_if<long unsigned int>", "std::get_if<unsigned int>")
    content = content.replace("std::get_if<uint64_t>", "std::get_if<unsigned int>")

    # 3. KRİTİK: IGNITION_ADD_PLUGIN makrosunu Humble formatına (2 argümanlı) çek
    # Bu regex, makronun içindeki tüm argümanları yakalar ve sadece Sınıfİsmi + System olarak yeniden yazar.
    # Örn: IGNITION_ADD_PLUGIN(vrx::Surface, "alias", ...) -> IGNITION_ADD_PLUGIN(vrx::Surface, ignition::gazebo::System)
    pattern = re.compile(r"IGNITION_ADD_PLUGIN\s*\(([^,)]+)(?:,[^)]+)?\)", re.DOTALL)
    content = pattern.sub(r"IGNITION_ADD_PLUGIN(\1, ignition::gazebo::System)", content)

    # 4. PIMPL Makroları ve Include kontrolü
    content = content.replace("GZ_UTILS_UNIQUE_IMPL_PTR", "IGN_UTILS_UNIQUE_IMPL_PTR")
    if "IGN_UTILS_UNIQUE_IMPL_PTR" in content and "ImplPtr.hh" not in content:
        content = "#include <ignition/utils/ImplPtr.hh>\n" + content
    if "ignition::gazebo" in content and "System.hh" not in content:
        content = "#include <ignition/gazebo/System.hh>\n" + content

    with open(filepath, 'w') as f:
        f.write(content)

# Tüm .cc ve .hh dosyalarını tara
for root, dirs, files in os.walk(plugin_dir):
    for file in files:
        if file.endswith((".cc", ".hh")):
            fix_file(os.path.join(root, file))

print("🚀 Humble/Fortress Porting işlemi başarıyla tamamlandı!")
