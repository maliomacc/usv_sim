import os
import re

def migrate_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    original_content = content

    # 1. Includes
    content = content.replace('#include <gz/sim/', '#include <ignition/gazebo/')
    content = content.replace('#include <gz/', '#include <ignition/')

    # 2. Namespace definition (handle carefully)
    # If using namespace gz; is present, replace it and add alias for sim
    if 'using namespace gz;' in content:
        content = content.replace('using namespace gz;', 'using namespace ignition;\nnamespace sim = ignition::gazebo;')
    
    # 3. Namespaces and Types (Order matters!)
    content = content.replace('gz::sim', 'ignition::gazebo')
    content = content.replace('gz::', 'ignition::')

    # 4. Macros
    content = content.replace('GZ_PROFILE', 'IGN_PROFILE')
    content = content.replace('GZ_ADD_PLUGIN', 'IGNITION_ADD_PLUGIN') # Covers ALIAS too
    content = content.replace('GZ_UTILS_UNIQUE_IMPL_PTR', 'IGN_UTILS_UNIQUE_IMPL_PTR')

    # 5. Logging
    content = content.replace('gzerr', 'ignerr')
    content = content.replace('gzmsg', 'ignmsg')
    content = content.replace('gzwarn', 'ignwarn')
    content = content.replace('gzdbg', 'igndbg')

    # X. Specific edge case: gz::transport::Node -> ignition::transport::Node 
    # (Covered by gz:: -> ignition::)

    if content != original_content:
        print(f"Migrating {filepath}...")
        with open(filepath, 'w') as f:
            f.write(content)
    else:
        print(f"No changes for {filepath}")

def main():
    extensions = ('.cc', '.hh')
    exclude_files = ['migrate_gz_to_ign.py']
    
    # Walk through current directory and subdirectories
    for root, dirs, files in os.walk('.'):
        for file in files:
            if file.endswith(extensions) and file not in exclude_files:
                migrate_file(os.path.join(root, file))

if __name__ == "__main__":
    main()
