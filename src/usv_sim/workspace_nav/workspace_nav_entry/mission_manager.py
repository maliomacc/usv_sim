"""
Thin entry-point wrapper for mission_manager.
Loads the actual implementation via its absolute file path, bypassing
the 'scripts' namespace conflict between workspace_nav and workspace_ros.
"""
import importlib.util
import os
import sys


def main():
    # Resolve the real implementation path relative to this wrapper's location.
    # Installed layout:
    #   site-packages/workspace_nav_entry/mission_manager.py  ← this file
    #   site-packages/scripts/mission_manager.py              ← real impl
    here = os.path.dirname(os.path.abspath(__file__))
    impl_path = os.path.normpath(os.path.join(here, '..', 'scripts', 'mission_manager.py'))

    if not os.path.isfile(impl_path):
        raise FileNotFoundError(
            f'mission_manager implementation not found at: {impl_path}'
        )

    spec = importlib.util.spec_from_file_location('_nav_mission_manager_impl', impl_path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['_nav_mission_manager_impl'] = mod
    spec.loader.exec_module(mod)
    mod.main()


if __name__ == '__main__':
    main()
