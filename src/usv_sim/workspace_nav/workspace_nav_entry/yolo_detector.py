"""
Thin entry-point wrapper for yolo_detector.
Loads the actual implementation via its absolute file path,
bypassing the 'scripts' namespace conflict with workspace_ros.
"""
import importlib.util
import os
import sys


def main():
    here      = os.path.dirname(os.path.abspath(__file__))
    impl_path = os.path.normpath(
        os.path.join(here, '..', 'scripts', 'yolo_detector.py')
    )
    if not os.path.isfile(impl_path):
        raise FileNotFoundError(
            f'yolo_detector implementation not found at: {impl_path}'
        )
    spec = importlib.util.spec_from_file_location('_nav_yolo_detector_impl', impl_path)
    mod  = importlib.util.module_from_spec(spec)
    sys.modules['_nav_yolo_detector_impl'] = mod
    spec.loader.exec_module(mod)
    mod.main()


if __name__ == '__main__':
    main()
