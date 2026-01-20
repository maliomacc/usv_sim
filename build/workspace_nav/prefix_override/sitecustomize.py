import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ngen/yildiz_ws/src/YILDIZ-USV/install/workspace_nav'
