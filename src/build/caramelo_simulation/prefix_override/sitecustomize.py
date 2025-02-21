import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/victor/Caramelo_workspace/src/install/caramelo_simulation'
