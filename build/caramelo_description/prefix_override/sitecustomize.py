import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/linux24-04/Caramelo_workspace/install/caramelo_description'
