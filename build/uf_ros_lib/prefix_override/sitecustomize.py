import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eugene/ufactory_ellmer_ws/install/uf_ros_lib'
