import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/airsim_user/Downloads/Landing-Assist-Module-LAM/install/pointcloud_publisher'
