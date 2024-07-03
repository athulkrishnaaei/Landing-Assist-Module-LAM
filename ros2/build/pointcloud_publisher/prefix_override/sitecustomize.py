import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/athul/Landing-Assist-Module-LAM/ros2/install/pointcloud_publisher'
