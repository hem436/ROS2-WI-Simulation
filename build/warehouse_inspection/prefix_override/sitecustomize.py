import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hem436/Documents/ros2_ws/install/warehouse_inspection'
