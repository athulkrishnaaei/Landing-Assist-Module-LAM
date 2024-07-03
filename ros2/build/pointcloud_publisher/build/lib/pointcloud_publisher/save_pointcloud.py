import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np

class LiDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/lidar/Lidar1',  # Change to your live LiDAR topic
            self.pointcloud_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.pcd_file_path = '/home/athul/Desktop/live_cloud.pcd'

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        points = self.pointcloud2_to_numpy(msg)
        self.save_pcd_file(points)

    def pointcloud2_to_numpy(self, cloud_msg):
        fmt = 'fff'
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype)
        points = np.vstack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']]).T
        return points

    def save_pcd_file(self, points):
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(self.pcd_file_path, o3d_cloud)
        self.get_logger().info(f'Saved point cloud data to {self.pcd_file_path}')

def main(args=None):
    rclpy.init(args=args)
    node = LiDARSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
