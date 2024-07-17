import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import struct
from std_msgs.msg import Header

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.declare_parameter('file_path', '/home/airsim_user/Downloads/Landing-Assist-Module-LAM/cloud.pcd')
        file_path = self.get_parameter('file_path').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pointcloud_data = self.load_pointcloud_file(file_path)
        
    def load_pointcloud_file(self, file_path):
        # Load the point cloud data
        pcd = o3d.io.read_point_cloud(file_path)
        points = np.asarray(pcd.points)
        return points

    def timer_callback(self):
        self.publish_pointcloud()

    def publish_pointcloud(self):
        points = self.pointcloud_data
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        points_array = points.flatten().tolist()
        data = struct.pack('%sf' % len(points_array), *points_array)

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(points),
            data=data,
            is_dense=True
        )

        self.publisher_.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()
    rclpy.spin(pointcloud_publisher)
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
