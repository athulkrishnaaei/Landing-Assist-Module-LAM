import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import struct

class LivePlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('live_lidar_segmentation')
        # Subscription to the PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/lidar/Lidar1',  # Topic name
            self.pointcloud_callback,  # Callback function
            10)  # QoS profile depth
        # Publisher for the detected plane points
        self.publisher_ = self.create_publisher(PointCloud2, 'live_detected_planes', 10)
        self.subscription  # Prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        # Convert ROS PointCloud2 message to numpy array
        points = self.pointcloud2_to_numpy(msg)
        
        # Check if points array is empty
        if points.size == 0:
            self.get_logger().warn('Empty point cloud received')
            return
        
        # Convert numpy array to Open3D point cloud
        o3d_cloud = self.numpy_to_open3d(points)
        o3d_cloud = o3d_cloud.voxel_down_sample(0.09)  # Downsample the point cloud
        
        # Segment the largest planar component from the point cloud
        plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.01,
                                                       ransac_n=3,
                                                       num_iterations=1000)
        inlier_cloud = o3d_cloud.select_by_index(inliers)
        
        # Publish the inlier points as a new point cloud
        self.publish_pointcloud(inlier_cloud)

    def pointcloud2_to_numpy(self, cloud_msg):
        # Convert ROS PointCloud2 message to numpy array
        fmt = 'fff'  # Format for unpacking
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype)
        points = np.vstack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']]).T
        return points

    def numpy_to_open3d(self, points):
        # Convert numpy array to Open3D point cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

    def publish_pointcloud(self, cloud):
        # Convert Open3D point cloud to ROS PointCloud2 message
        points = np.asarray(cloud.points)
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

        # Publish the PointCloud2 message
        self.publisher_.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivePlaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
