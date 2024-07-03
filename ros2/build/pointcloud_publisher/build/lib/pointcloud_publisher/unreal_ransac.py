
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

class SafeLandingDetectionNode(Node):
    def __init__(self):
        super().__init__('safe_landing_detection')
        # Subscription to the LiDAR PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/lidar/Lidar1',  # Change to your LiDAR topic name
            # 'pointcloud'
            self.pointcloud_callback,  # Callback function
            10)  # QoS profile depth
        self.subscription  # Prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        
        # Convert ROS PointCloud2 message to numpy array
        points = self.pointcloud2_to_numpy(msg)
        self.get_logger().info(f'Converted PointCloud2 to numpy array with shape: {points.shape}')
        
        # Convert numpy array to Open3D point cloud
        o3d_cloud = self.numpy_to_open3d(points)
        self.get_logger().info('Converted numpy array to Open3D point cloud')
        
        # Downsample the point cloud
        o3d_cloud = o3d_cloud.voxel_down_sample(0.09)
        self.get_logger().info('Downsampled the point cloud')
        
        # Segment the largest planar component from the point cloud
        plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.01,
                                                       ransac_n=3,
                                                       num_iterations=1000)
        self.get_logger().info(f'Segmented plane with model: {plane_model}')
        self.get_logger().info(f'Number of inlier points: {len(inliers)}')
        
        inlier_cloud = o3d_cloud.select_by_index(inliers)
        
        # Check if the detected plane is suitable for landing
        if self.is_safe_for_landing(inlier_cloud):
            self.get_logger().info('Safe landing zone detected')
            landing_coordinates = self.get_landing_coordinates(inlier_cloud)
            self.get_logger().info(f'Safe landing coordinates: {landing_coordinates}')
        else:
            self.get_logger().info('No safe landing zone detected')

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

    def is_safe_for_landing(self, cloud):
        # Check if the plane is large and flat enough for safe landing
        # Define your criteria for a safe landing zone
        min_points = 500  # Minimum number of points in the plane
        max_height_variation = 0.05  # Maximum allowed height variation in the plane
        
        points = np.asarray(cloud.points)
        if points.shape[0] < min_points:
            self.get_logger().info(f'Plane rejected: insufficient points ({points.shape[0]} < {min_points})')
            return False
        
        # Calculate the height variation
        height_variation = np.max(points[:, 2]) - np.min(points[:, 2])
        if height_variation > max_height_variation:
            self.get_logger().info(f'Plane rejected: height variation too large ({height_variation} > {max_height_variation})')
            return False
        
        self.get_logger().info('Plane accepted as safe landing zone')
        return True

    def get_landing_coordinates(self, cloud):
        # Calculate the centroid of the inlier points for landing
        points = np.asarray(cloud.points)
        centroid = np.mean(points, axis=0)
        return centroid

def main(args=None):
    rclpy.init(args=args)
    node = SafeLandingDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
