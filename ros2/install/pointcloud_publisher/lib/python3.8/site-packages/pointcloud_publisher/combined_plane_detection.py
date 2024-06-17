import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import struct

class PlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('combined_plane_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.pointcloud_callback,
            10)
        self.publisher_ = self.create_publisher(PointCloud2, 'combined_detected_planes', 10)
        self.subscription  # prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        points = self.pointcloud2_to_numpy(msg)
        o3d_cloud = self.numpy_to_open3d(points)
        # inlier_cloud = self.detect_planar_patches(o3d_cloud)
        inlier_cloud = self.combined_plane_detection(o3d_cloud)
        self.publish_pointcloud(inlier_cloud)

    def pointcloud2_to_numpy(self, cloud_msg):
        fmt = 'fff'  # format for unpacking
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype)
        points = np.vstack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']]).T
        return points

    def numpy_to_open3d(self, points):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud
    
    def combined_plane_detection(self,cloud):
        points = np.asarray(cloud.points)

        # Calculate distances from sensor origin (assuming (0,0,0))
        distances = np.linalg.norm(points, axis=1)

        # Define a threshold for considering distances as 'same'
        threshold = 0.01  # Adjust based on your LiDAR's precision

        # Create a histogram of distances
        hist, bin_edges = np.histogram(distances, bins='auto')

        # Find the peak in the histogram
        max_bin_index = np.argmax(hist)
        plane_distance = bin_edges[max_bin_index]

        # Extract points near the plane distance
        plane_points_indices = np.where((distances >= plane_distance - threshold) & 
                                (distances <= plane_distance + threshold))[0]

        # Create a point cloud for the detected plane points
        plane_pcd = cloud.select_by_index(plane_points_indices)
        return plane_pcd
    def detect_planar_patches(self, cloud):
        # Estimate normals for the point cloud
        cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # DBSCAN clustering
        labels = np.array(cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")

        planes = o3d.geometry.PointCloud()
        for i in range(max_label + 1):
            indices = np.where(labels == i)[0]
            if len(indices) > 100:  # example threshold for minimum points in a plane
                planes += cloud.select_by_index(indices)

        return planes

    def publish_pointcloud(self, cloud):
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

        self.publisher_.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
