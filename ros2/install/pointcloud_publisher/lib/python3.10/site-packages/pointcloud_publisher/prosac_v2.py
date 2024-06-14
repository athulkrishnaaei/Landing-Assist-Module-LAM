# plane_detection_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import struct
from std_msgs.msg import Header
from ransac_prosac import prosac, Model

class PlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('prosac_v2_plane_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.pointcloud_callback,
            10)
        self.publisher_ = self.create_publisher(PointCloud2, 'prosac_v2_detected_planes', 10)
        self.subscription  

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        points = self.pointcloud2_to_numpy(msg)
        
        o3d_cloud = self.numpy_to_open3d(points)
        o3d_cloud = o3d_cloud.voxel_down_sample(0.0999)
        sensor_confidence = 1.0
        point_density = self.compute_point_density(o3d_cloud, radius=1.0)
        reflectance_intensity = self.compute_reflectance_intensity(np.random.uniform(0, 255, len(o3d_cloud.points)))

        quality_scores = (
            sensor_confidence + 
            reflectance_intensity + 
            point_density
        ) / 3.0

        quality_scores /= np.sum(quality_scores)
        quality_scores = np.nan_to_num(quality_scores, nan=1.0 / len(quality_scores))

        self.get_logger().info(f'QUALITY SCORES {quality_scores}')

        # Use the imported prosac function
        points_array = np.asarray(o3d_cloud.points)
        tolerance = 0.01
        beta = 0.1
        eta0 = 0.05
        psi = 0.05
        max_outlier_proportion = 0.9
        p_good_sample = 0.99
        max_number_of_draws = 1000

        plane_model = prosac(points_array, quality_scores, PlaneModel, tolerance, beta, eta0, psi,
                             max_outlier_proportion, p_good_sample, max_number_of_draws)

        inliers = self.get_inliers(points_array, plane_model, tolerance)
        inlier_cloud = o3d_cloud.select_by_index(inliers)
        self.publish_pointcloud(inlier_cloud)

    def compute_point_density(self, pcd, radius=1.0):
        self.get_logger().info('Computing point density')
        num_points = len(pcd.points)
        density = np.zeros(num_points)

        pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        for i in range(num_points):
            self.get_logger().info("Point number {}".format(num_points))
            [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], radius)
            density[i] = len(idx)

        min_val = np.min(density)
        max_val = np.max(density)
        if min_val != max_val:
            density_normalized = (density - min_val) / (max_val - min_val)
        else:
            density_normalized = np.zeros(num_points)

        return density_normalized

    def compute_reflectance_intensity(self, intensities):
        self.get_logger().info('Computing reflectance intensity')
        min_val = np.min(intensities)
        max_val = np.max(intensities)
        if min_val != max_val:
            intensity_normalized = (intensities - min_val) / (max_val - min_val)
        else:
            intensity_normalized = np.zeros(len(intensities))
      
        return intensity_normalized

    def get_inliers(self, points, model, tolerance):
        distances_to_plane = np.abs(np.dot(points, model[:3]) + model[3])
        inliers = np.where(distances_to_plane < tolerance)[0]
        return inliers

    def pointcloud2_to_numpy(self, cloud_msg):
        fmt = 'fff'
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype)
        points = np.vstack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']]).T
        return points

    def numpy_to_open3d(self, points):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

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

class PlaneModel(Model):
    def __init__(self):
        self.normal = np.zeros(3)
        self.d = 0

    def fit(self, pts):
        centroid = np.mean(pts, axis=0)
        centered_points = pts - centroid
        u, s, vh = np.linalg.svd(centered_points)
        self.normal = vh[-1, :]
        self.d = -np.dot(self.normal, centroid)

    def error(self, data):
        return np.abs(np.dot(data, self.normal) + self.d)

    def predict(self, data):
        return np.dot(data, self.normal) + self.d

    @staticmethod
    def get_complexity():
        return 3

def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
