import rclpy

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class PlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('prosac_plane_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.pointcloud_callback,
            10)
        self.publisher_ = self.create_publisher(PointCloud2, 'prosac_detected_planes', 10)
        self.subscription  

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        points = self.pointcloud2_to_numpy(msg)
        
        o3d_cloud = self.numpy_to_open3d(points)
        o3d_cloud= o3d_cloud.voxel_down_sample(0.0999)
        sensor_confidence =1.0 # sensor confidence value between 0 and 1
        # reflectance_intensity = 200.0  # 0 and 255.
        # # surface_normals = 0.1  #  0.1 radians for surface normals within a neighborhood of 10 points.
        # point_density = 50.0  # 50 points per square meter within a radius of 1 meter.
    

        # # Normalize each parameter
        # sensor_confidence /= np.max(sensor_confidence)
        # reflectance_intensity /= np.max(reflectance_intensity)
        # # surface_normal_consistency = ...  # Calculate surface normal consistency and normalize
        # point_density /= np.max(point_density)
        # manual_annotation /= np.max(manual_annotation)
        # surface_normal_consistency = self.compute_surface_normal_consistency(pcd, radius=0.1)
        point_density = self.compute_point_density(o3d_cloud, radius=1.0)
        reflectance_intensity = self.compute_reflectance_intensity(np.random.uniform(0, 255, len(o3d_cloud.points)))
        # Combine parameters into a single quality score
        quality_scores = (
            sensor_confidence + 
            reflectance_intensity + 
            point_density  ) / 3.0  # Normalize to [0, 1]
        
        # Normalize combined quality scores
        quality_scores /= np.sum(quality_scores)

        # Ensure all elements are valid probabilities
        quality_scores = np.nan_to_num(quality_scores, nan=1.0 / len(quality_scores))
        self.get_logger().info('QUALITY SCORES {quality_scores}')
        # print(quality_scores)
        plane_model,inliers = self.segment_plane_PROSAC(o3d_cloud,quality_scores)
        # plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.01,
        #                                               ransac_n=3,
        #                                             num_iterations=1000)
        inlier_cloud = o3d_cloud.select_by_index(inliers)
        self.publish_pointcloud(inlier_cloud)

    def compute_point_density(self,pcd, radius=1.0):
        self.get_logger().info('Computing point density')
        num_points = len(pcd.points)
        density = np.zeros(num_points)

        pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        for i in range(num_points):
            self.get_logger().info("Point number {}".format(num_points))
            [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], radius)
            density[i] = len(idx)

    # Normalize to [0, 1]
        min_val = np.min(density)
        max_val = np.max(density)
        if min_val != max_val:
            density_normalized = (density - min_val) / (max_val - min_val)
        else:
            density_normalized = np.zeros(num_points)

       
        return density_normalized

    def compute_reflectance_intensity(self,intensities):

        self.get_logger().info('Computing reflectance intensity')
        min_val = np.min(intensities)
        max_val = np.max(intensities)
        if min_val != max_val:
            intensity_normalized = (intensities - min_val) / (max_val - min_val)
        else:
            intensity_normalized = np.zeros(len(intensities))
      
        return intensity_normalized
    
    def fit_plane(self,points):
    # Fit a plane to a set of points using SVD
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        u, s, vh = np.linalg.svd(centered_points)
        normal = vh[-1, :]
        d = -np.dot(normal, centroid)
        return np.append(normal, d)

    # def segment_plane_PROSAC(self,pcd, quality_scores_normalized,ransac_n=3, num_iterations=1000, distance_threshold=0.01):
    #     points = np.asarray(pcd.points)
    #     num_points = len(points)
        
    #     # PROSAC implementation
    #     best_model = None
    #     best_inliers = []
    #     best_inlier_count = 0
    
    #     for i in range(num_iterations):
    #         # Progressive sampling,this step differentiate it from RANSAC
    #         sample_indices = np.random.choice(num_points, ransac_n, replace=False, p=quality_scores_normalized)
    #         sample_points = points[sample_indices]
        
           
    #         plane_model = self.fit_plane(sample_points)
        
    #         # Calculate distances to the plane for all points
    #         distances_to_plane = np.abs(np.dot(points, plane_model[:3]) + plane_model[3])
        
    #         # Count inliers based on the distance threshold
    #         inliers = np.where(distances_to_plane < distance_threshold)[0]
        
    #         # Update the best model if we found more inliers
    #         if len(inliers) > best_inlier_count:
    #             best_model = plane_model
    #             best_inliers = inliers
    #             best_inlier_count = len(inliers)
    
    #     return best_model, best_inliers
    def segment_plane_PROSAC(self,pcd, quality_scores, ransac_n=3, max_iterations=1000, threshold=0.01):
        num_points = len(pcd.points)

        best_model = None
        best_inliers = []

        for _ in range(max_iterations):
            sample_indices = np.random.choice(num_points, ransac_n, replace=False, p=quality_scores)
            sampled_points = np.asarray(pcd.points)[sample_indices]

            # Use Open3D's built-in RANSAC for plane segmentation
            plane_model, inliers = pcd.segment_plane(distance_threshold=threshold, ransac_n=ransac_n, num_iterations=max_iterations)

            if len(inliers) > len(best_inliers):
                best_model = plane_model
                best_inliers = inliers

        return best_model, best_inliers



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
