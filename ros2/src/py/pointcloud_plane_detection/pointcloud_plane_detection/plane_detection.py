import random
from abc import ABC, abstractmethod
import rclpy
from scipy import stats
from scipy.stats.distributions import chi2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import argparse

# Abstract Model class
class Model(ABC):
    @abstractmethod
    def fit(self, pts):
        pass

    @abstractmethod
    def error(self, data):
        pass

    @abstractmethod
    def predict(self, data):
        pass

    @staticmethod
    @abstractmethod
    def get_complexity():
        pass

# RANSAC implementation
def ransac(data, model_type, tolerance, prob_inlier, p=0.99):
    m = model_type.get_complexity()
    best_num_inliers = 0
    n = data.shape[0]
    max_times = int(np.ceil(np.log(1 - p) / np.log(1 - prob_inlier ** m)))
    satisfactory_inlier_ratio = prob_inlier * n

    inliers = []
    for _ in range(max_times):
        pts = data[random.sample(range(n), m)]
        model = model_type()
        model.fit(pts)
        error = model.error(data)
        num_inliers = (error < tolerance).sum()
        if num_inliers / n > satisfactory_inlier_ratio:
            inliers = data[error < tolerance]
            break
        if num_inliers > best_num_inliers:
            best_num_inliers = num_inliers
            inliers = data[error < tolerance]

    model = model_type()
    model.fit(inliers)
    return model



# PROSAC implementation
def prosac(data, quality, model_type, tolerance, beta, eta0, psi,
           max_outlier_proportion, p_good_sample, max_number_of_draws,
           enable_n_star_optimization=True):
    indexes = np.argsort(quality)
    data = data[indexes[::-1]]

    num_points = data.shape[0]
    num_points_to_sample = model_type.get_complexity()
    chi2_value = chi2.isf(2 * psi, 1)

    def niter_ransac(p, epsilon, s, n_max):
        if n_max == -1:
            n_max = np.iinfo(np.int32).max
        if not (n_max >= 1):
            raise ValueError('n_max must be positive')
        if epsilon <= 0:
            return 1
        logarg = - np.exp(s * np.log(1 - epsilon))
        logval = np.log(1 + logarg)
        n = np.log(1 - p) / logval
        if logval < 0 and n < n_max:
            return np.ceil(n)
        return n_max

    def i_min(m, n, beta):
        mu = n * beta
        sigma = np.sqrt(n * beta * (1 - beta))
        return np.ceil(m + mu + sigma * np.sqrt(chi2_value))

    N = num_points
    m = num_points_to_sample
    T_N = niter_ransac(p_good_sample, max_outlier_proportion, num_points_to_sample, -1)
    I_N_min = (1 - max_outlier_proportion) * N

    n_star = N
    I_n_star = 0
    I_N_best = 0
    t = 0
    n = m
    T_n = T_N

    for i in range(m):
        T_n = T_n * (n - i) / (N - i)

    T_n_prime = 1
    k_n_star = T_N

    while ((I_N_best < I_N_min) or t <= k_n_star) and t < T_N and t <= max_number_of_draws:
        t = t + 1

        if (t > T_n_prime) and (n < n_star):
            T_nplus1 = (T_n * (n + 1)) / (n + 1 - m)
            n = n + 1
            T_n_prime = T_n_prime + np.ceil(T_nplus1 - T_n)
            T_n = T_nplus1

        if t > T_n_prime:
            pts_idx = np.random.choice(n, m, replace=False)
        else:
            pts_idx = np.append(np.random.choice(n - 1, m - 1, replace=False), n)

        sample = data[pts_idx]

        model = model_type()
        model.fit(sample)

        error = model.error(data)
        is_inlier = (error < tolerance)
        I_N = is_inlier.sum()

        if I_N > I_N_best:
            I_N_best = I_N
            n_best = N
            I_n_best = I_N
            best_model = model

            if enable_n_star_optimization:
                epsilon_n_best = I_n_best / n_best
                I_n_test = I_N
                for n_test in range(N, m, -1):
                    if not (n_test >= I_n_test):
                        raise RuntimeError('Loop invariant broken: n_test >= I_n_test')
                    if ((I_n_test * n_best > I_n_best * n_test) and (I_n_test > epsilon_n_best * n_test + np.sqrt(
                            n_test * epsilon_n_best * (1 - epsilon_n_best) * chi2_value))):
                        if I_n_test < i_min(m, n_test, beta):
                            break
                        n_best = n_test
                        I_n_best = I_n_test
                        epsilon_n_best = I_n_best / n_best
                    I_n_test = I_n_test - is_inlier[n_test - 1]

            if I_n_best * n_star > I_n_star * n_best:
                if not (n_best >= I_n_best):
                    raise RuntimeError('Assertion not respected: n_best >= I_n_best')
                n_star = n_best
                I_n_star = I_n_best
                k_n_star = niter_ransac(1 - eta0, 1 - I_n_star / n_star, m, T_N)

    return best_model

# LinearModel class
class LinearModel(Model):
    def __init__(self):
        self.m = 0
        self.b = 0

    def fit(self, pts):
        xs = pts[:, 0]
        ys = pts[:, 1]
        self.m, self.b = stats.linregress(xs, ys)[:2]

    def predict(self, x):
        return self.m * x + self.b

    def error(self, data):
        prediction = self.predict(data[:, 0])
        true_value = data[:, 1]
        return np.sqrt(np.square(true_value - prediction))

    @staticmethod
    def get_complexity():
        return 2

class PlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('plane_detection')
        self.declare_parameter('method','prosac')
        self.method = self.get_parameter('method').get_parameter_value().string_value
        self.get_logger().info('method used is : %s' % self.method)
        # Subscription to the PointCloud2 topic
        topic="/pointcloud"
        #topic = "/airsim_node/PX4/lidar/Lidar1"
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,  # Topic name
            self.pointcloud_callback,  # Callback function
            10)  # QoS profile depth
        # Publisher for the detected plane points
        self.publisher_ = self.create_publisher(PointCloud2, 'detected_planes', 10)
        self.subscription  # Prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        # Convert ROS PointCloud2 message to numpy array
        points = self.pointcloud2_to_numpy(msg)

        # Convert numpy array to Open3D point cloud
        o3d_cloud = self.numpy_to_open3d(points)
        o3d_cloud = o3d_cloud.voxel_down_sample(0.09)  # Downsample the point cloud

        if self.method == "prosac":
            inlier_cloud = self.prosac_plane_segmentation(o3d_cloud)
        elif self.method == "ransac":
            inlier_cloud = self.ransac_plane_segmentation(o3d_cloud)
        elif self.method == "planar_patch":
            inlier_cloud = self.detect_planar_patches(o3d_cloud)

        # Publish the inlier points as a new point cloud
        self.publish_pointcloud_colored(inlier_cloud)

    def ransac_plane_segmentation(self, cloud):
        self.get_logger().info(f'Running ransac plane segmentation')
        # Segment the largest planar component from the point cloud
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.01,
                                                   ransac_n=3,
                                                   num_iterations=1000)
        inlier_cloud = cloud.select_by_index(inliers)
        return inlier_cloud

    def prosac_plane_segmentation(self, cloud):
        self.get_logger().info(f'Running prosac plane segmentation')
        points = np.asarray(cloud.points)
        cloud = cloud.voxel_down_sample(0.05)

        sensor_confidence = 1.0
        point_density = self.compute_point_density(cloud, radius=1.0)
        reflectance_intensity = self.compute_reflectance_intensity(np.random.uniform(0, 255, len(cloud.points)))

        quality_scores = (
            sensor_confidence +
            reflectance_intensity +
            point_density
        ) / 3.0

        quality_scores /= np.sum(quality_scores)
        quality_scores = np.nan_to_num(quality_scores, nan=1.0 / len(quality_scores))

        self.get_logger().info(f'QUALITY SCORES {quality_scores}')

        plane_model, inliers = self.segment_plane_PROSAC(cloud, quality_scores)
        inlier_cloud = cloud.select_by_index(inliers)

        return inlier_cloud

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

    def fit_plane(self, points):
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        u, s, vh = np.linalg.svd(centered_points)
        normal = vh[-1, :]
        d = -np.dot(normal, centroid)
        return np.append(normal, d)

    def detect_planar_patches(self, cloud):
        print('Running planar patch segmentation')

        if not isinstance(cloud, o3d.geometry.PointCloud):
            print("The provided object is not a PointCloud.")
            return None

        if cloud.is_empty():
            print("The provided cloud is empty.")
            return None

        # Ensure the cloud has points
        if len(cloud.points) == 0:
            print("The provided cloud has no points.")
            return None

        # Estimate normals to assist with the segmentation
        cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
        # DBSCAN clustering to identify clusters
        labels = np.array(cloud.cluster_dbscan(eps=0.05, min_points=1, print_progress=True))

        if labels.size == 0 or labels.max() < 0:
            print("No clusters found by DBSCAN.")
            return None

        max_label = labels.max()
        print(f"Point cloud has {max_label + 1} clusters")

        # Initialize an empty PointCloud to hold all inliers
        inlier_cloud = o3d.geometry.PointCloud()

        for i in range(max_label + 1):
            indices = np.where(labels == i)[0]
            if len(indices) > 50:  # Threshold for a minimum number of points in a plane
                # Add points that belong to this plane to the inlier cloud
                inlier_cloud += cloud.select_by_index(indices)

        return inlier_cloud

    def segment_plane_PROSAC(self, pcd, quality_scores_normalized, ransac_n=3, num_iterations=1000, distance_threshold=0.01):
        points = np.asarray(pcd.points)
        num_points = len(points)

        best_model = None
        best_inliers = []
        best_inlier_count = 0

        for i in range(num_iterations):
            sample_indices = np.random.choice(num_points, ransac_n, replace=False, p=quality_scores_normalized)
            sample_points = points[sample_indices]

            plane_model = self.fit_plane(sample_points)
            distances_to_plane = np.abs(np.dot(points, plane_model[:3]) + plane_model[3])

            inliers = np.where(distances_to_plane < distance_threshold)[0]

            if len(inliers) > best_inlier_count:
                best_model = plane_model
                best_inliers = inliers
                best_inlier_count = len(inliers)

        return best_model, best_inliers

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
    
    def publish_pointcloud_colored(self, cloud):
        # Convert Open3D point cloud to ROS PointCloud2 message
        points = np.asarray(cloud.points)
        num_points = points.shape[0]

        # Set all points to green color (0, 255, 0)
        r = 0
        g = 255
        b = 0
        rgb = (r << 16) | (g << 8) | b
        rgb_packed = struct.unpack('f', struct.pack('I', rgb))[0]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = []
        for point in points:
            data.extend(struct.unpack('f', struct.pack('f', point[0])))
            data.extend(struct.unpack('f', struct.pack('f', point[1])))
            data.extend(struct.unpack('f', struct.pack('f', point[2])))
            data.extend(struct.unpack('f', struct.pack('f', rgb_packed)))

        data = struct.pack('%sf' % len(data), *data)

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=num_points,
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * num_points,
            data=data,
            is_dense=True
        )

    # Publish the PointCloud2 message
        self.publisher_.publish(pointcloud_msg)
       
def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()