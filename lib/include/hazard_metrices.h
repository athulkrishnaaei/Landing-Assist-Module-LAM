#ifndef HAZARD_METRICES_H
#define HAZARD_METRICES_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/surface/mls.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include <eigen3/Eigen/Dense>
#include <open3d/Open3D.h>

#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>

#include <omp.h>

#include <common.h>
#include <variant>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/octree/octree_search.h>
#include <pcl/segmentation/extract_clusters.h>

#include <queue>
#include <unordered_set>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

template <typename PointT>
using CloudInput =
    std::variant<std::string, typename pcl::PointCloud<PointT>::Ptr>;

using Open3DCloudInput =
    std::variant<std::string, std::shared_ptr<open3d::geometry::PointCloud>>;

//======================================PCA (Principle Component Analysis)(PCL)==========================================================================================================

// template <typename PointT>
inline PCLResult PrincipleComponentAnalysis(const CloudInput<PointT> &input,
                                            float angleThreshold = 20.0f,
                                            int k = 10) {
  PCLResult result;
  result.pcl_method =
      "Principal Component Analysis (using NormalEstimationOMP)";
  result.inlier_cloud = pcl::make_shared<PointCloudT>();
  result.outlier_cloud = pcl::make_shared<PointCloudT>();
  result.downsampled_cloud = pcl::make_shared<PointCloudT>();

  // Load the cloud and determine if downsampling is needed.
  auto cloud = loadPCLCloud<PointT>(input);

  result.downsampled_cloud = cloud;

  // Compute normals in parallel using NormalEstimationOMP.
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  ne.setInputCloud(result.downsampled_cloud);
  ne.setKSearch(k);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // Compute PCA on the entire cloud.
  pcl::PCA<PointT> pca;
  pca.setInputCloud(result.downsampled_cloud);
  Eigen::Matrix3f eigenvectors =
      pca.getEigenVectors(); // Eigenvectors sorted by descending eigenvalues.
  Eigen::Vector4f mean = pca.getMean();

  // Use the eigenvector with the smallest eigenvalue (typically the 3rd column)
  // as the global plane normal.
  Eigen::Vector3f global_normal = eigenvectors.col(2);

  // Compute and output the global slope (angle between global_normal and the
  // vertical (0,0,1)).
  float global_dot =
      std::fabs(global_normal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
  float global_slope =
      std::acos(global_dot) * 180.0f / static_cast<float>(M_PI);
  std::cout << "Global PCA plane slope: " << global_slope << " degrees"
            << std::endl;

  // Compute plane coefficients: Ax + By + Cz + D = 0.
  float A = global_normal(0);
  float B = global_normal(1);
  float C = global_normal(2);
  float D = -(A * mean(0) + B * mean(1) + C * mean(2));
  result.plane_coefficients = std::make_shared<pcl::ModelCoefficients>();
  result.plane_coefficients->values.push_back(A);
  result.plane_coefficients->values.push_back(B);
  result.plane_coefficients->values.push_back(C);
  result.plane_coefficients->values.push_back(D);
  std::cout << "Plane coefficients (A, B, C, D): " << A << ", " << B << ", "
            << C << ", " << D << std::endl;

  // Classify points based on the computed normals and the angle threshold.
  for (size_t i = 0; i < normals->points.size(); i++) {
    Eigen::Vector3f normal(normals->points[i].normal_x,
                           normals->points[i].normal_y,
                           normals->points[i].normal_z);
    // Check for invalid normal values.
    if (std::isnan(normal.norm()) || normal.norm() == 0) {
      result.outlier_cloud->push_back(result.downsampled_cloud->points[i]);
      continue;
    }
    float dot_product =
        std::fabs(normal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
    float slope = std::acos(dot_product) * 180.0f / static_cast<float>(M_PI);
    if (slope <= angleThreshold)
      result.inlier_cloud->push_back(result.downsampled_cloud->points[i]);
    else
      result.outlier_cloud->push_back(result.downsampled_cloud->points[i]);
  }
  std::cout << "Inliers (slope ≤ " << angleThreshold
            << "°): " << result.inlier_cloud->size() << std::endl;
  std::cout << "Outliers (slope > " << angleThreshold
            << "°): " << result.outlier_cloud->size() << std::endl;

  return result;
}

//======================= Ransac Plane Segmentation (PCL) ===================================================================

inline PCLResult performRANSAC(const CloudInput<PointT> &input,
                               float distanceThreshold = 0.02f,
                               int maxIterations = 100) {
  // Initialize the result structure with new point clouds.
  PCLResult result;
  result.pcl_method = "RANSAC";

  result.downsampled_cloud = pcl::make_shared<PointCloudT>();
  result.inlier_cloud = pcl::make_shared<PointCloudT>();
  result.outlier_cloud = pcl::make_shared<PointCloudT>();

  auto cloud = loadPCLCloud<PointT>(input);
  result.downsampled_cloud = cloud;

  // Set up RANSAC segmentation for a plane model.
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(result.downsampled_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    return result;
  }
  std::cout << "RANSAC found " << inliers->indices.size() << " inliers."
            << std::endl;

  // Store the plane coefficients in the result structure.
  result.plane_coefficients = coefficients;

  // Extract the inlier cloud.
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(result.downsampled_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*result.inlier_cloud);

  // Extract outliers (points not on the plane).
  extract.setNegative(true);
  extract.filter(*result.outlier_cloud);

  return result;
}

//=========================================== PROSAC Plane Segmentation (PCL) =================================================

inline PCLResult performPROSAC(const CloudInput<PointT> &input,
                               float distanceThreshold = 0.02f,
                               int maxIterations = 100) {
  // Initialize the result structure with new point clouds
  PCLResult result;
  result.pcl_method = "PROSAC";

  result.downsampled_cloud = pcl::make_shared<PointCloudT>();
  result.inlier_cloud = pcl::make_shared<PointCloudT>();
  result.outlier_cloud = pcl::make_shared<PointCloudT>();

  auto cloud = loadPCLCloud<PointT>(input);
  result.downsampled_cloud = cloud;

  // If detecting a table or ground plane → SACMODEL_PLANE + SAC_RANSAC
  // If extracting pipes or poles → SACMODEL_CYLINDER + SAC_RANSAC
  // If detecting objects with circular features → SACMODEL_CIRCLE3D + SAC_LMEDS
  // If handling large noisy datasets → SACMODEL_PLANE + SAC_RRANSAC
  // If prioritizing accuracy over speed → SACMODEL_PLANE + SAC_MLESAC
  // Extract inliers (the plane) from the cloud
  // Set up PROSAC segmentation for a plane model
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_PROSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(result.downsampled_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    return result;
  }
  std::cout << "PROSAC found " << inliers->indices.size() << " inliers."
            << std::endl;

  // Store the plane coefficients in the result structure
  result.plane_coefficients = coefficients;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(result.downsampled_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*result.inlier_cloud);

  // Extract outliers (points not on the plane)
  extract.setNegative(true);
  extract.filter(*result.outlier_cloud);

  return result;
}

//======================================= Least of Median Square Plane Fitting (PCL) ============================================

inline PCLResult performLMEDS(const CloudInput<PointT> &input,
                              float distanceThreshold = 0.02f,
                              int maxIterations = 100) {
  // Initialize the result structure with new point clouds
  PCLResult result;
  result.pcl_method = "LMEDS";

  result.downsampled_cloud = pcl::make_shared<PointCloudT>();
  result.inlier_cloud = pcl::make_shared<PointCloudT>();
  result.outlier_cloud = pcl::make_shared<PointCloudT>();

  auto cloud = loadPCLCloud<PointT>(input);
  result.downsampled_cloud = cloud;

  // If detecting a table or ground plane → SACMODEL_PLANE + SAC_RANSAC
  // If extracting pipes or poles → SACMODEL_CYLINDER + SAC_RANSAC
  // If detecting objects with circular features → SACMODEL_CIRCLE3D + SAC_LMEDS
  // If handling large noisy datasets → SACMODEL_PLANE + SAC_RRANSAC
  // If prioritizing accuracy over speed → SACMODEL_PLANE + SAC_MLESAC
  // Extract inliers (the plane) from the cloud
  // Set up LMEDS segmentation for a plane model
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_LMEDS);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(result.downsampled_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    return result;
  }
  std::cout << "LMEDS found " << inliers->indices.size() << " inliers."
            << std::endl;

  // Store the plane coefficients in the result structure
  result.plane_coefficients = coefficients;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(result.downsampled_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*result.inlier_cloud);

  // Extract outliers (points not on the plane)
  extract.setNegative(true);
  extract.filter(*result.outlier_cloud);

  return result;
}

//========================== Region growing segmentation (PCL) ===================================================================

inline PCLResult regionGrowingSegmentation(
    const CloudInput<PointT> &input, float angleThreshold = 15.0f,
    int min_cluster_size = 10,       // Minimum number of points per cluster.
    int max_cluster_size = 10000000, // Maximum number of points per cluster.
    int number_of_neighbours = 30, // Nearest neighbors used in region growing.
    int normal_k_search = 50,      // K nearest neighbors for normal estimation.
    float smoothness_threshold = 2.0f / 180.0f * M_PI,
    float curvature_threshold = 0.9f) {
  // Initialize the result struct.
  PCLResult result;

  // Load the pointcloud.
  auto cloud = loadPCLCloud<PointT>(input);
  result.downsampled_cloud = cloud;

  // Remove any NaN points.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*result.downsampled_cloud,
                               *result.downsampled_cloud, indices);

  // ------------------------------------------------------------------------
  // Compute normals in parallel using NormalEstimationOMP
  // ------------------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
  // Optionally, specify the number of threads (0 uses all available threads):
  // normal_estimator.setNumberOfThreads(4);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(result.downsampled_cloud);
  normal_estimator.setKSearch(normal_k_search);
  normal_estimator.compute(*normals);

  // Use the indices from NaN removal as valid indices.
  pcl::IndicesPtr valid_indices(new std::vector<int>(indices));

  // Set up region growing segmentation.
  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  reg.setMinClusterSize(min_cluster_size);
  reg.setMaxClusterSize(max_cluster_size);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(number_of_neighbours);
  reg.setInputCloud(result.downsampled_cloud);
  reg.setIndices(valid_indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smoothness_threshold);
  reg.setCurvatureThreshold(curvature_threshold);

  // Extract clusters.
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  std::cout << "Found " << clusters.size() << " clusters." << std::endl;

  // Containers for inliers and outliers.
  pcl::PointCloud<PointT>::Ptr inliers_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr outliers_cloud(new pcl::PointCloud<PointT>);

  // Global vertical axis (assumed here as the Z-axis).
  Eigen::Vector3f vertical(0.0f, 0.0f, 1.0f);
  float horizontal_dot_threshold = std::cos(angleThreshold * M_PI / 180.0f);

  // Process clusters: determine whether each cluster is "horizontal" or not.
  for (const auto &cluster : clusters) {
    Eigen::Vector3f avg_normal(0.0f, 0.0f, 0.0f);
    for (const auto &idx : cluster.indices) {
      const auto &n = normals->points[idx];
      avg_normal += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
    }
    if (!cluster.indices.empty())
      avg_normal /= static_cast<float>(cluster.indices.size());
    if (avg_normal.norm() != 0)
      avg_normal.normalize();

    float dot = std::fabs(avg_normal.dot(vertical));
    if (dot >= horizontal_dot_threshold) {
      // Cluster is horizontal.
      for (const auto &idx : cluster.indices) {
        inliers_cloud->points.push_back(result.downsampled_cloud->points[idx]);
      }
    } else {
      // Cluster is inclined.
      for (const auto &idx : cluster.indices) {
        outliers_cloud->points.push_back(result.downsampled_cloud->points[idx]);
      }
    }
  }

  // Add any downsampled points not part of any cluster to outliers.
  std::set<int> cluster_indices;
  for (const auto &cluster : clusters) {
    for (const auto &idx : cluster.indices) {
      cluster_indices.insert(idx);
    }
  }
  for (size_t i = 0; i < result.downsampled_cloud->points.size(); ++i) {
    if (cluster_indices.find(static_cast<int>(i)) == cluster_indices.end()) {
      outliers_cloud->points.push_back(result.downsampled_cloud->points[i]);
    }
  }

  // Propagate header information.
  inliers_cloud->header = result.downsampled_cloud->header;
  outliers_cloud->header = result.downsampled_cloud->header;

  // Build and return the result.
  result.inlier_cloud = inliers_cloud;
  result.outlier_cloud = outliers_cloud;
  result.pcl_method = "REGION GROWING SEGMENTATION (OMP Normals)";
  return result;
}

//=============================== Calculate Roughness (PCL) =================================================================================================

inline double calculateRoughnessPCL(PCLResult &result) {
  // Check if the plane coefficients are valid
  if (result.plane_coefficients->values.size() < 4 ||
      result.inlier_cloud->points.empty()) {
    std::cerr << "Invalid plane coefficients or empty inlier cloud. Cannot "
                 "compute roughness."
              << std::endl;
    return -1.0;
  }

  // Extract plane parameters (ax + by + cz + d = 0).
  double a = result.plane_coefficients->values[0];
  double b = result.plane_coefficients->values[1];
  double c = result.plane_coefficients->values[2];
  double d = result.plane_coefficients->values[3];

  // Calculate the plane normal's magnitude for normalization
  double norm = std::sqrt(a * a + b * b + c * c);

  // Variable to accumulate the squared distance of each point from the plane
  double sum_squared = 0.0;
  size_t N = result.inlier_cloud->points.size();

  // Loop over each point in the result.er cloud to calculate the roughness
  for (const auto &pt : result.inlier_cloud->points) {
    // Calculate the distance from the point to the plane
    double distance = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
    sum_squared += distance * distance;
  }

  // Return the square root of the average squared distance (roughness)
  return std::sqrt(sum_squared / static_cast<double>(N));
}

//============================= Calculate Relief (PCL) ======================================================================================================
// Calculate relief from the inlier cloud (safe landing zone).
inline double calculateReliefPCL(PCLResult &result) {
  if (!result.inlier_cloud || result.inlier_cloud->points.empty()) {
    std::cerr << "Error: Inlier cloud is empty." << std::endl;
    return -1.0;
  }

  double z_min = std::numeric_limits<double>::max();
  double z_max = std::numeric_limits<double>::lowest();

  // Iterate through inlier points and compute min and max z values.
  for (const auto &pt : result.inlier_cloud->points) {
    double z = pt.z;
    if (z < z_min)
      z_min = z;
    if (z > z_max)
      z_max = z;
  }

  return z_max - z_min;
}

//============================= Calculate Data Confidence (PCL)==============================================================================================

inline double calculateDataConfidencePCL(PCLResult &result) {
  if (!result.inlier_cloud || result.inlier_cloud->points.empty()) {
    std::cerr << "Error: Inlier cloud is empty." << std::endl;
    return -1.0;
  }

  size_t N = result.inlier_cloud->points.size();

  // Compute the convex hull of the inlier cloud projected onto a plane.
  pcl::ConvexHull<pcl::PointXYZI> chull;
  chull.setInputCloud(result.inlier_cloud);
  chull.setDimension(2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr hull_points(
      new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<pcl::Vertices> polygons;
  chull.reconstruct(*hull_points, polygons);

  if (polygons.empty() || hull_points->points.empty()) {
    std::cerr << "Error: Convex hull could not be computed." << std::endl;
    return -1.0;
  }

  // Compute the area of the first polygon using the shoelace formula.
  double area = 0.0;
  const std::vector<int> &indices = polygons[0].vertices;
  size_t n = indices.size();
  if (n < 3) {
    std::cerr
        << "Error: Convex hull does not have enough points to form an area."
        << std::endl;
    return -1.0;
  }

  for (size_t i = 0; i < n; i++) {
    const auto &p1 = hull_points->points[indices[i]];
    const auto &p2 = hull_points->points[indices[(i + 1) % n]];
    area += (p1.x * p2.y - p2.x * p1.y);
  }
  area = std::abs(area) / 2.0;

  if (area <= 0.0) {
    std::cerr << "Error: Computed hull area is non-positive." << std::endl;
    return -1.0;
  }

  double data_confidence = static_cast<double>(N) / area;
  return data_confidence;
}

enum MetricType {
  DATA_CONFIDENCE = 1, // 0001
  RELIEF = 2,          // 0010
  ROUGHNESS = 4,       // 0100
  ALL = 7              // 0111 (all metrics)
};

//=========================================== Rank Candidate Patch from PCL Result ===========================================================================

inline SLZDCandidatePoints
rankCandidatePatchFromPCLResult(PCLResult &result,
                                const std::string &metrics = "ALL") {
  // Create an SLZDCandidatePoints object for the result
  SLZDCandidatePoints candidate;
  // By default, the new SLZDCandidatePoints has dataConfidence, relief,
  // roughness, and score set to 0.0 in its constructor.

  // Check if the result contains a valid inlier cloud
  if (result.inlier_cloud && !result.inlier_cloud->points.empty()) {
    PCLResult surfResult;
    surfResult.inlier_cloud = result.inlier_cloud;
    surfResult.plane_coefficients = result.plane_coefficients;

    // Calculate metrics if "ALL" is selected
    if (metrics == "ALL") {
      candidate.dataConfidence = calculateDataConfidencePCL(surfResult);
      candidate.relief = calculateReliefPCL(surfResult);
      candidate.roughness = calculateRoughnessPCL(surfResult);
    } else if (metrics == "DATA_CONFIDENCE") {
      candidate.dataConfidence = calculateDataConfidencePCL(surfResult);
    } else if (metrics == "RELIEF") {
      candidate.relief = calculateReliefPCL(surfResult);
    } else if (metrics == "ROUGHNESS") {
      candidate.roughness = calculateRoughnessPCL(surfResult);
    } else {
      std::cerr << "[rankCandidatePatchFromPCLResult] Error: Invalid metric "
                   "specified."
                << std::endl;
    }

    // Compute the candidate patch's score.
    // If "ALL" or "DATA_CONFIDENCE" is selected => add dataConfidence
    // If "ALL" or "RELIEF" => subtract relief
    // If "ALL" or "ROUGHNESS" => subtract roughness
    double tempScore = 0.0;
    if (metrics == "ALL" || metrics == "DATA_CONFIDENCE") {
      tempScore += candidate.dataConfidence;
    }
    if (metrics == "ALL" || metrics == "RELIEF") {
      tempScore -= candidate.relief;
    }
    if (metrics == "ALL" || metrics == "ROUGHNESS") {
      tempScore -= candidate.roughness;
    }

    candidate.score = tempScore;

    // Print details of the candidate patch
    std::cout << "Candidate Patch Details:" << std::endl;
    if (metrics == "ALL" || metrics == "DATA_CONFIDENCE") {
      std::cout << "  Data Confidence: " << candidate.dataConfidence
                << std::endl;
    }
    if (metrics == "ALL" || metrics == "RELIEF") {
      std::cout << "  Relief: " << candidate.relief << std::endl;
    }
    if (metrics == "ALL" || metrics == "ROUGHNESS") {
      std::cout << "  Roughness: " << candidate.roughness << std::endl;
    }

    std::cout << "  Final Score: " << candidate.score << std::endl;
  } else {
    std::cerr
        << "[rankCandidatePatchFromPCLResult] Error: Inlier cloud is empty."
        << std::endl;
  }

  return candidate;
}

#endif
