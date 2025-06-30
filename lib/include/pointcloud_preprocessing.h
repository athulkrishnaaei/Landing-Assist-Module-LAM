#ifndef POINTCLOUD_PREPROCESSING_H
#define POINTCLOUD_PREPROCESSING_H

#include <common.h>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <thread>
#include <chrono>

#include <cmath>
// Open3D headers
#include <open3d/Open3D.h>
// Ocotmap headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
//PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/bilateral.h>        // Custom Bilateral Filter
#include <pcl/filters/impl/bilateral.hpp> // Implementation
#include <pcl/search/kdtree.h>             // Include for KdTree
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/grid_minimum.h>

#include <unordered_set>

#include <pcl/filters/statistical_outlier_removal.h>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

template <typename PointT>
using CloudInput = std::variant<std::string, typename pcl::PointCloud<PointT>::Ptr>;

using Open3DCloudInput = std::variant<std::string, std::shared_ptr<open3d::geometry::PointCloud>>;
 
 
 
//=============================== FILTERING OUTLIER REMOVAL ===============================================

inline OPEN3DResult apply_sor_filter(
    const Open3DCloudInput &input,
    int nb_neighbors,
    double std_ratio)
{
    OPEN3DResult result;
    result.open3d_method = "StatisticalOutlierRemoval";


    auto cloud= loadOpen3DCloud(input);
    // Apply SOR filter: RemoveStatisticalOutliers returns a pair: (filtered_cloud, inlier_indices)
    // Here, filtered_cloud contains the inliers (i.e. noise removed) and inlier_indices are their indices.
    auto [filtered_cloud, inlier_indices] = cloud->RemoveStatisticalOutliers(nb_neighbors, std_ratio);

    // Instead of manually computing the complement, we can use the invert flag in SelectByIndex.
    // In this case, the noise (outliers) are the complement of the inlier indices.
    auto noise_cloud = cloud->SelectByIndex(inlier_indices, true);

    // Swap the assignment so that:
    //   - result.inlier_cloud holds the noise (points removed by filtering)
    //   - result.outlier_cloud holds the filtered inlier points.
    result.inlier_cloud = filtered_cloud;
    result.outlier_cloud = noise_cloud;

    // For SOR filter, downsampled_cloud and plane_model are not applicable.
    result.downsampled_cloud = nullptr;
    result.plane_coefficients = Eigen::Vector4d(0, 0, 0, 0);

    return result;
}

// Reference Link: https://www.open3d.org/docs/0.6.0/cpp_api/namespaceopen3d_1_1geometry.html#add56e2ec673de3b9289a25095763af6d
// https://github.com/isl-org/Open3D/blob/main/cpp/open3d/geometry/PointCloud.cpp#L602

inline PCLResult applyRadiusFilter(
    const CloudInput<PointT>& input,
    double radius_search=0.9,
    int min_neighbors=40)
{
    PCLResult result;

    result.pcl_method = "Radius Outlier Removal";
    result.inlier_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    result.outlier_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    result.downsampled_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();

 
    auto cloud = loadPCLCloud<PointT>(input);
    result.downsampled_cloud = cloud;
    std::cout << "[INFO] Loaded " << result.downsampled_cloud->size();
              
    // Set up the Radius Outlier Removal filter.
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(result.downsampled_cloud);
    ror.setRadiusSearch(radius_search);
    ror.setMinNeighborsInRadius(min_neighbors);

    // First pass: get inliers (points that meet the criteria).
    ror.setNegative(false);
    ror.filter(*result.inlier_cloud);
    // result.inlier_cloud = result.inlier_cloud;
    std::cout << "[INFO] Applied Radius Outlier Removal for inliers. Cloud size: " 
              << result.inlier_cloud->size() << std::endl;

    // Second pass: get outliers (points that do not meet the criteria).
    ror.setNegative(true);
    ror.filter(*result.outlier_cloud);
    // result.outlier_cloud = cloud_outliers;
    std::cout << "[INFO] Applied Radius Outlier Removal for outliers. Cloud size: " 
              << result.outlier_cloud->size() << std::endl;

    return result;
}

//// Referencle Link : http://pointclouds.org/documentation/classpcl_1_1_radius_outlier_removal.html
////                   https://github.com/PointCloudLibrary/pcl/blob/master/filters/src/radius_outlier_removal.cpp#L47




//=============================== FILTERING SMOOTHING ======================================================

// Function to apply Bilateral Filter

inline PCLResult applyBilateralFilter(
    const CloudInput<PointT>& input,
    double sigma_s,
    double sigma_r)
{
    PCLResult result;
    result.pcl_method = "BilateralFilter";
    result.inlier_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    result.outlier_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    result.downsampled_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    
  
    auto cloud= loadPCLCloud<PointT>(input);
    result.downsampled_cloud = cloud;
    std::cout << "[INFO] Loaded " << cloud->size();
            


    // Initialize the bilateral filter
    pcl::BilateralFilter<PointT> bilateral_filter;


    // Set the input cloud
    bilateral_filter.setInputCloud(result.downsampled_cloud);
    
    // Set filter parameters
    bilateral_filter.setHalfSize(sigma_s);
    bilateral_filter.setStdDev(sigma_r);
    
    // Use KdTree as the search method
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    bilateral_filter.setSearchMethod(tree);

    // result.inlier_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    // Apply the filter
    bilateral_filter.filter(*result.inlier_cloud);

    std::cout << "[INFO] Applied Bilateral Filter. Filtered cloud size: " 
              << result.inlier_cloud->size() << std::endl;

    // Store the filtered cloud
  

    // No inlier/outlier separation or plane fitting in bilateral filtering
    result.outlier_cloud = nullptr;
    result.plane_coefficients = nullptr;

    return result;
}

//   Reference Link: https://pointclouds.org/documentation/classpcl_1_1_bilateral_filter.html
//                   https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/bilateral.h#L56

#endif // POINTCLOUD_PREPROCESSING_H
