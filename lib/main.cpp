#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <pcl/io/pcd_io.h>
#include "hazard_metrices.h"
#include "pointcloud_preprocessing.h"
#include "common.h"
#include "architecture.h"
#include <chrono>

using PointT = pcl::PointXYZI;

int main(int argc, char **argv)
{

    std::string config_file = "/home/airsim_user/Landing-Assist-Module-LAM/lib/config/config.yaml";
    // Load YAML configuration.
    YAML::Node config = YAML::LoadFile(config_file);
    YAML::Node params = config["ros__parameters"];

    // Set file paths.

    std::string pcd_file_path = params["pcd_file_path"].as<std::string>();

    float voxelSize = params["voxel_size"].as<float>();
    bool voxel_downsample_pointcloud = params["voxel_downsample_pointcloud"].as<bool>();
    bool visualize = params["visualize"].as<bool>();

    // Load input point cloud using the provided loadPCLCloud function.
    PCLResult pclResult;
    pclResult.downsampled_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    pclResult.inlier_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    auto loaded_cloud_pcl = loadPCLCloud<PointT>(pcd_file_path);


    PCLResult final_result;
    final_result.outlier_cloud = loaded_cloud_pcl;

    if (voxel_downsample_pointcloud)
    {
        // Downsample if necessary (here, using a voxel size of 0.45 as example).
        downsamplePointCloudPCL<PointT>(loaded_cloud_pcl, pclResult.inlier_cloud, voxelSize);
        std::cout << "Downsampled cloud has " << pclResult.inlier_cloud->points.size() << " points." << std::endl;
     
    }
    else
    {
        // pclResult.downsampled_cloud = loaded_cloud_pcl;
        pclResult.inlier_cloud = loaded_cloud_pcl;
    }
    // Start with the downsampled cloud.
    pcl::PointCloud<PointT>::Ptr current_cloud = pclResult.inlier_cloud;

    pcl::PointCloud<PointT>::Ptr sor_result;

    // Variable to store safe landing zones
    //  std::vector<typename pcl::PointCloud<PointT>::Ptr>> slz;

    // Get the pipeline configuration.
    YAML::Node pipeline = params["pipeline"];

    // Process each step in the pipeline sequentially.
    for (std::size_t i = 0; i < pipeline.size(); ++i)
    {
        std::string step = pipeline[i]["step"].as<std::string>();
        bool enabled = pipeline[i]["enabled"].as<bool>();
        if (!enabled)
        {
            std::cout << "\n--- Skipping disabled step: " << step << " ---\n";
            continue;
        }
        std::cout << "\n--- Running pipeline step: " << step << " ---\n";

        if (step == "SOR")
        {
            int nb_neighbors = pipeline[i]["parameters"]["nb_neighbors"].as<int>();
            double std_ratio = pipeline[i]["parameters"]["std_ratio"].as<double>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();

            // Convert pointcloud into open3d version
            OPEN3DResult pointcloud;
            auto pointCloud = convertPCLToOpen3D(pclResult);
            OPEN3DResult result = apply_sor_filter(pointCloud.inlier_cloud, nb_neighbors, std_ratio);
            auto pclResult = convertOpen3DToPCL(result);
            // sor_result = pcl_cloud.inlier_cloud;

            if (visualize)
            {
                visualizePCL(pclResult, visualization);
            }
            // current_cloud = pcl_cloud.inlier_cloud;

        }
        else if (step == "Radial")
        {
            double radius_search = pipeline[i]["parameters"]["radius_search"].as<double>();
            int min_neighbors = pipeline[i]["parameters"]["min_neighbors"].as<int>();

            PCLResult radial_result = applyRadiusFilter(pclResult.inlier_cloud, radius_search, min_neighbors);
            if (visualize)
            {
                visualizePCL(radial_result);
            }
            // current_cloud = radial_result.inlier_cloud;
            pclResult.inlier_cloud = radial_result.inlier_cloud;
        }
        else if (step == "Bilateral")
        {
            
            double sigma_s = pipeline[i]["parameters"]["sigma_s"].as<double>();
            double sigma_r = pipeline[i]["parameters"]["sigma_r"].as<double>();

            PCLResult bilateral_result = applyBilateralFilter(pclResult.inlier_cloud, sigma_s, sigma_r);
            if (visualize)
            {
                visualizePCL(bilateral_result);
            }
            // current_cloud = bilateral_result.inlier_cloud;
            pclResult.inlier_cloud = bilateral_result.inlier_cloud;
        }
        else if (step == "2dGridmap"){
            float resolution = pipeline[i]["parameters"]["resolution"].as<float>();
            pclResult.inlier_cloud = create2DGridMap(pclResult.inlier_cloud,resolution);
        }
        
        else if (step == "PROSAC")
        {
          
            float distanceThreshold = pipeline[i]["parameters"]["distanceThreshold"].as<float>();
            int maxIterations = pipeline[i]["parameters"]["maxIterations"].as<int>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();

            PCLResult prosac_result = performPROSAC(pclResult.inlier_cloud, distanceThreshold, maxIterations);
            pclResult.inlier_cloud = prosac_result.inlier_cloud;
            pclResult.plane_coefficients = prosac_result.plane_coefficients;
            
            if (visualize)
            {
                visualizePCL(prosac_result, visualization);
            }
        }
        else if (step == "RANSAC")
        {
          
            float distanceThreshold = pipeline[i]["parameters"]["distanceThreshold"].as<float>();
            int maxIterations = pipeline[i]["parameters"]["maxIterations"].as<int>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();

            PCLResult ransac_result = performRANSAC(pclResult.inlier_cloud, distanceThreshold, maxIterations);
            pclResult.inlier_cloud = ransac_result.inlier_cloud;
            pclResult.plane_coefficients = ransac_result.plane_coefficients;
            
            if (visualize)
            {
                visualizePCL(ransac_result, visualization);
            }
        }
        else if (step == "LMEDS")
        {
          
            float distanceThreshold = pipeline[i]["parameters"]["distanceThreshold"].as<float>();
            int maxIterations = pipeline[i]["parameters"]["maxIterations"].as<int>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();

            PCLResult lmeds_result = performLMEDS(pclResult.inlier_cloud, distanceThreshold, maxIterations);
            pclResult.inlier_cloud = lmeds_result.inlier_cloud;
            pclResult.plane_coefficients = lmeds_result.plane_coefficients;
            
            if (visualize)
            {
                visualizePCL(lmeds_result, visualization);
            }
        }
        
        else if (step == "PCA")
        {
            
            int k = pipeline[i]["parameters"]["k"].as<int>();
            float angleThreshold = pipeline[i]["parameters"]["angleThreshold"].as<float>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();

            PCLResult PCA_Result = PrincipleComponentAnalysis(pclResult.inlier_cloud, angleThreshold, k);
        
            pclResult.inlier_cloud = PCA_Result.inlier_cloud;
            pclResult.plane_coefficients = PCA_Result.plane_coefficients;
            
            if (visualize)
            {
                visualizePCL(PCA_Result, visualization);
            }
        }
        
        else if (step == "SphericalNeighbourhood")
        {
            auto start = std::chrono::high_resolution_clock::now();

            
            int k = pipeline[i]["parameters"]["k"].as<int>();
            float angleThreshold = pipeline[i]["parameters"]["angleThreshold"].as<float>();
            double radius = pipeline[i]["parameters"]["radius"].as<double>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();
            int landingZoneNumber = pipeline[i]["parameters"]["landingZoneNumber"].as<int>();
            int maxAttempts = pipeline[i]["parameters"]["maxAttempts"].as<int>();
            float textSize = pipeline[i]["parameters"]["visualization_textSize"].as<float>();
            
            PCLResult result;
    
            std::vector<SLZDCandidatePoints> candidatePoints;
            std::tie(result, candidatePoints) =kdtreeNeighbourhoodPCAFilterOMP(pclResult.inlier_cloud,
                                            radius, k, angleThreshold,
                                            landingZoneNumber, maxAttempts);
            
            
            // candidatePoints.push_back(finalCandidate);
            // Add plane coeffiecient to the struct we gonaa pass to calculate roughness
            result.plane_coefficients = pclResult.plane_coefficients;
            auto rankedCandidates = rankCandidatePatches(candidatePoints, result);
           
             // End the timer
            auto end = std::chrono::high_resolution_clock::now();
            // Calculate the elapsed time in seconds (or choose another unit)
            std::chrono::duration<double> duration = end - start;
            // Print the elapsed time
            std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
           
            pclResult.inlier_cloud = result.inlier_cloud;
            if (visualize)
            {
                // visualizePCL(result, visualization);
                visualizeRankedCandidatePatches(rankedCandidates, result,textSize);
                
            }
        }
        else if(step == "HazarMetrices"){
            std::string hazardMetricsName = pipeline[i]["parameters"]["hazard"].as<std::string>();
            auto hazard = rankCandidatePatchFromPCLResult(pclResult, hazardMetricsName);
        }
        else
        {
            std::cerr << "Unknown pipeline step: " << step << std::endl;
            return -1;
        }
    }

    return 0;
}
