#include "pointcloud_preprocessing.h"
#include <gtest/gtest.h>
#include <fstream>
#include <common.h>


// Global flag to control visualization in tests.
bool g_skipVisualization = false;

// Change this file path if needed.
static const std::string filePath = "/home/airsim_user/Landing-Assist-Module-LAM/test.pcd";

// Helper function to check if a file exists.
bool fileExists(const std::string &path) {
    std::ifstream f(path.c_str());
    return f.good();
}



////////////////////////////////////////////////////////////
// TESTS for FILTERING functions
////////////////////////////////////////////////////////////

TEST(OPEN3DFiltering, ApplySORFilter) {
    int nb_neighbors = 15;
    double std_ratio = 0.1;
    OPEN3DResult result = apply_sor_filter(filePath, nb_neighbors, std_ratio);
    

    if (!g_skipVisualization) {
        // Visualize the segmentation result; press 'q' to close the window.
        visualizeOPEN3D(result);
    } 
    EXPECT_NE(result.inlier_cloud, nullptr);
    EXPECT_NE(result.inlier_cloud, nullptr);
    if(result.inlier_cloud)
    {
        EXPECT_GT(result.inlier_cloud->points_.size(), 0);
    }

}


TEST(PCLFiltering, ApplyRadiusFilter) {
    // Load original cloud.
    double radius_search = 0.9; //0.1 to 0.3,0.3 to 0.7,0.7 to 0.15
    int min_neighbors = 50;      // 5 to 15,10 to 30,20 to 50
    PCLResult result = applyRadiusFilter(filePath, radius_search, min_neighbors);
    
    if (!g_skipVisualization) {
        // Visualize the segmentation result; press 'q' to close the window.
        visualizePCL(result);
    } 
    
    // Check that the filtered cloud is not empty.
    EXPECT_FALSE(result.inlier_cloud->empty());
}

TEST(PCLFiltering, ApplyBilateralFilter) {

       
    double sigma_s = 15.0; // Small point clouds or detailed structures: sigma_s = 1.0 - 5.0 ,Noisy or dense point clouds: sigma_s = 5.0 - 10.0,Large or very noisy point clouds: sigma_s = 10.0 - 15.0
    double sigma_r = 0.3;  //Preserve edges and details: sigma_r = 0.05 - 0.1, Moderate smoothing: sigma_r = 0.1 - 0.2, Heavy denoising (risk of over-smoothing): sigma_r = 0.2 - 0.3

    PCLResult result = applyBilateralFilter(filePath, sigma_s, sigma_r);

    
    if (!g_skipVisualization) {
        visualizePCL(result);
    } 

    // Check that the bilateral filtered cloud is not empty.
    EXPECT_FALSE(result.inlier_cloud->empty());
}

////////////////////////////////////////////////////////////
// Main function for Google Test
////////////////////////////////////////////////////////////

int main(int argc, char **argv) {

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--no-vis") {
            g_skipVisualization = true;
        }
    }
    ::testing::InitGoogleTest(&argc, argv);
    // Note: We do not call visualization functions here in order not to block the tests.
    return RUN_ALL_TESTS();
}
