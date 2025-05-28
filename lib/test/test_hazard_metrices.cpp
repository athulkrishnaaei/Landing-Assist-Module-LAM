#include "hazard_metrices.h"
#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include "hazard_metrices.h"
#include <gtest/gtest.h>
#include <string>
#include <iostream>

// Global flag to control visualization in tests.
bool g_skipVisualization = false;

// Input file path (adjust if needed)
static const std::string filePath = "/home/airsim_user/Landing-Assist-Module-LAM/test.pcd";

//--------------------------------------------------------------------------
// Test 1: PCA / Normal Estimation / Classification (PCL)
//--------------------------------------------------------------------------
TEST(HazardMetricesTest, TestPCA_NormalEstimation) {
    float slope_threshold = 5.0f;
    int k = 10;
    // Call the PCA-based classification function.
    PCLResult result =  PrincipleComponentAnalysis(filePath,
                                                
                                                    slope_threshold,
                                                    k);
    if (!g_skipVisualization) {
        // Visualize the result; press 'q' to close the viewer.
        visualizePCL(result);
    } else {

        std::cout << "[TestPCA_NormalEstimation] Inliers: " << result.inlier_cloud->size()  << ", Outliers: " << result.outlier_cloud->size() << std::endl;
    }
    // Check that some points have been classified.
    EXPECT_GT(result.inlier_cloud->size() + result.outlier_cloud->size(), 0);

}
//--------------------------------------------------------------------------
// Test 2: PCL-Based RANSAC Segmentation
//-------------------------------------------------------------------------- 
TEST(HazardMetricesTest, TestPCL_RANSAC) {
    float distanceThreshold = 1.9f;
    int maxIterations = 1000;

    // Call the PCL RANSAC segmentation function.
    PCLResult result = performRANSAC(filePath, distanceThreshold, maxIterations);

    if (!g_skipVisualization) {
        visualizePCL(result);
    } else {
        std::cout << "[TestPCL_RANSAC] Inliers: " << result.inlier_cloud->size()
                  << ", Outliers: " << result.outlier_cloud->size() << std::endl;
    }
    // Verify that both inliers and outliers were found.
    EXPECT_GT(result.inlier_cloud->size(), 0);
    EXPECT_GT(result.outlier_cloud->size(), 0);
}

//--------------------------------------------------------------------------
// Test 3: PROSAC Segmentation (PCL-Based)
//--------------------------------------------------------------------------
TEST(HazardMetricesTest, TestPROSAC) {
    float distanceThreshold = 1.9f;
    int maxIterations = 200;

    // Call the PROSAC segmentation function.
    PCLResult result = performPROSAC(filePath, distanceThreshold, maxIterations);

    if (!g_skipVisualization) {
        visualizePCL(result);
    } else {
        std::cout << "[TestPROSAC] Inliers: " << result.inlier_cloud->size()
                  << ", Outliers: " << result.outlier_cloud->size() << std::endl;
    }
    // Check that segmentation produced inliers and outliers.
    EXPECT_GT(result.inlier_cloud->size(), 0);
    EXPECT_GT(result.outlier_cloud->size(), 0);
}


//--------------------------------------------------------------------------
// Test 4: LMEDS Plane Fitting (PCL-Based)
//--------------------------------------------------------------------------
TEST(HazardMetricesTest, TestLMEDS) {
    float distanceThreshold = 1.9f;
    int maxIterations = 100;

    // Call the LMEDS segmentation function.
    PCLResult result = performLMEDS(filePath, distanceThreshold, maxIterations);

    if (!g_skipVisualization) {
        visualizePCL(result);
    } else {
        std::cout << "[TestLMEDS] Inliers: " << result.inlier_cloud->size()
                  << ", Outliers: " << result.outlier_cloud->size() << std::endl;
    }
    // Ensure that inliers and outliers are found.
    EXPECT_GT(result.inlier_cloud->size(), 0);
    EXPECT_GT(result.outlier_cloud->size(), 0);
}

//--------------------------------------------------------------------------
// Test 5: Calculate Roughness (PCL-Based)
//--------------------------------------------------------------------------
TEST(HazardMetricesTest, TestRoughnessPCL) {
    float distanceThreshold = 1.9f;
    int maxIterations = 200;

    // Perform PROSAC segmentation (PCL-based)
    PCLResult result = performPROSAC(filePath, distanceThreshold, maxIterations);

    // Calculate roughness using the PCL-based method.
    double roughness = calculateRoughnessPCL(result);
    std::cout << "[TestRoughnessPCL] Roughness of the point cloud: " << roughness << std::endl;
    
    // Verify that a valid roughness value was calculated.
    EXPECT_GE(roughness, 0);
}

//--------------------------------------------------------------------------
// Test 6: Calculate Relief (PCL-Based)
//--------------------------------------------------------------------------
TEST(HazardMetricesTest, TestReliefPCL) {
    float distanceThreshold = 1.9f;
    int maxIterations = 200;

    // Perform PROSAC segmentation (PCL-based)
    PCLResult result = performPROSAC(filePath, distanceThreshold, maxIterations);

    // Calculate relief using the PCL-based method.
    double relief = calculateReliefPCL(result);
    std::cout << "[TestReliefPCL] Relief of the landing zone (PCL-based): " << relief << std::endl;
    
    // Verify that a valid relief value was calculated.
    EXPECT_GE(relief, 0);
}
//--------------------------------------------------------------------------
// Test 7: Calculate Data Confidence (PCL-Based)
//--------------------------------------------------------------------------
TEST(HazardMetricesTest, TestDataConfidencePCL) {
    float distanceThreshold = 1.9f;
    int maxIterations = 200;

    // Perform PROSAC segmentation (PCL-based)
    PCLResult result = performPROSAC(filePath, distanceThreshold, maxIterations);

    // Calculate data confidence using the PCL-based method.
    double data_confidence = calculateDataConfidencePCL(result);
    std::cout << "[TestDataConfidencePCL] Data confidence (PCL-based): " << data_confidence << std::endl;
    
    // Verify that a valid data confidence value was calculated.
    EXPECT_GE(data_confidence, 0);
}

//--------------------------------------------------------------------------
// main() for Google Test
//--------------------------------------------------------------------------
int main(int argc, char **argv) {
    // Process additional command-line arguments.
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--no-vis") {
            g_skipVisualization = true;
        }
    }
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

