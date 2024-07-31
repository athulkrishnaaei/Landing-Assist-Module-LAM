#include <octomap/octomap.h>      // For handling OcTree
#include <pcl/io/pcd_io.h>        // For saving PCD files
#include <pcl/point_cloud.h>      // For handling point cloud
#include <pcl/point_types.h>      // For using XYZ point type

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.bt> <output.pcd>" << std::endl;
        return 1;
    }

    const char* input_filename = argv[1];
    const char* output_filename = argv[2];

    // Load the .bt file
    octomap::OcTree* octree = new octomap::OcTree(input_filename);

    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate through all leaf nodes in the octree
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            // Node is occupied: add a point to the cloud
            cloud->push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
        }
    }

    // Save the point cloud to a PCD file
    pcl::io::savePCDFileASCII(output_filename, *cloud);

    delete octree;  // Clean up
    std::cout << "Saved " << cloud->points.size() << " data points to " << output_filename << std::endl;

    return 0;
}