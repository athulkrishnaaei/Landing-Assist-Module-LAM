#include <octomap/octomap.h>      // For handling OcTree
#include <pcl/io/pcd_io.h>        // For loading PCD files
#include <pcl/point_cloud.h>      // For handling point cloud
#include <pcl/point_types.h>      // For using XYZ point type

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.bt>" << std::endl;
        return 1;
    }

    const char* input_filename = argv[1];
    const char* output_filename = argv[2];

    // Load the point cloud from the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_filename, *cloud) == -1) {
        std::cerr << "Couldn't read the PCD file " << input_filename << std::endl;
        return -1;
    }

    // Create an empty octree with a resolution of 0.1
    double resolution = 0.1;
    octomap::OcTree octree(resolution);

    // Iterate through the point cloud and add points to the octree
    for (const auto& point : cloud->points) {
        octree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }

    // Update the octree (optional: prune the tree to remove redundancies)
    octree.updateInnerOccupancy();

    // Write the octree to a .bt file
    octree.writeBinary(output_filename);

    std::cout << "Saved octree to " << output_filename << std::endl;

    return 0;
}
