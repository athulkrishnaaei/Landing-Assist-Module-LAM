#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.bt>" << std::endl;
        return 1;
    }

    const char* input_filename = argv[1];
    const char* output_filename = argv[2];

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_filename);
        return -1;
    }
    std::cout << "Loaded " << cloud->points.size() << " data points from " << input_filename << std::endl;

    // Create an Octree
    float resolution = 0.1;  // Change the resolution according to your needs
    octomap::OcTree octree(resolution);

    // Insert points into the Octree
    for (auto& point : *cloud) {
        octree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }

    // Update to reflect changes
    octree.updateInnerOccupancy();

    // Save the Octree to a .bt file
    octree.writeBinary(output_filename);
    std::cout << "Saved OctoMap to " << output_filename << std::endl;

    return 0;
}
