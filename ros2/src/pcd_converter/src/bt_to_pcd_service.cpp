// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/empty.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// using Empty = sensor_msgs::srv::Empty;
// using namespace std::chrono_literals;

// class OctomapToPCDService : public rclcpp::Node {
// public:
//     OctomapToPCDService() : Node("octomap_to_pcd_service") {
//         service_ = this->create_service<Empty>("convert_bt_to_pcd", std::bind(&OctomapToPCDService::handle_service, this, std::placeholders::_1, std::placeholders::_2));
//         RCLCPP_INFO(this->get_logger(), "Service 'convert_bt_to_pcd' is ready.");
//     }

// private:
//     void handle_service(const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>) {
//         RCLCPP_INFO(this->get_logger(), "Converting OctoMap .bt to PCD...");

//         // Specify your file names here
//         const char* input_filename = "/home/airsim_user/Downloads/Landing-Assist-Module-LAM/ros2/map2.bt";
//         const char* output_filename = "/home/airsim_user/Downloads/Landing-Assist-Module-LAM/ros2/output.pcd";

//         octomap::OcTree* octree = new octomap::OcTree(input_filename);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//         for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
//             if (octree->isNodeOccupied(*it)) {
//                 cloud->push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
//             }
//         }

//         pcl::io::savePCDFileASCII(output_filename, *cloud);
//         delete octree;
//         RCLCPP_INFO(this->get_logger(), "Saved %lu data points to %s", cloud->points.size(), output_filename);
//     }

//     rclcpp::Service<Empty>::SharedPtr service_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<OctomapToPCDService>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }






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

