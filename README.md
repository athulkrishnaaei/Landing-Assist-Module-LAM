# Landing-Assist-Module-LAM
For Autonomous UAV landing a LAM addresses a sequence of problems as shown in the fig below.

![LAM](https://github.com/Robotgir/Landing-Assist-Module-LAM/assets/47585672/b35b1021-e451-4f36-b16c-2d00289be256)

This repo is work in progress.
Focusing on Point-Cloud based Safe Landing Zone Detection PC-SLZD algorithms at the moment.

# Point Cloud Publisher and Plane Detection

This guide explains how to use the `pointcloud_publisher` to publish a point cloud and apply different plane detection methods using the `pointcloud_plane_detection` package.

## Prerequisites
1. git clone https://github.com/athulkrishnaaei/Landing-Assist-Module-LAM.git
2. cd Landing-Assist-Module-LAM
3. git checkout develop
4. cd ros2
5. colcon build
2. Download the example point cloud file from [here](https://drive.google.com/file/d/1NYAtHWjuo6R7qI4s55TbW7oRZUM73guQ/view?usp=sharing).
3. Save the file path for use in the commands below.

## Publishing Your Point Cloud

To publish the point cloud using the `pointcloud_publisher` node:

ros2 run pointcloud_publisher pointcloud_publisher --ros-args -p file_path:=/path/to/your/cloud.pcd


## Plane Detection Methods

You can apply different plane detection methods to the published point cloud using the pointcloud_plane_detection package.

## 1 RANSAC
To apply the RANSAC method:

ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=ransac

## 2 PROSAC
To apply the PROSAC method:

ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=prosac

## 3 PLANAR PATCH
To apply the PLANAR PATCH method:

ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=planar_patch

## Using pointcloud from Unreal engine 

## Prerequisites
1. git clone https://github.com/athulkrishnaaei/Landing-Assist-Module-LAM.git
2. cd Landing-Assist-Module-LAM
3. git checkout develop
4. cd ros2
5. colcon build
6. Download the example ros bag file from [here] https://drive.google.com/drive/folders/1rGiJMeT2GD3_851eh8hQ2LQEv7G3B-eL?usp=sharing

## Publishing Your Point Cloud
1. cd my_bag
2. ros2 bag play my_bag or ros2 bag play /path_to_bag_file

## Plane Detection Methods

You can apply different plane detection methods to the published point cloud using the pointcloud_plane_detection package.

To avoid using remap you can manaully change the topic name in plane_detection.py [a link] https://github.com/athulkrishnaaei/Landing-Assist-Module-LAM/blob/develop/ros2/src/pointcloud_plane_segmentation/pointcloud_plane_segmentation/plane_segmentation.py 
## change line 190

## 1 RANSAC
To apply the RANSAC method:

ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=ransac --remap /pointcloud:=/airsim_node/PX4/lidar/Lidar1

## 2 PROSAC
To apply the PROSAC method:

ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=prosac --remap /pointcloud:=/airsim_node/PX4/lidar/

## 3 PLANAR PATCH
To apply the PLANAR PATCH method:

ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=planar_patch --remap /pointcloud:=/airsim_node/PX4/lidar/