# Point Cloud Publisher and Plane Detection

This guide explains how to use the `pointcloud_publisher` to publish a point cloud and apply different plane detection methods using the `pointcloud_plane_detection` package.

## Prerequisites

1. Download the example point cloud file from [here](https://drive.google.com/file/d/1NYAtHWjuo6R7qI4s55TbW7oRZUM73guQ/view?usp=sharing).
2. Save the file path for use in the commands below.

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