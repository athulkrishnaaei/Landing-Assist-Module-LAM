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

```bash
ros2 run pointcloud_publisher pointcloud_publisher --ros-args -p file_path:=/path/to/your/cloud.pcd
```

## Plane Detection Methods

You can apply different plane detection methods to the published point cloud using the pointcloud_plane_detection package.

source your environment before running the code 
cd ros2 
source install/setup.bash
## 1 RANSAC
To apply the RANSAC method:
```bash
ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=ransac
```
## 2 PROSAC
To apply the PROSAC method:
```bash
ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=prosac
```
## 3 PLANAR PATCH
To apply the PLANAR PATCH method:
```bash
ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=planar_patch
```
## 4 Launch rviz
rviz2 -d /path_to_defaul.rviz file 
default.rviz is located at pointcloud_plane_detection/config/default.rviz

To see the pointcloud change the topic name under pointcloud2 to /pointcloud

To see detected planes change the topic name under pointcloud2 to /detected_planes

## Watch the Video

[![Watch the video](https://img.youtube.com/vi/lce0nvfPICo/0.jpg)](https://youtu.be/lce0nvfPICo?si=43Ronj9vu1_F6V7A)


## Using Ros2 bag file created from Unreal engine 

## Prerequisites
1. git clone https://github.com/athulkrishnaaei/Landing-Assist-Module-LAM.git
2. cd Landing-Assist-Module-LAM
3. git checkout develop
4. cd ros2
5. colcon build
6. Download the example ros bag file from [here] https://drive.google.com/drive/folders/1BH9uTZXAa8oBXjFqKlQPvcEAp5Yt7zXi?usp=sharing

## Publishing Your Point Cloud
1. cd my_bag
```bash
ros2 bag play my_bag or ros2 bag play /path_to_bag_file
```

2 Launch rviz2
  rviz2 -d /path_to/rosbag.rviz file 
  rosbag.rviz is located at rviz_config

  To see pointcloud change the topic name under pointcloud2 to /airsim_node/PX4/lidar/Lidar1 and change fixed frame to world_enu

  To see detected planes change the topic name under pointcloud2 to /detected_planes and change fixed frame into map


## Plane Detection Methods

You can apply different plane detection methods to the published point cloud using the pointcloud_plane_detection package.

To avoid using remap you can manaully change the topic name in plane_detection.py [a link] https://github.com/athulkrishnaaei/Landing-Assist-Module-LAM/blob/develop/ros2/src/pointcloud_plane_segmentation/pointcloud_plane_segmentation/plane_segmentation.py 
## change line 190

source your environment before running the code 
cd ros2 
source install/setup.bash

## 1 RANSAC
To apply the RANSAC method:

```bash
ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=ransac --remap /pointcloud:=/airsim_node/PX4/lidar/Lidar1
```

## 2 PROSAC
To apply the PROSAC method:

```bash
ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=prosac --remap /pointcloud:=/airsim_node/PX4/lidar/Lidar1
```

## 3 PLANAR PATCH
To apply the PLANAR PATCH method:

```bash
ros2 run pointcloud_plane_detection plane_detection --ros-args -p method:=planar_patch --remap /pointcloud:=/airsim_node/PX4/lidar/Lidar1
```

[![Applying plane detection with rosbag file](https://img.youtube.com/vi/XihAnF-MoZY/0.jpg)](https://youtu.be/XihAnF-MoZY?si=Vi4I29JDdzPJ1LcO)

## Saving Lidar Data As a map

## Prerequisites
With this package we can convert lidar messages into pcd using OCTOMAP package,this package is a modified version of https://github.com/iKrishneel/octomap_server2.git

sudo apt-get install ros-foxy-octomap-msgs

cd ros2/src/

Clone the dependency repositories to the workspace
```bash
vcs import . < deps.repos
```
will clone octomap_msgs to the workspace

#### Building
cd ros2/src
Use colcon to build the workspace
```bash
colcon build --symlink-install --packages-select octomap_server2
```

Then build the whole package by going to ros2 by ignoring octomap_server2

```bash
colcon build --symlink-install --packages-ignore octomap_server2 airsim_ros
```
## With this package we can generate PCD version of map from the live lidar data which is published from airsim or we can generate it from by playing a rosbag file 

## Converting rosbag file into pcd 

Download the example ros bag file from [here] https://drive.google.com/drive/folders/1BH9uTZXAa8oBXjFqKlQPvcEAp5Yt7zXi?usp=sharing


1. Launch octomap server


cd ros2/src
source install/setup.bash 

cd ros2
source install/setup.bash 

```bash
ros2 launch octomap_server2 octomap_server_launch.py
```
2. Launch rviz2
   octomap.rviz file is in rviz_config folder
   rviz2 -d /path_to/octomap.rviz file 
   

3. cd my_bag

```bash
ros2 bag play my_bag or ros2 bag play /path_to_bag_file
```

4. To save pointcloud as .bt file 

```bash
ros2 run octomap_server2 octomap_saver_node --ros-args -p full:=false -p octomap_path:='map.bt'
```
or with launch file 

```bash
ros2 launch octomap_server2 octomap_saver_launch.py 
```

5. Converting .bt file into PCD

```bash
ros2 run my_octomap_converter octomap_to_pcd /path_to/map3.bt /path_to/map.pcd 
```

## If you want to convert a pcd file into .bt file 

Converting .pcd file into .bt

```bash
ros2 run my_octomap_converter pcd_to_octomap /path_to/map3.pcd /path_to/map.bt 
```

## Watch the Video

[![Creating pcd from rosbag file using octomap](https://img.youtube.com/vi/cDaW_hDrfdA/0.jpg)](https://youtu.be/cDaW_hDrfdA?si=w2jpNsvu7kT7NY4G)

## Creating PCD version of map from live lidar data 

Instead of step 3 in the above 

1. Run the docker 

sudo docker run --name uav --rm -it --privileged -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -e XDG_RUNTIME_DIR=/run/user/$UID -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d -v /lib:/lib -e XR_LOADER_DEBUG=all  -v /tmp/.X11-unix:/tmp/.X11-unix -v $XAUTHORITY:$XAUTHORITY -v /home/athul/test2/airsim_ros2:/home/airsim_user/workspace/AirSim -v /home/athul/Downloads/AbandonedPark:/home/airsim_user/Downloads -v ~/.ssh:/home/athul/Documents/.ssh -v /home/athul/Hiwi/Landing-Assist-Module-LAM:/home/airsim_user/Downloads/Landing-Assist-Module-LAM --gpus all --entrypoint /bin/bash giri6937/ue_airsim_inside_docker:foxy 

## for opening multiple terminals use this code 

    docker exec -it uav bash 

2. Launch AbandonedPark in Unreal Engine

    1. cd Downloads/LinuxNoEditor/
    2. ./AbandonedPark.sh -windowed -ResX=1080 -ResY=720 --no-border

3. Launch airsim node
    if you have used vcs import . < deps.repos command before airsim_ros package will be downloaded to src file if not you can 

    ```bash
    git clone https://github.com/athulkrishnaaei/airsim_ros.git 
    ```
    to the  src folder 

    ## Install Airsim
    cd src/airsim_ros/
    ./setup.sh
    ./build.sh

    ## Launch airsim node 
    cd src/airsim_ros/ros2
    colcon build
    source install/setup.bash
    
    ros2 launch airsim_ros_pkgs airsim_node.launch.py 

4. Move the drone in square path 

   1. cd src/ros2/airsim_ros/PythonClient/LAM
   2. python3 drone_square_path.py

   Now the drone will move in square path and publish the lidar data

Now you have live lidar data being published 

After this we can continue with the step 4 in the above [To save pointcloud as .bt file] 

