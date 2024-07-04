#for publishing point cloud data for testing run
ros2 run pointcloud_publisher pointcloud_publisher

#for applying ransac plane detection on the published point cloud run
ros2 run pointcloud_plane_segmentation plane_segmentation method:=ransac

#for applying prosac plane detection on the published point cloud run
ros2 run pointcloud_plane_segmentation plane_segmentation method:=prosac