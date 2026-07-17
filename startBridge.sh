#!/bin/bash
echo "Starting Gazebo camera bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "/world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
--ros-args \
-p image_queue_size:=1
