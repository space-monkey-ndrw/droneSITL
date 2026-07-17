# ArUco Detector Node Setup Guide

## Build Instructions

```bash
cd /path/to/aruco_detector_ws
colcon build --symlink-install
source install/setup.bash
```

## Run Instructions

**Terminal 1:** Start Gazebo
```bash
./startDrone.sh
```
Wait until Gazebo fully loads.

**Terminal 2:** Start the bridge
```bash
./startBridge.sh
```

**Terminal 3:** Run the ArUco detector node
```bash
cd /path/to/aruco_detector_ws
source install/setup.bash
ros2 run aruco_detector aruco_detector_node
```

Or use the launch file:
```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

## Verify Output

Monitor the published pose topic:
```bash
ros2 topic echo /aruco_pose
```

View camera feed to verify marker detection:
```bash
ros2 run image_tools showimage --ros-args -r image:=/world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image
```

## Implementation Details

### OpenCV 4.6 API
- Uses old ArUco API: `cv::aruco::detectMarkers()` and `cv::aruco::estimatePoseSingleMarkers()`
- Dictionary: `DICT_4X4_50`
- Marker length: 1.0 meters

### Message Synchronization
- Uses `message_filters` with `ApproximateTime` policy to sync image and camera_info
- Queue size: 10

### Output
- Topic: `/aruco_pose` (geometry_msgs/PoseStamped)
- Frame ID: `camera_link`
- Contains 3D position and orientation (as quaternion) of marker relative to camera

### Marker Detection Notes
- Currently looks for marker ID 0
- If marker has inverted colors (white-on-black), may need to:
  - Invert the image in code: `cv::bitwise_not(gray, gray);`
  - Adjust detector parameters

## Troubleshooting

**No markers detected:**
1. Verify camera_info is being published (check ROS_DOMAIN_ID=99)
2. Check camera feed with image_tools showimage
3. Try inverting image if marker colors are inverted
4. Adjust detector parameters in node constructor

**Pose values seem wrong:**
1. Verify camera intrinsics are correct
2. Ensure marker_length_ matches actual physical marker size
3. Check that tvecs are in camera frame (not world frame) - transform needed for Phase 3

**Synchronization issues:**
1. Check both topics are publishing at reasonable rates (~15 Hz camera expected)
2. Try increasing queue size in synchronizer if needed
