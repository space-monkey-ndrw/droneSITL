#include "aruco_detector/aruco_detector_node.hpp"
#include <opencv2/imgproc.hpp>

ArUcoDetectorNode::ArUcoDetectorNode() : rclcpp::Node {"aruco_detector_node"}
{
  // Initialize ArUco dictionary (OpenCV 4.6 old API)
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  
  // Create detector parameters
  parameters_ = cv::aruco::DetectorParameters::create();
  
  // Create subscribers with message_filters for time synchronization
  image_subscriber_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image");
  
  camera_info_subscriber_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
    this, "/world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info");
  
  // Create synchronizer with approximate time policy
  synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10),  // queue size
    *image_subscriber_,
    *camera_info_subscriber_);
  
  synchronizer_->registerCallback(
    std::bind(&ArUcoDetectorNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // Create publisher for pose
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "aruco_pose", 10);
  
  RCLCPP_INFO(this->get_logger(), "ArUco Detector Node initialized");
  RCLCPP_INFO(this->get_logger(), "Using dictionary: DICT_4X4_50");
  RCLCPP_INFO(this->get_logger(), "Marker length: %.1f meters", marker_length_);
}

void ArUcoDetectorNode::updateCameraParameters(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg)
{
  // Extract camera matrix K (3x3)
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = camera_info_msg->k[0];  // fx
  camera_matrix_.at<double>(0, 2) = camera_info_msg->k[2];  // cx
  camera_matrix_.at<double>(1, 1) = camera_info_msg->k[4];  // fy
  camera_matrix_.at<double>(1, 2) = camera_info_msg->k[5];  // cy
  
  // Extract distortion coefficients (typically 5-element: k1, k2, p1, p2, k3)
  dist_coeffs_ = cv::Mat(5, 1, CV_64F);
  for (int i = 0; i < 5 && i < static_cast<int>(camera_info_msg->d.size()); ++i) {
    dist_coeffs_.at<double>(i, 0) = camera_info_msg->d[i];
  }
  
  camera_info_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera calibration received");
}

void ArUcoDetectorNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg)
{
  // Update camera parameters from camera_info
  if (!camera_info_received_) {
    updateCameraParameters(camera_info_msg);
  }
  
  // Convert ROS image to OpenCV Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  // Detect markers and publish pose
  detectAndPublishPose(cv_ptr->image, image_msg->header);
}

void ArUcoDetectorNode::detectAndPublishPose(const cv::Mat& image, const std_msgs::msg::Header& header)
{
  if (!camera_info_received_) {
    return;  // Wait for camera calibration
  }
  
  // Convert RGB to grayscale for detection
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  
  // Detect markers using OpenCV 4.6 old API
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> rejected;
  
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, parameters_, rejected);
  
  if (ids.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No markers detected");
    return;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Detected %lu marker(s)", ids.size());
  
  // Estimate pose for each marker using OpenCV 4.6 old API
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
  
  // Find marker with ID 0 and publish its pose
  for (size_t i = 0; i < ids.size(); ++i) {
    if (ids[i] == 0) {
      // Create PoseStamped message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = header;
      pose_msg.header.frame_id = "camera_link";
      
      // Set position (translation vector from camera to marker)
      pose_msg.pose.position.x = tvecs[i][0];
      pose_msg.pose.position.y = tvecs[i][1];
      pose_msg.pose.position.z = tvecs[i][2];
      
      // Convert rotation vector to quaternion
      cv::Mat rotation_mat;
      cv::Rodrigues(rvecs[i], rotation_mat);
      
      double trace = rotation_mat.at<double>(0, 0) + rotation_mat.at<double>(1, 1) + rotation_mat.at<double>(2, 2);
      double w = std::sqrt(1.0 + trace) / 2.0;
      double x = (rotation_mat.at<double>(2, 1) - rotation_mat.at<double>(1, 2)) / (4.0 * w);
      double y = (rotation_mat.at<double>(0, 2) - rotation_mat.at<double>(2, 0)) / (4.0 * w);
      double z = (rotation_mat.at<double>(1, 0) - rotation_mat.at<double>(0, 1)) / (4.0 * w);
      
      pose_msg.pose.orientation.w = w;
      pose_msg.pose.orientation.x = x;
      pose_msg.pose.orientation.y = y;
      pose_msg.pose.orientation.z = z;
      
      // Publish
      pose_publisher_->publish(pose_msg);
      
      RCLCPP_DEBUG(this->get_logger(), "Marker 0 pose - x: %.3f, y: %.3f, z: %.3f",
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
      
      break;
    }
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArUcoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
