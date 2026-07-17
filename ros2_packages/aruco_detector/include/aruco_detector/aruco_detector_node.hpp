#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

class ArUcoDetectorNode : public rclcpp::Node
{
public:
  ArUcoDetectorNode();

private:
  // Subscribers with message_filters for synchronization
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subscriber_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> camera_info_subscriber_;
  
  // Synchronizer for approximate time sync
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // Camera parameters
  cv::Mat camera_matrix_; // 3×3 matrix of focal length, principal point
  cv::Mat dist_coeffs_; // 5 distortion coefficients (lens aberrations)
  bool camera_info_received_ = false;

  // ArUco parameters
  float marker_length_ = 1.0f;  // Physical marker size: 1 meter
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;

  // Callback for synchronized image and camera info
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

  // Helper to extract camera matrix and distortion coefficients
  void updateCameraParameters(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

  // Helper to detect and estimate pose
  void detectAndPublishPose(const cv::Mat& image, const std_msgs::msg::Header& header);
};