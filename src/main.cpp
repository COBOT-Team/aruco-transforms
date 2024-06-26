#include <rclcpp/rclcpp.hpp>

#include "aruco_transforms/aruco_object_manager.hpp"
#include "aruco_transforms/config.hpp"

using namespace aruco_object_manager;
using namespace aruco_transforms_params;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

// List of object managers. This is populated in main.
vector<ArucoObjectManager> object_managers;

/**
 * Callback for the camera. This will be called whenever a new image is received and it will update
 * all of the object managers.
 *
 * @param[in] node ROS node, used for logging.
 * @param[in] image_msg The image message.
 * @param[in] camera_info_msg The camera info message.
 */
void camera_callback(rclcpp::Node::SharedPtr node,
                     const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("aruco_transforms");

  ParamListener param_listener(node);
  Params params = param_listener.get_params();

  image_transport::ImageTransport it(node);
  auto tf_broadcaster = make_shared<tf2_ros::TransformBroadcaster>(node);

  // Chessboard.
  object_managers.emplace_back(ArucoObjectManager(node, tf_broadcaster, it, params.chessboard.frame,
                                                  params.chessboard.warped.topic, CHESSBOARD_PARAMS,
                                                  params.chessboard.warped.size, false));

  // Table.
  object_managers.emplace_back(ArucoObjectManager(node, tf_broadcaster, it, params.table.frame,
                                                  params.table.warped.topic, TABLE_PARAMS,
                                                  params.table.warped.width, true));

  image_transport::CameraSubscriber camera_sub =
    it.subscribeCamera(params.camera_base_topic, 1, bind(camera_callback, node, _1, _2));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

void camera_callback(rclcpp::Node::SharedPtr node,
                     const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg)
{
  // Convert the image to an OpenCV image.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Extract camera info into OpenCV format.
  string camera_frame = camera_info_msg->header.frame_id;
  cv::Mat camera_matrix(3, 3, CV_64F, (void*)camera_info_msg->k.data());
  cv::Mat dist_coeffs(1, 5, CV_64F, (void*)camera_info_msg->d.data());

  // Find the Aruco markers in the image.
  static const auto aruco_detector_params = [] {
    auto params = cv::aruco::DetectorParameters();
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    return params;
  }();
  static const auto aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  static const cv::aruco::ArucoDetector aruco_detector(aruco_dictionary, aruco_detector_params);
  vector<int> aruco_ids;
  vector<vector<cv::Point2f>> aruco_corners, rejected;
  aruco_detector.detectMarkers(cv_ptr->image, aruco_corners, aruco_ids, rejected);

  // Process the image with each object manager.
  for (auto& object_manager : object_managers) {
    object_manager.refine_markers(aruco_detector, cv_ptr->image, aruco_ids, aruco_corners, rejected,
                                  camera_matrix, dist_coeffs);
    object_manager.process(cv_ptr->image, aruco_ids, aruco_corners, camera_frame, camera_matrix,
                           dist_coeffs);
  }
}