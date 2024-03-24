#include "aruco_transforms/aruco_transform_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace aruco_transforms_params;
using std::placeholders::_1;
using std::placeholders::_2;

ArucoTransformNode::ArucoTransformNode(ArucoDefinedObject* chessboard_aruco_object,
                                       ArucoDefinedObject* table_aruco_object,
                                       const rclcpp::NodeOptions& options)
  : chessboard_aruco_object_(chessboard_aruco_object), table_aruco_object_(table_aruco_object)
{
  node_ = make_shared<rclcpp::Node>("aruco_transforms", options);
  param_listener_ = make_unique<ParamListener>(node_);
  params_ = make_unique<Params>(param_listener_->get_params());

  // Initialize the tf broadcaster.
  tf_broadcaster_ = make_unique<tf2_ros::TransformBroadcaster>(node_);

  // Initialize the image transport and all publishers.
  it_ = make_unique<image_transport::ImageTransport>(node_);
  chessboard_warped_pub_ =
    make_unique<image_transport::Publisher>(it_->advertise(params_->warped_chessboard_topic, 1));
  table_warped_pub_ =
    make_unique<image_transport::Publisher>(it_->advertise(params_->warped_table_topic, 1));

  // Initialize the camera subscriber.
  auto bound_callback = bind(&ArucoTransformNode::image_callback, this, _1, _2);
  camera_sub_ = make_unique<image_transport::CameraSubscriber>(
    it_->subscribeCamera(params_->camera_base_topic, 1, bound_callback));

  RCLCPP_INFO(get_logger(), "Aruco transform node started");
}

rclcpp::Node::SharedPtr ArucoTransformNode::get_node() const
{
  return node_;
}

rclcpp::Logger ArucoTransformNode::get_logger() const
{
  return node_->get_logger();
}

void ArucoTransformNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo)
{
  auto now = rclcpp::Clock().now();

  // Convert the image to an OpenCV image.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Extract camera info into OpenCV format.
  string camera_frame = cinfo->header.frame_id;
  cv::Mat camera_matrix(3, 3, CV_64F, (void*)cinfo->k.data());
  cv::Mat dist_coeffs(1, 5, CV_64F, (void*)cinfo->d.data());

  // Find the Aruco markers in the image.
  static const auto aruco_detector_params = cv::aruco::DetectorParameters();
  static const auto aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  static const cv::aruco::ArucoDetector aruco_detector(aruco_dictionary, aruco_detector_params);
  vector<int> aruco_ids;
  vector<vector<cv::Point2f>> aruco_corners;
  aruco_detector.detectMarkers(cv_ptr->image, aruco_corners, aruco_ids);

  vector<geometry_msgs::msg::TransformStamped> transforms;

  // Solve the chessboard transform.
  tf2::Transform chessboard_transform;
  bool chessboard_tf_success = chessboard_aruco_object_->solve_transform(
    aruco_ids, aruco_corners, camera_matrix, dist_coeffs, chessboard_transform, false);
  if (chessboard_tf_success) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = now;
    tf_stamped.header.frame_id = camera_frame;
    tf_stamped.child_frame_id = params_->chessboard_frame;
    tf_stamped.transform = tf2::toMsg(chessboard_transform);
    transforms.emplace_back(tf_stamped);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to solve chessboard transform.");
  }

  // Warp the chessboard image.
  cv::Mat warped_chessboard;
  bool chessboard_warp_success = chessboard_aruco_object_->warp_perspective(
    cv_ptr->image, warped_chessboard, aruco_ids, aruco_corners,
    cv::Size(params_->warped_chessboard_size, params_->warped_chessboard_size));
  if (chessboard_warp_success) {
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = camera_frame;
    sensor_msgs::msg::Image::SharedPtr warped_chessboard_msg =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, warped_chessboard)
        .toImageMsg();
    chessboard_warped_pub_->publish(warped_chessboard_msg);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to warp chessboard image.");
  }

  // Solve the table transform.
  tf2::Transform table_transform;
  bool table_tf_success = table_aruco_object_->solve_transform(
    aruco_ids, aruco_corners, camera_matrix, dist_coeffs, table_transform, true);
  if (table_tf_success) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = now;
    tf_stamped.header.frame_id = params_->table_frame;
    tf_stamped.child_frame_id = camera_frame;
    tf_stamped.transform = tf2::toMsg(table_transform);
    transforms.emplace_back(tf_stamped);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to solve table transform.");
  }

  // Warp the table image.
  cv::Mat warped_table;
  bool table_warp_success = table_aruco_object_->warp_perspective(
    cv_ptr->image, warped_table, aruco_ids, aruco_corners,
    cv::Size(params_->warped_table_width,
             params_->warped_table_width / table_aruco_object_->get_warp_aspect_ratio()));
  if (table_warp_success) {
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = camera_frame;
    sensor_msgs::msg::Image::SharedPtr warped_table_msg =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, warped_table).toImageMsg();
    table_warped_pub_->publish(warped_table_msg);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to warp table image.");
  }

  // Publish transforms.
  if (!transforms.empty()) tf_broadcaster_->sendTransform(transforms);
}