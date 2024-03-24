#ifndef ARUCO_TRANSFORM_NODE__ARUCO_TRANSFORMS_HPP_
#define ARUCO_TRANSFORM_NODE__ARUCO_TRANSFORMS_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "aruco_transforms/aruco_object.hpp"
#include "aruco_transforms_params.hpp"
#include "cv_bridge/cv_bridge.h"

class ArucoTransformNode
{
public:
  /**
   * Construct a new Aruco Transform Node object.
   *
   * @param[in] aruco_object The Aruco object to be used for the transform.
   * @param[in] options The node options.
   */
  explicit ArucoTransformNode(ArucoDefinedObject* chessboard_aruco_object,
                              ArucoDefinedObject* table_aruco_object,
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * Get the raw node.
   *
   * @return The raw node.
   */
  rclcpp::Node::SharedPtr get_node() const;

  /**
   * Get the node's logger.
   *
   * @return The node's logger.
   */
  rclcpp::Logger get_logger() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<aruco_transforms_params::ParamListener> param_listener_;
  std::unique_ptr<aruco_transforms_params::Params> params_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  std::unique_ptr<image_transport::CameraSubscriber> camera_sub_;

  std::unique_ptr<image_transport::Publisher> chessboard_warped_pub_;
  ArucoDefinedObject* chessboard_aruco_object_;

  std::unique_ptr<image_transport::Publisher> table_warped_pub_;
  ArucoDefinedObject* table_aruco_object_;

  /**
   * Callback to be called when a new image is received. This will identify the objects in the
   * image and publish the appropriate transforms and images.
   *
   * @param image The image that was received.
   * @param cinfo The camera info that was received.
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo);
};

#endif