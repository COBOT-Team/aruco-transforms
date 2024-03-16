#include <rclcpp/rclcpp.hpp>

#include "aruco_transforms/aruco_transform_node.hpp"
#include "aruco_transforms/config.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ArucoTransformNode node(&ARUCO_CHESSBOARD);
  rclcpp::spin(node.get_node());
  rclcpp::shutdown();
  return 0;
}
