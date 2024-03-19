#include <rclcpp/rclcpp.hpp>

#include "aruco_transforms/aruco_transform_node.hpp"
#include "aruco_transforms/config.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ArucoDefinedObject chessboard = aruco_chessboard();
  ArucoTransformNode node(&chessboard);
  rclcpp::spin(node.get_node());
  rclcpp::shutdown();
  return 0;
}
