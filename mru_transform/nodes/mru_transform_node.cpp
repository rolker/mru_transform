#include "mru_transform.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("mru_transform"));
  mru_transform::MRUTransform mru_transform(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
