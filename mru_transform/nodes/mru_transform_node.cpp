#include "mru_transform.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("mru_transform"));
  mru_transform::MRUTransform mru_transform(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <mru_transform/mru_transform.h>

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "mru_transform");

//   ros::NodeHandle nh;
//   ros::NodeHandle nh_private("~");
  
//   mru_transform::MRUTransform t(nh, nh_private);
  
//   ros::spin();
//   return 0;
// }
