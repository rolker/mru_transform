#include "mru_transform/mru_transform.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("mru_transform"));
  try
  {
    // Invalid configuration (e.g. a half-set or out-of-range map origin) throws
    // from the constructor. Exit cleanly with a reason instead of letting the
    // exception reach std::terminate.
    mru_transform::MRUTransform mru_transform(node);
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(node->get_logger(), "mru_transform failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
