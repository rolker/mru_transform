#include "mru_transform/mru_transform.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  try
  {
    // Both node construction and invalid configuration (e.g. a half-set or
    // out-of-range map origin, thrown from the MRUTransform constructor) end
    // up here: exit with a reason instead of letting the exception reach
    // std::terminate.
    node = std::make_shared<rclcpp::Node>("mru_transform");
    mru_transform::MRUTransform mru_transform(node);
    // Single-threaded spin: callbacks are serialized (see the threading note
    // in mru_transform.hpp).
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    // The node does not exist if its own construction failed.
    auto logger = node ? node->get_logger() : rclcpp::get_logger("mru_transform");
    RCLCPP_FATAL(logger, "mru_transform failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
