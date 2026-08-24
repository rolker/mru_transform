#include "mru_transform/nodes/tide_copier.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TideCopier>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
