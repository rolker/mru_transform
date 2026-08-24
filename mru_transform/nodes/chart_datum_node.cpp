#include "mru_transform/nodes/chart_datum_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChartDatumNode>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
