#include "mru_transform/nodes/sea_surface_estimator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SeaSurfaceEstimator>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
