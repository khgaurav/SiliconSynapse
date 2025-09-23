#include <rclcpp/rclcpp.hpp>
#include "terra_sense/terrain_layer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("terra_sense_node");

  auto terrain = std::make_shared<terra_sense::TerrainLayer>();
  terrain->initialize(node);

  RCLCPP_INFO(node->get_logger(), "terra_sense node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
