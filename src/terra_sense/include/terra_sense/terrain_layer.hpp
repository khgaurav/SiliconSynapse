// Copyright (c) 2025
// Simple standalone TerrainLayer-like helper for publishing grid_map from terrain classes

#ifndef TERRAIN_LAYER_H_
#define TERRAIN_LAYER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

namespace terra_sense
{

class TerrainLayer
{
public:
  TerrainLayer();
  virtual ~TerrainLayer() = default;

  // Initialize with a node (creates subscriptions/publishers)
  void initialize(rclcpp::Node::SharedPtr node);

private:
  void updateCostsFromInput();
  void terrainCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr terrain_subscriber_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr cost_publisher_;
  std::string terrain_class_;
  unsigned char terrain_cost_;
};

}  // namespace terra_sense

#endif  // TERRAIN_LAYER_H_
