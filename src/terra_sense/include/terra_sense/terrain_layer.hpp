#ifndef TERRA_SENSE__TERRAIN_LAYER_HPP_
#define TERRA_SENSE__TERRAIN_LAYER_HPP_

#include <string>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

namespace terra_sense
{

class TerrainLayer : public rclcpp::Node
{
public:
  TerrainLayer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void updateAndPublishMap();

private:
  void terrainCallback(const std_msgs::msg::String::SharedPtr msg);

  // Grid map
  grid_map::GridMap grid_map_;
  
  // Subscriptions and publishers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr terrain_subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cost_publisher_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr update_timer_;
  
  // Parameters and state variables
  bool enabled_;
  std::string map_frame_id_;
  double map_resolution_;
  double map_width_;
  double map_height_;
  
  // Terrain state
  std::string terrain_;
};

}  // namespace terra_sense

#endif  // TERRA_SENSE__TERRAIN_LAYER_HPP_
