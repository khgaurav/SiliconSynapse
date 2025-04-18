#include <terra_sense/terrain_layer.hpp>

namespace terra_sense
{

TerrainLayer::TerrainLayer(const rclcpp::NodeOptions & options)
: rclcpp::Node("terrain_layer", options)
{
  // Initialize parameters
  declare_parameter("enabled", rclcpp::ParameterValue(true));
  get_parameter("enabled", enabled_);
  
  declare_parameter("map_frame_id", "base_link");
  get_parameter("map_frame_id", map_frame_id_);
  
  declare_parameter("map_resolution", 0.05);
  get_parameter("map_resolution", map_resolution_);
  
  declare_parameter("map_width", 10.0);
  get_parameter("map_width", map_width_);
  
  declare_parameter("map_height", 10.0);
  get_parameter("map_height", map_height_);
  
  // Initialize grid map with default layers
  grid_map_.setFrameId(map_frame_id_);
  grid_map_.setGeometry(grid_map::Length(map_width_, map_height_), map_resolution_);
  grid_map_.add("terrain_cost");
  
  // Initialize with default cost (unknown terrain)
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    grid_map_.at("terrain_cost", *it) = 255.0;  // NO_INFORMATION equivalent
  }
  
  // Create subscriptions and publishers
  terrain_subscription_ = create_subscription<std_msgs::msg::String>(
    "/terrain_class", 10, 
    std::bind(&TerrainLayer::terrainCallback, this, std::placeholders::_1));
  
  grid_map_publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(
    "/terrain_grid_map", 10);
  
  cost_publisher_ = create_publisher<std_msgs::msg::String>("/cost_changes", 40);
  
  // Create a timer for periodic updates
  update_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TerrainLayer::updateAndPublishMap, this));
  
  RCLCPP_INFO(get_logger(), "TerrainLayer initialized");
}

void TerrainLayer::updateAndPublishMap()
{
  if (!enabled_) {
    return;
  }
  
  // Update the timestamp
  grid_map_.setTimestamp(now().nanoseconds());
  
  // Convert to message and publish
  auto message = grid_map::GridMapRosConverter::toMessage(grid_map_);
  grid_map_publisher_->publish(*message);
}

void TerrainLayer::terrainCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "Data %s", msg->data.c_str());
  if(msg->data == terrain_)
    return;
    
  terrain_ = msg->data;
  float terrain_cost;
  
  // Define terrain costs (equivalent to original logic but without nav2 constants)
  if (terrain_ == "1" || terrain_ == "4") {
    terrain_cost = 0.0;  // FREE_SPACE
  } else if (terrain_ == "2" || terrain_ == "3" || terrain_ == "5") {
    terrain_cost = 5.0;  // Low cost
  } else if (terrain_ == "6") {
    terrain_cost = 254.0;  // LETHAL_OBSTACLE
  } else {
    terrain_cost = 255.0;  // NO_INFORMATION
  }
  
  // Update the entire grid map with the new terrain cost
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    const grid_map::Index index = *it;
    grid_map_.at("terrain_cost", index) = terrain_cost;
  }
  
  // Publish cost change information
  std_msgs::msg::String cost_msg;
  std::stringstream ss;
  ss << "terrain: " << terrain_ 
     << ", new cost: " << terrain_cost;
  cost_msg.data = ss.str();
  cost_publisher_->publish(cost_msg);
  
  RCLCPP_INFO(get_logger(), "Updated terrain cost to %.1f for terrain type %s", 
              terrain_cost, terrain_.c_str());
}

}  // namespace terra_sense

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terra_sense::TerrainLayer)
