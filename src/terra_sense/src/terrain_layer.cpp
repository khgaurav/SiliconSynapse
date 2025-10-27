#include <terra_sense/terrain_layer.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_costmap_2d/costmap_2d_converter.hpp>

using grid_map::GridMap;

namespace terra_sense
{

TerrainLayer::TerrainLayer()
: terrain_cost_(0)
{}

void TerrainLayer::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  terrain_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    "/terrain_class", 10, std::bind(&TerrainLayer::terrainCallback, this, std::placeholders::_1));
  cost_publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("/terrasense_grid_map", 2);
}

void TerrainLayer::updateCostsFromInput()
{
  // Since we no longer have nav2 master costmap, simply publish an empty grid_map with the terrain cost as a property
  try {
    grid_map::GridMap gridMap;
    gridMap.setFrameId("map");
    gridMap.setTimestamp(rclcpp::Clock().now().nanoseconds());
    // create a very small map placeholder
    gridMap.setGeometry(grid_map::Length(0.1, 0.1), 0.05);
  gridMap.add("cost", 0.0);
  // set single cell value to terrain_cost_
  grid_map::Index idx(0, 0);
  gridMap.at("cost", idx) = static_cast<float>(terrain_cost_);

    auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
    cost_publisher_->publish(*msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to publish grid_map: %s", e.what());
  }
}

void TerrainLayer::terrainCallback(const std_msgs::msg::String::SharedPtr msg)
{

  terrain_class_ = msg->data;
  if (terrain_class_ == "1" || terrain_class_ == "4") {
    terrain_cost_ = 0;
  } else if (terrain_class_ == "2" || terrain_class_ == "3" || terrain_class_ == "5") {
    terrain_cost_ = 5;
  } else if (terrain_class_ == "6") {
    terrain_cost_ = 254;
  } else {
    terrain_cost_ = 255;
  }

  updateCostsFromInput();
}

}  // namespace terra_sense
