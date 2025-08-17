#ifndef TERRAIN_LAYER_H_
#define TERRAIN_LAYER_H_

#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mutex>
#include <memory>

namespace terra_sense
{

class TerrainLayer : public rclcpp::Node
{
public:
  TerrainLayer();
  virtual ~TerrainLayer() = default;

  void initialize();
  void updateGridMap();

private:
  // Grid map
  std::unique_ptr<grid_map::GridMap> terrain_map_;
  std::string map_frame_id_;
  std::string robot_frame_id_;
  
  // Grid map parameters
  double map_resolution_;  // meters per pixel
  double map_length_x_;    // map size in x direction (meters)
  double map_length_y_;    // map size in y direction (meters)
  
  // Terrain classification
  void terrainCallback(const std_msgs::msg::String::SharedPtr msg);
  void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishGridMap();
  void updateTerrainCosts(const geometry_msgs::msg::Pose& robot_pose);
  
  // Cost mapping
  float getTerrainCost(const std::string& terrain_class);
  
  // ROS2 interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr terrain_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cost_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // State
  std::string current_terrain_;
  geometry_msgs::msg::Pose current_robot_pose_;
  bool terrain_updated_;
  bool pose_updated_;
  std::mutex data_mutex_;
  
  // Cost values
  float smooth_cost_;
  float rough_cost_;
  float obstacle_cost_;
  float unknown_cost_;
  
  // Update region parameters
  double update_radius_;  // radius around robot to update costs
  
  // Layer names
  const std::string TERRAIN_LAYER = "terrain";
  const std::string COST_LAYER = "traversability_cost";
  const std::string ELEVATION_LAYER = "elevation";  // optional, for compatibility
};

}  // namespace terra_sense

#endif  // TERRAIN_LAYER_H_
