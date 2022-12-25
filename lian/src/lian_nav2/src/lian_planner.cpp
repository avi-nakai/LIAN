#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "lian_planner.hpp"
#include "mission.h"
#include "map.h"
#include "config.h"

namespace nav2_lian_planner
{
void LianPlaner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<mission::Mission>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
}

nav_msgs::msg::Path LianPlaner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  searchParams[CNS_TAG_SX] = start.pose.position.x
  searchParams[CNS_TAG_SY] = start.pose.position.y
  searchParams[CNS_TAG_FX] = goal.pose.position.x
  searchParams[CNS_TAG_FY] = goal.pose.position.y

  nav_msgs::msg::Path lian_path;
  auto start_time = std::chrono::steady_clock::now();

  if (sr.pathfound) {
    lian_path.header.stamp = node_->now();
    lian_path.header.frame_id = map;

    for (int node=0; node<len(sr.hppath); node++){
      geometry_msgs::msg::PoseStamped posestamped;

      tf2::Quaternion nodeQuaternion;
      nodeQuaternion.setRPY(0,0,sr.hppath[node].angle);

      posestamped.pose.position.x = sr.hppath[node].i;
      posestamped.pose.position.y = sr.hppath[node].j;
      posestamped.pose.orientation.z = nodeQuaternion.getZ();
      posestamped.pose.orientation.w = nodeQuaternion.getW();
      posestamped.header.stamp = node_->now();
      posestamped.header.frame_id = map;

      lian_path.push_back(posestamped);
    }     
  }
  return(lian_path)
} 

} //namespace nav2_lian_planner