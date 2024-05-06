#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_behaviors.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class AutonomyNode : public rclcpp::Node
{
public:
  explicit AutonomyNode(const std::string &node_name);
  void setup();
  void create_behavior_tree();
  void update_behavior_tree();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  BT::Tree tree_;
};