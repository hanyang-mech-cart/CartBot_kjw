#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>


class DockingAction : public BT::StatefulActionNode
{
public:
    DockingAction(const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    rclcpp_action::Client<GoalHandleNav>::SharedPtr action_client_ptr_;
    bool done_flag_;    
    
    double pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw;

    double angle_degrees;
    
    double error, integral, derivative, prev_error;

    bool ok, again, first;
    int cnt_near, cnt_time, cnt1, cnt2, ccnt, cnt_num;
    double prev_error, integral, max_angular_z, min_angular_z;
    std::vector<float> prev_cmd_angular_z, prev_angle_degrees;

    double kp = 2;
    double ki = 0.6;
    double kd = 80;
    
    // Method overrides
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

    // Action Client callback
    void docking_action_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
};

Eigen::Matrix3d quatToRotationMatrix(double qx, double qy, double qz, double qw);