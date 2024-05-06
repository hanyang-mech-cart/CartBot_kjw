#include "docking_action.hpp"

DockingAction::DockingAction(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  done_flag_ = false;
}

DockingAction::~DockingAction()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "SHUTTING DOWN QR NODE");
}


BT::NodeStatus DockingAction::onStart()
{
    const std::string sub_topic_name = "/aruco_poses";
    const std::string pub_topic_name = "/cmd_vel";
    const std::string TTS_service_name = "???";

    sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseArray>(
            sub_topic_name,
            rclcpp::QoS(1),
            std::bind(&docking_action_callback, this, std::placeholders::_1));

    cmd_vel_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(pub_topic_name, rclcpp::QoS(1));


    // action 추가해야함
    // TTS_client_ = node_->create_client<yacyac_interface::srv::TTS>(TTS_service_name);


    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockingAction::onRunning()
{
    rclcpp::spin_some(node_ptr_);

    double angular_z = kp * error + ki * integral + kd * derivative;
    geometry_msgs::msg::Twist cmd_vel_msg_;

    if (pose_z < 0.7 && !ok && !again){
        RCLCPP_INFO(node_ptr_->get_logger(), "##################################poze_z < 0.7##################################");
        
        if (pose_z < 0.7 && error < 0.04 && angle_degrees < 10.0){
            cnt_near++;

            if (angle_degrees < 10.0 && cnt_near >= 5){
                RCLCPP_INFO(node_ptr_->get_logger(), "////////////////////Wait////////////////////");
                std::this_thread::sleep_for(std::chrono::seconds(2));

                cmd_vel_msg_.linear.x = -0.3;
                cmd_vel_msg_.angular.z = 0.0;
                RCLCPP_INFO(node_ptr_->get_logger(), "&&&&&&&&&&Docking&&&&&&&&&&");

                while (cnt_time < 200000){
                    cnt_time++;
                    cmd_vel_publisher_->publish(cmd_vel_msg_);
                }

                cmd_vel_msg_.linear.x = 0.0;
                cmd_vel_publisher_->publish(cmd_vel_msg_);
                ok = true;
                return BT::NodeStatus::SUCCESS;
            }
        }

        else if (pose_z < 0.7 && (error >= 0.04 && angle_degrees >= 10.0) && ccnt < 30){
            cmd_vel_msg_.linear.x = 0.0;
            cmd_vel_msg_.angular.z = angular_z;
            ccnt++;
            RCLCPP_INFO(node_ptr_->get_logger(), "Turn1");
        }

        else{
            ccnt = 0;
            prev_error = 0.0;
            integral = 0.0;
            again = true;
        }

    }

    if (again){
        RCLCPP_INFO(node_ptr_->get_logger(), "@@@@@@@@@@Again@@@@@@@@@@");       
        if (pose_z < 0.7){
                    cmd_vel_msg_.linear.x = 0.2;
                    cmd_vel_msg_.linear.x = 0.1;
        }

        else if (angle_degrees >= 4.0 && prev_cmd_angular_z.back() >= 0.1){
            if (angle_degrees - (std::accumulate(prev_angle_degrees.begin(), prev_angle_degrees.end(), 0.0) / prev_angle_degrees.size()) <= 0){
                cnt1++;
                if (cnt1 > 10){
                    cmd_vel_msg_.angular.z = (std::accumulate(prev_angle_degrees.begin(), prev_angle_degrees.end(), 0.0) / std::abs(std::accumulate(prev_angle_degrees.begin(), prev_angle_degrees.end(), 0.0))) * 0.1;
                    cnt1 = 0;
                    cnt2 = 0;
                }
            }
            
            else{
                cnt2++;
                if (cnt2 > 10){
                    cnt1 = 0;
                    cnt2 = 0;
                    cmd_vel_msg_.angular.z = -(std::accumulate(prev_angle_degrees.begin(), prev_angle_degrees.end(), 0.0) / std::abs(std::accumulate(prev_angle_degrees.begin(), prev_angle_degrees.end(), 0.0))) * 0.1;
                }
            }

            cmd_vel_msg_.linear.x = 0.0;
            RCLCPP_INFO(node_ptr_->get_logger(), "Turn2");
        }

        else if ((pose_z <= 1.2 && error < 0.04 && angle_degrees < 10.0) || pose_z > 1.2){
            again = false;
            first = true;
            prev_error = 0.0;
            integral = 0.0;
            cnt_near = 0;
            RCLCPP_INFO(node_ptr_->get_logger(), "##########GOGO##########");
        }

        else{
            cmd_vel_msg_.linear.x = 0.2;
            cmd_vel_msg_.angular.z = angular_z;
        }
    }    

    if (cmd_vel_msg_.angular.z > max_angular_z){
        cmd_vel_msg_.angular.z = max_angular_z;
    }

    else if (cmd_vel_msg_.angular.z < -max_angular_z){
        cmd_vel_msg_.angular.z = -max_angular_z;
    }

    if (std::abs(cmd_vel_msg_.angular.z) < min_angular_z){
        cmd_vel_msg_.angular.z = cmd_vel_msg_.angular.z > 0 ? min_angular_z : -min_angular_z;
    }

    cmd_vel_publisher_->publish(cmd_vel_msg_);
    RCLCPP_INFO(node_ptr_->get_logger(), "Cmd angular z: %f", cmd_vel_msg_.angular.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Distance: %f", pose_z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Error: %f", error);

    prev_error = error;
    prev_cmd_angular_z.push_back(cmd_vel_msg_.angular.z);
    prev_angle_degrees.push_back(angle_degrees);
        
    return BT::NodeStatus::RUNNING;
}

void DockingAction::docking_action_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    pose_x = msg->poses[0].position.x;
    pose_y = msg->poses[0].position.y;
    pose_z = msg->poses[0].position.z;
    pose_qx = msg->poses[0].orientation.x;
    pose_qy = msg->poses[0].orientation.x;
    pose_qz = msg->poses[0].orientation.x;
    pose_qw = msg->poses[0].orientation.x;

    Eigen::Matrix3d rotation_matrix = quatToRotationMatrix(pose_qx, pose_qy, pose_qz, pose_qw);

    // 변환 행렬을 사용하여 객체의 방향 계산
    Eigen::Vector3d object_direction = rotation_matrix.col(0); // 첫 번째 열을 가져와서 객체 방향으로 사용
    Eigen::Vector3d my_direction(1.0, 0.0, 0.0); // 내 방향

    // 두 벡터 간의 각도 계산
    double angle_radians = std::acos(object_direction.dot(my_direction));
    double angle_degrees = angle_radians * 180.0 / M_PI; // 라디안을 도로 변환

    error = -0.03 - pose_x;
    integral += error;
    derivative = error - prev_error;
}

Eigen::Matrix3d quatToRotationMatrix(double qx, double qy, double qz, double qw) {
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
    return rotation_matrix;
}