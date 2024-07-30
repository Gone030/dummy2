#ifndef ACKERMANN_GAZEBO_PLUGIN__ACKERMANN_GAZEBO_PLUGIN_HPP_
#define ACKERMANN_GAZEBO_PLUGIN__ACKERMANN_GAZEBO_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

namespace ackermann_gazebo_plugin{
    #define WHEELBASE_WIDTH 0.25
    #define WHEEL_DIAMETRY 0.112
    #define WHEELBASE_LENGTH 0.61
    #define MAX_STEER_ANGLE M_PI/6.0

    class AckermannGazeboPlugin : public gazebo::ModelPlugin{
        public:
            AckermannGazeboPlugin();

            void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
        private:
            void update();

            std::string robot_namespace_;

            gazebo::physics::JointControllerPtr jc;

            gazebo::physics::ModelPtr model_;
            gazebo::physics::WorldPtr world_;

            gazebo::physics::JointPtr fl_str_joint;
            gazebo::physics::JointPtr fr_str_joint;
            gazebo::physics::JointPtr fl_wheel_joint;
            gazebo::physics::JointPtr fr_wheel_joint;

            gazebo::physics::JointPtr fl_shock_joint;
            gazebo::physics::JointPtr fr_shock_joint;
            gazebo::physics::JointPtr bl_shock_joint;
            gazebo::physics::JointPtr br_shock_joint;

            gazebo::physics::JointPtr bl_wheel_joint;
            gazebo::physics::JointPtr br_wheel_joint;

            gazebo::common::PID fl_pid, fr_pid, bl_pid, br_pid;
            gazebo::common::PID fl_shock_pid, fr_shock_pid, bl_shock_pid, br_shock_pid;

            rclcpp::Node::SharedPtr ros_node_;
            std::map<std::string, std::pair<gazebo::physics::JointPtr, gazebo::common::PID>> joints_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;


            gazebo::common::Time last_sim_time_;
            gazebo::common::Time last_update_time_;
            double update_period_ms_;

            gazebo::event::ConnectionPtr world_connection_callback_;

            double getvelo(double linear){
                double meters_per_rev = M_PI * WHEEL_DIAMETRY;
                double revs_per_sec = linear / meters_per_rev;
                double rads_per_sec = 2 * M_PI * revs_per_sec;
                return rads_per_sec;
            }

            gazebo::physics::JointPtr get_joint(const char *joint_name){
                auto joint = model_ ->GetJoint(joint_name);
                if(joint.get() == 0){
                    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to get joint %s",
                    joint_name);
                }
                return joint;
            }
            void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg){
                // RCLCPP_INFO(ros_node_->get_logger(), "get twist velue");
                if(msg->linear.x == 0){ //정지
                    jc->SetPositionTarget(fl_str_joint->GetScopedName(),
                                            0.0);
                    jc->SetPositionTarget(fr_str_joint->GetScopedName(),
                                            0.0);
                    jc->SetVelocityTarget(bl_wheel_joint->GetScopedName(),
                                            0.0);
                    jc->SetVelocityTarget(br_wheel_joint->GetScopedName(),
                                            0.0);

                } else if(msg->angular.z == 0) { // 직진
                    jc->SetPositionTarget(fr_str_joint->GetScopedName(),
                                            0.0);
                    jc->SetPositionTarget(fl_str_joint->GetScopedName(),
                                            0.0);
                    jc->SetVelocityTarget(bl_wheel_joint->GetScopedName(),
                                            getvelo(msg->linear.x));
                    jc->SetVelocityTarget(br_wheel_joint->GetScopedName(),
                                            getvelo(msg->linear.x));
                } else {
                    double curvature = msg->angular.z / msg->linear.x * WHEELBASE_LENGTH;
                    double steer_angle = atan(curvature);
                    jc->SetPositionTarget(fr_str_joint->GetScopedName(),
                                            steer_angle);
                    jc->SetPositionTarget(fl_str_joint->GetScopedName(),
                                            steer_angle);
                    double radius = 1. / curvature;
                    double L_radius = radius - WHEELBASE_WIDTH / 2.;
                    double R_radius = radius + WHEELBASE_WIDTH / 2.;

                    jc->SetVelocityTarget(br_wheel_joint->GetScopedName(),
                                            getvelo(msg->linear.x) * R_radius / radius);
                    jc->SetVelocityTarget(bl_wheel_joint->GetScopedName(),
                                            getvelo(msg->linear.x) * L_radius / radius);
                }
            }
    };
}

#endif
