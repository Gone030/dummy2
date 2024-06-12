#include "ackermann_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/Time.hh>

namespace ackermann_gazebo_plugin{

    AckermannGazeboPlugin::AckermannGazeboPlugin()
        : robot_namespace_{""},
          last_sim_time_{0},
          last_update_time_{0},
          update_period_ms_{5} {}

    void AckermannGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf){
        model_ = model;
        world_ = model_->GetWorld();
        std::cout << "model name : " << model_->GetName() << std::endl;

        if(sdf->HasElement("dummy2")){
            robot_namespace_ = sdf->GetElement("dummy2")->Get<std::string>() + "/";
        }
        ros_node_ = gazebo_ros::Node::Get(sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "loading plugin");

        auto all_joints = model_->GetJoints();
        for(auto const& j : all_joints){
            if(j->GetType() == gazebo::physics::Joint::FIXED_JOINT){
                continue;
            }

            auto pid = gazebo::common::PID{};
            pid.SetPGain(200.0);
            pid.SetIGain(0.0);
            pid.SetDGain(0.0);

            auto const& name = j->GetName();
            joints_[name] = std::make_pair(j, pid);
        }

        for(auto const& j : joints_){
            RCLCPP_DEBUG(ros_node_-> get_logger(), j.first.c_str());
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*ros_node_);

        joint_state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());

        jc = model_->GetJointController();

        fl_pid = gazebo::common::PID(1, 0, 0);
        fl_str_joint = get_joint("front_left_wheel_steer_joint");
        jc->SetPositionPID(fl_str_joint->GetScopedName(), fl_pid);

        fr_pid = gazebo::common::PID(1, 0, 0);
        fr_str_joint = get_joint("front_right_wheel_steer_joint");
        jc->SetPositionPID(fr_str_joint->GetScopedName(), fr_pid);

        fl_shock_pid = gazebo::common::PID(500, 0, 60);
        fl_shock_joint = get_joint("front_left_shock_joint");
        jc->SetPositionPID(fl_shock_joint->GetScopedName(), fl_shock_pid);
        jc->SetPositionTarget(fl_shock_joint->GetScopedName(), 0.0);

        fr_shock_pid = gazebo::common::PID(500, 0, 60);
        fr_shock_joint = get_joint("front_right_shock_joint");
        jc->SetPositionPID(fr_shock_joint->GetScopedName(), fr_shock_pid);
        jc->SetPositionTarget(fr_shock_joint->GetScopedName(), 0.0);

        bl_shock_pid = gazebo::common::PID(500, 0, 60);
        bl_shock_joint = get_joint("back_left_shock_joint");
        jc->SetPositionPID(bl_shock_joint->GetScopedName(), bl_shock_pid);
        jc->SetPositionTarget(bl_shock_joint->GetScopedName(), 0.0);

        br_shock_pid = gazebo::common::PID(500, 0, 60);
        br_shock_joint = get_joint("back_right_shock_joint");
        jc->SetPositionPID(br_shock_joint->GetScopedName(), bl_shock_pid);
        jc->SetPositionTarget(br_shock_joint->GetScopedName(), 0.0);

        fl_wheel_joint = get_joint("front_left_wheel_joint");
        fr_wheel_joint = get_joint("front_right_wheel_joint");

        bl_pid = gazebo::common::PID(0.1, 0.01, 0);
        bl_wheel_joint = get_joint("back_left_wheel_joint");
        jc->SetVelocityPID(bl_wheel_joint->GetScopedName(), bl_pid);

        br_pid = gazebo::common::PID(0.1, 0.01, 0);
        br_wheel_joint = get_joint("back_right_wheel_joint");
        jc->SetVelocityPID(br_wheel_joint->GetScopedName(), br_pid);

        cmd_vel_sub = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 2, std::bind(&AckermannGazeboPlugin::twist_callback, this, std::placeholders::_1)
        );

        odom_pub = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/" + model_->GetName() + "/odom", rclcpp::SensorDataQoS());
        pose_pub = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/" + model_->GetName() + "/odom", rclcpp::SensorDataQoS());

        world_connection_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermannGazeboPlugin::update, this)
        );
    }

    void AckermannGazeboPlugin::update(){
        auto cur_time = world_->SimTime();
        if(last_sim_time_ == 0){
            last_sim_time_ = cur_time;
            last_update_time_ = cur_time;
            return;
        }

        auto update_dt = (cur_time - last_update_time_).Double();
        if(update_dt * 1000 >= update_period_ms_){
            auto pose = model_->WorldPose();

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = ros_node_->now();
            pose_msg.header.frame_id = "world";
            pose_msg.pose.position.x = pose.X();
            pose_msg.pose.position.y = pose.Y();
            pose_msg.pose.position.z = pose.Z();
            pose_msg.pose.orientation.x = pose.Rot().X();
            pose_msg.pose.orientation.y = pose.Rot().Y();
            pose_msg.pose.orientation.z = pose.Rot().Z();
            pose_msg.pose.orientation.w = pose.Rot().W();
            pose_pub->publish(pose_msg);

            nav_msgs::msg::Odometry odom_msg;
            auto linear_vel = model_->WorldLinearVel();
            odom_msg.header.stamp = ros_node_->now();
            odom_msg.header.frame_id = "world";
            odom_msg.pose.pose.position.x = pose.X();
            odom_msg.pose.pose.position.y = pose.Y();
            odom_msg.pose.pose.position.z = pose.Z();
            odom_msg.pose.pose.orientation.x = pose.Rot().X();
            odom_msg.pose.pose.orientation.y = pose.Rot().Y();
            odom_msg.pose.pose.orientation.z = pose.Rot().Z();
            odom_msg.pose.pose.orientation.w = pose.Rot().W();
            odom_msg.twist.twist.linear.x = linear_vel.X();
            odom_msg.twist.twist.linear.y = linear_vel.Y();
            odom_msg.twist.twist.linear.z = linear_vel.Z();
            odom_pub->publish(odom_msg);

            rclcpp::Time now = ros_node_->now();
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.stamp.sec = cur_time.sec;
            t.header.stamp.nanosec = cur_time.nsec;

            t.header.frame_id = "odom";
            t.child_frame_id = "base_footprint";

            t.transform.translation.x = pose.X();
            t.transform.translation.y = pose.Y();
            t.transform.translation.z = pose.Z(); // maybe 0

            t.transform.rotation.x = pose.Rot().X();
            t.transform.rotation.y = pose.Rot().Y();
            t.transform.rotation.z = pose.Rot().Z();
            t.transform.rotation.w = pose.Rot().W();
            tf_broadcaster_->sendTransform(t);

            auto msg = sensor_msgs::msg::JointState{};
            msg.header.stamp.sec = cur_time.sec;
            msg.header.stamp.nanosec = cur_time.nsec;

            for(auto& j : joints_){
                auto const& name = j.first;
                auto& joint = j.second.first;
                auto position = joint->Position();
                msg.name.push_back(name);
                msg.position.push_back(position);
            }
            joint_state_pub_->publish(msg);
            last_update_time_ = cur_time;
        }
        last_sim_time_ = cur_time;
    }

    GZ_REGISTER_MODEL_PLUGIN(AckermannGazeboPlugin)
}
