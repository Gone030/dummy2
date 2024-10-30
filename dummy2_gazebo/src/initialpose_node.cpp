#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() : Node("initial_pose_publisher"){

        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_yaw", 0.0);

        double initial_x = this->get_parameter("initial_x").as_double();
        double initial_y = this->get_parameter("initial_y").as_double();
        double initial_yaw = this->get_parameter("initial_yaw").as_double();

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = this->now();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.pose.pose.position.x = initial_x;
        initial_pose_msg.pose.pose.position.y = initial_y;
        initial_pose_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_yaw); // Roll, Pitch, Yaw
        initial_pose_msg.pose.pose.orientation = tf2::toMsg(q);

        // 일정 시간 간격으로 초기 위치 메시지 발행
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(500),
        //     [this, initial_pose_msg]() {
        //         initial_pose_pub_->publish(initial_pose_msg);
        //     }
        // );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
