#ifndef EXPLORATION_NODE_HPP
#define EXPLORATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <vector>
#include <queue>
#include <unordered_map>

class ExplorationNode : public rclcpp::Node{
    public:
        ExplorationNode();
    private:

        void initializeCostmap();
        void explore();
        std::vector<geometry_msgs::msg::PoseStamped> dijkstra(const nav2_costmap_2d::Costmap2D* costmap,
                                                              const geometry_msgs::msg::PoseStamped& start,
                                                              const geometry_msgs::msg::PoseStamped& goal);
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

        void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "topic receive");
            initial_pose_ = *msg;

            if (!costmap_ros_) {
                initializeCostmap();
            }

            if (initial_pose_.pose.pose.position.x < costmap_ros_->getCostmap()->getOriginX() ||
                initial_pose_.pose.pose.position.x > costmap_ros_->getCostmap()->getOriginX() + costmap_ros_->getCostmap()->getSizeInMetersX() ||
                initial_pose_.pose.pose.position.y < costmap_ros_->getCostmap()->getOriginY() ||
                initial_pose_.pose.pose.position.y > costmap_ros_->getCostmap()->getOriginY() + costmap_ros_->getCostmap()->getSizeInMetersY()) {
                RCLCPP_ERROR(this->get_logger(), "Initial pose is out of the costmap bounds!");
            }else{
                RCLCPP_INFO(this->get_logger(), "Robot initial pose: x: %f, y: %f", initial_pose_.pose.pose.position.x, initial_pose_.pose.pose.position.y);
                explore();
            }
        }
};
#endif //EXPLORATION_NODE_HPP
