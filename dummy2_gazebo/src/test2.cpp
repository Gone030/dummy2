#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>


#include <cmath>
#include <map>
#include <stack>
#include <vector>
#include <string>



class Exploration_map : public rclcpp::Node
{
    public:

        Exploration_map() : rclcpp::Node("Exploration_map_Node"){

            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&Exploration_map::map_callback, this, std::placeholders::_1)
            );

            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/pose", 10, std::bind(&Exploration_map::pose_callback, this, std::placeholders::_1)
            );

            // marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            //     "/marker", 1, std::bind(&Exploration_map::marker_callback, this, std::placeholders::_1)
            // );

            btl_sub = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
                "/behavior_tree_log", 1, std::bind(&Exploration_map::btl_callback, this, std::placeholders::_1)
            );

            goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);


            // timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&Exploration_map::explore, this));
        }

    protected:

        // void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
        //     if(msg->markers.back().color.r == 1.0) {explore();}
        // }

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            if(msg->event_log.empty()) {explore();}
            else{
                for(size_t i = 0; i < msg->event_log.size(); i ++){
                    std::string node_name_ = msg->event_log[i].node_name;
                    std::string current_status_ = msg->event_log[i].current_status;
                    std::string previous_status_ = msg->event_log[i].previous_status;
                    if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                        RCLCPP_WARN(this->get_logger(), "Failed to compute pose.");
                        explore();
                        RCLCPP_WARN(this->get_logger(), "End to searching.");
                        break;
                    }
                    else if(node_name_ == "FollowPath" && current_status_ == "IDLE" && previous_status_ == "SUCCESS"){
                        RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                        clearAround(current_goal);
                        nextGoalStack_ = std::stack<std::pair<unsigned int, unsigned int>>();
                        explore(); // 임시
                        break;
                    }
                    else if(node_name_ == "FollowPath" && current_status_ == "IDLE" && previous_status_ == "FAILURE"){
                        RCLCPP_WARN(this->get_logger(), "Failed to move.");

                        unsigned int current_y = (pose_.pose.pose.position.y - map_->info.origin.position.y) / map_->info.resolution;
                        unsigned int current_x = (pose_.pose.pose.position.x - map_->info.origin.position.x) / map_->info.resolution;

                        clearAround({current_y, current_x});
                        nextGoalStack_ = std::stack<std::pair<unsigned int, unsigned int>>();
                        explore();
                        RCLCPP_WARN(this->get_logger(), "End to searching.");
                        break;
                    }
                }

            }
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }
            map_ = msg;
        }

        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            last_pose_ = pose_;
            pose_ = *msg;
        }

/*        bool isGoalReached(const geometry_msgs::msg::PoseWithCovarianceStamped &current_pose, const geometry_msgs::msg::PoseStamped &goal_pose){
            // RCLCPP_INFO(this->get_logger(), "Verifying that the move is complete");

            if(Collision){
                RCLCPP_INFO(this->get_logger(), "Collision ahead. Need new goal.");
                return true;
            }
            if(goal_pose.pose.position.x == 0 && goal_pose.pose.position.y == 0){
                RCLCPP_INFO(this->get_logger(), "goal is 0,0");
                return true;
            }
            double dist = distance(current_pose.pose.pose.position.y, current_pose.pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.x);



            // RCLCPP_INFO(this->get_logger(), "temp_idx : %d , temp_y : %d , temp_x : %d", temp_idx, grid_y, grid_x);

            if(last_dist == dist){
                RCLCPP_INFO(this->get_logger(), "Robot is stuck, resetting goal.");
                return true;
            }
            last_dist = dist;
            double threshold = 0.5;
            RCLCPP_INFO(this->get_logger(), "Distance : %f, Threshold : %f", dist, threshold);
            if(dist < threshold){
                RCLCPP_INFO(this->get_logger(), "HAS GOAL RECHED");
                int radius = 100;
                for(int i = -radius; i <= radius; i++){
                    for(int j = -radius; j <= radius; j++){
                        visited[{(current_goal.first + i), (current_goal.second + j)}] = true;
                    }
                }
                nextGoalStack_ = std::stack<std::pair<unsigned int, unsigned int>>();
            }
            return (dist < threshold);
        } */
        void clearAround(std::pair<unsigned int , unsigned int> current_goal){
            int radius = 100;
            for(int i = -radius; i <= radius; i++){
                for(int j = -radius; j <= radius; j++){
                    visited[{(current_goal.first + i), (current_goal.second + j)}] = true;
                }
            }
        }

        bool isoutofrange(unsigned int pixel_y, unsigned int pixel_x){
            if(pixel_y >= map_->info.height - 50 || pixel_x >= map_->info.width - 50 || pixel_x <= 50 || pixel_y <= 50){
                // RCLCPP_WARN(this->get_logger(), "OUT OF RANGE , y : %d , x : %d, height : %d, width : %d", pixel_y, pixel_x, map_->info.height, map_->info.width);
                visited[current_goal] = true;
                return false;
            }

            // auto coord_on_map = pixeltocoord(pixel_y, pixel_x);
            return true;
        }

        std::vector<std::pair<unsigned int, unsigned int>> search(){

            RCLCPP_INFO(this->get_logger(), "Searching");
            auto &map = *map_;
            // std::pair<unsigned int, unsigned int> new_goal = {0, 0}; // (y, x)

            // RCLCPP_INFO(this->get_logger(), "set current coord");
            //현재 좌표를 픽셀위치로 변환.
            // unsigned int current_y = (pose_.pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
            // unsigned int current_x = (pose_.pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;

            unsigned int width = map.info.width;
            unsigned int height = map.info.height;
            RCLCPP_INFO(this->get_logger(),"W : %d, H : %d", width, height);
            std::vector<std::pair<unsigned int, unsigned int>> candidates;

            // unsigned int robot_idx = current_x * width + current_y;

            // double last_dist = 0;

            for(unsigned int y = 0; y < height; y++){
                for(unsigned int x = 0; x < width; x++){
                    unsigned int idx = pixelcoordtoidx(y, x, width);
                    if(map_->data[idx] == -1 && !visited[{y, x}]){
                        // double dist = distance(current_y, current_x, y, x);
                        if(isoutofrange(y,x)){
                            candidates.push_back({y, x});
                        }
                    }
                }
            }
            return candidates;
        }

        void setgoal(const std::pair<unsigned int, unsigned int> &goal){

            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";

            current_goal = goal;
            visited[current_goal] = true;

            goal_msg.pose.position.y = pixeltocoord(goal.first);
            goal_msg.pose.position.x = pixeltocoord(goal.second);

            RCLCPP_INFO(this->get_logger(), "Goal : (%f, %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);

            goal_pub_->publish(goal_msg);
        }

        void explore(){
            RCLCPP_INFO(this->get_logger(), "Explore");

            if(nextGoalStack_.empty()) {
                RCLCPP_INFO(this->get_logger(),"Empty stack");
                std::vector<std::pair<unsigned int, unsigned int>> new_goals = search();
                for (const auto& goal : new_goals){
                    nextGoalStack_.push(goal);
                }
            }
            if(!nextGoalStack_.empty()){
            // 새로운 목표를 검색하여 스택에 추가
                RCLCPP_INFO(this->get_logger(),"Add stack");
                auto newGoal = nextGoalStack_.top();
                current_goal = newGoal;
                if (newGoal.first != 0 && newGoal.second != 0){
                    nextGoalStack_.pop();
                    setgoal(newGoal);
                }
            }
        }

        ///x2, y2 > x1, y1
        double distance(double y1, double x1, double y2, double x2){
            return std::hypot((x2 - x1), (y2 - y1));
        }

        // std::pair<unsigned int, unsigned int> idxtopixelcoord(unsigned int idx, unsigned int width){
        //     return {idx / width, idx % width};
        // }

        unsigned int pixelcoordtoidx(unsigned int y, unsigned int x, unsigned int width){
            return (y * width + x);
        }

        double pixeltocoord(unsigned int p){
            return map_->info.origin.position.y + p * map_->info.resolution;
        }



        std::pair<unsigned int, unsigned int> current_goal;
        std::map<std::pair<unsigned int, unsigned int>, bool> visited;
        std::stack<std::pair<unsigned int, unsigned int>> nextGoalStack_;

        geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
        geometry_msgs::msg::PoseWithCovarianceStamped pose_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        geometry_msgs::msg::PoseStamped goal_msg;


        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;
        // rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration_map>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
