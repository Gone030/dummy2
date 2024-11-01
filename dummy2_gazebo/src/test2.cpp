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
                "/behavior_tree_log", 5, std::bind(&Exploration_map::btl_callback, this, std::placeholders::_1)
            );

            goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);


            // timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&Exploration_map::explore, this));
        }

    protected:

        // void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
        //     if(msg->markers.back().color.r == 1.0) {explore();}
        // }

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){

            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "Failed to compute pose.");
                    nextGoalStack_.pop();
                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    clearAround(current_goal);
                    unsigned int idx_ = current_goal.first * map_->info.width + current_goal.second;
                    RCLCPP_INFO(this->get_logger(), "map data : %d ", map_->data[idx_]);
                    nextGoalStack_ = std::stack<std::pair<unsigned int, unsigned int>>();
                    explore();
                }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "Failed to move.");
                    nextGoalStack_.pop();
                    explore();
                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "goal set");
                    goal_set();
                }
            }
        }
        bool first = false;
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }
            map_ = msg;
            explore();
            // if(!first){
            //     first = true;
            //     goal_set();
            // }

        }

        void goal_set(){ ///temp
            if(!nextGoalStack_.empty()){
                while(1){
                    if(nextGoalStack_.empty()){
                        RCLCPP_INFO(this->get_logger(), "EMPTY.");
                        setgoal({0,0}); // 비어있으면 임시로 0,0
                        break;
                    }
                    auto new_goal = nextGoalStack_.top();
                    if(visited[new_goal]) {
                        nextGoalStack_.pop();
                        continue;
                    }
                    if(new_goal.first != 0 && new_goal.second != 0){
                        current_goal = new_goal;
                        break; // 조건 부합 시 탈출
                    }
                    clearAround(current_goal);
                    nextGoalStack_.pop();
                }
                setgoal(current_goal); // Goal pub
            }
        }

        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            last_pose_ = pose_;
            pose_ = *msg;
        }


        void clearAround(std::pair<unsigned int , unsigned int> current_goal){
            int radius = 10;
            for(int i = -radius; i <= radius; i++){
                for(int j = -radius; j <= radius; j++){
                    visited[{(current_goal.first + i), (current_goal.second + j)}] = true;
                }
            }
        }

        bool isoutofrange(unsigned int pixel_y, unsigned int pixel_x){
            double coord_x = pixeltocoord(pixel_x);
            double coord_y = pixeltocoord(pixel_y);
            if(coord_y >= std::fabs(map_->info.origin.position.y) - 3 || coord_x >= std::fabs(map_->info.origin.position.x) - 3
                || coord_y <= map_->info.origin.position.y + 3 || coord_x <= map_->info.origin.position.x + 3){
                // RCLCPP_WARN(this->get_logger(), "OUT OF RANGE , y : %d , x : %d, height : %d, width : %d", pixel_y, pixel_x, map_->info.height, map_->info.width);
                return false;
            }

            // auto coord_on_map = pixeltocoord(pixel_y, pixel_x);
            return true;
        }

        std::vector<std::pair<unsigned int, unsigned int>> search(){
            //현재 지도의 데이터에서 -1 인 idx의 픽셀좌표를 저장
            RCLCPP_INFO(this->get_logger(), "Searching");
            auto &map = *map_;
            // std::pair<unsigned int, unsigned int> new_goal = {0, 0}; // (y, x)

            unsigned int width = map.info.width;
            unsigned int height = map.info.height;
            RCLCPP_INFO(this->get_logger(),"W : %d, H : %d", width, height);
            std::vector<std::pair<unsigned int, unsigned int>> candidates;

            // unsigned int robot_idx = current_x * width + current_y;

            // double last_dist = 0;

            for(unsigned int y = 15; y < height - 15; y++){
                for(unsigned int x = 15; x < width - 15; x++){
                    unsigned int idx = pixelcoordtoidx(y, x, width);
                    if(map_->data[idx] == -1 && !visited[{y, x}]){
                        if(isoutofrange(y,x)){
                            candidates.push_back({y, x});
                        }
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(),"Vector size : %ld", candidates.size());
            return candidates;
        }

        void setgoal(const std::pair<unsigned int, unsigned int> &goal){

            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";

            current_goal = goal;
            // visited[current_goal] = true;

            goal_msg.pose.position.y = pixeltocoord(goal.first);
            goal_msg.pose.position.x = pixeltocoord(goal.second);
            unsigned int idx_ = goal.first * map_->info.width + goal.second;

            RCLCPP_INFO(this->get_logger(), "Goal : (%f, %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Checking pixel at (%d, %d): %d", goal.first, goal.second, map_->data[idx_]);

            goal_pub_->publish(goal_msg);
        }

        bool explore(){

            if(nextGoalStack_.empty()) {
                RCLCPP_INFO(this->get_logger(),"Empty stack");
                std::vector<std::pair<unsigned int, unsigned int>> new_goals = search();
                for (const auto& goal : new_goals){
                    nextGoalStack_.push(goal);
                }
                return true;
            }
            else{
                return false;
            }
            // if(!nextGoalStack_.empty()){
            // // 새로운 목표를 검색하여 스택에 추가
            //     RCLCPP_INFO(this->get_logger(),"Use stack top");
            //     auto newGoal = nextGoalStack_.top();
            //     current_goal = newGoal;
            //     if (newGoal.first != 0 && newGoal.second != 0 && !visited[newGoal]){
            //         RCLCPP_INFO(this->get_logger(),"INSIDE");
            //         nextGoalStack_.pop();
            //         setgoal(newGoal);
            //     }
            // }
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
