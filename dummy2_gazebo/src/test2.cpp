#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <visualization_msgs/msg/marker.hpp>



#include <cmath>
#include <map>
#include <stack>
#include <vector>
#include <string>
#include <utility>

using P = std::pair<unsigned int, unsigned int>;

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

            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visited_points", 10);


            // timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&Exploration_map::explore, this));
        }

    protected:

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            P new_goal;
            bool fail = false;
            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "Failed to compute pose. %d %d",current_goal.first, current_goal.second);
                    RCLCPP_INFO(this->get_logger(),"Map size : %ld", visited.size());
                    visited[current_goal] = true;
                    fail_count++;
                    RCLCPP_WARN(this->get_logger(), "FAIL COUNT : %d", fail_count);
                    if(fail_count == 10){
                        performReverse();
                        fail_count = 0;
                        new_goal = setGoal(fail = true);
                    }else if(fail_count > 5){
                        new_goal = setGoal(fail = true);
                    }else{
                        new_goal = setGoal(fail = false);
                    }

                    RCLCPP_INFO(this->get_logger(),"set new goal : %d %d", new_goal.first, new_goal.second);
                    RCLCPP_INFO(this->get_logger(),"visited[new_goal] : %d", visited[new_goal]);

                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    clearAround(current_goal);
                    RCLCPP_INFO(this->get_logger(), "visited data : %d ", visited[current_goal]);
                    fail_count = 0;
                    new_goal = setGoal(fail);
                    // RCLCPP_INFO(this->get_logger(),"set new goal : %d %d", new_goal.first, new_goal.second);
                    RCLCPP_INFO(this->get_logger(), "visited temp : %d ", visited[new_goal]);

                }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "Failed to move. %d %d", current_goal.first, current_goal.second);
                    RCLCPP_WARN(this->get_logger(), "visited[current_goal] : %d", visited[current_goal]);
                    visited[current_goal] = true;
                    clearAround({(pose_.pose.pose.position.y - map_->info.origin.position.y) / map_->info.resolution,
                                 (pose_.pose.pose.position.x - map_->info.origin.position.x) / map_->info.resolution});
                    new_goal = setGoal(fail);
                    // RCLCPP_INFO(this->get_logger(),"set new goal : %d %d", new_goal.first, new_goal.second);
                    RCLCPP_INFO(this->get_logger(),"visited[new_goal] : %d", visited[new_goal]);
                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "READY 2 MOVE");
                    pubgoal(new_goal);
                    publishVisitedPoints();
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
            width = map_->info.width;
            height = map_->info.height;
            origin_x = map_->info.origin.position.x;
            origin_y = map_->info.origin.position.y;

            if(last_width_ != width){
                search();
            }
            last_width_ = width;
            // if(!first){
            //     first = true;
                // explore();
            // }

        }

        void publishVisitedPoints() {
                visualization_msgs::msg::Marker points;
                points.header.frame_id = "map";
                points.header.stamp = this->get_clock()->now();
                points.ns = "visited_points";
                points.id = 0;
                points.type = visualization_msgs::msg::Marker::POINTS;
                points.action = visualization_msgs::msg::Marker::ADD;
                points.scale.x = 0.025;  // 점 크기 설정 (m 단위)
                points.scale.y = 0.025;
                points.color.a = 1.0;   // 점 투명도 설정
                points.color.r = 1.0;
                points.color.g = 0.0;
                points.color.b = 0.0;

                for (const auto& itr : visited) {
                    if (itr.second == false) { // 아직 방문되지 않은 좌표만 추가
                        geometry_msgs::msg::Point p;
                        p.y = itr.first.first * map_->info.resolution + origin_y;
                        p.x = itr.first.second * map_->info.resolution + origin_x;
                        p.z = 0.0;
                        points.points.push_back(p);
                    }
                }

                marker_pub_->publish(points);
            }

        void performReverse() {
            geometry_msgs::msg::Twist reverse_msg;
            reverse_msg.linear.x = -0.2;  // 음수 값으로 설정하여 후진
            reverse_msg.angular.z = 0.0;  // 회전 없이 직진 후진

            // 후진 퍼블리시
            for (int i = 0; i < 10; i++) { // 후진 명령을 잠시 유지하기 위해 반복 퍼블리시
                cmd_pub_->publish(reverse_msg);
                rclcpp::sleep_for(std::chrono::milliseconds(100));  // 퍼블리시 간격 설정
            }

            // 정지 명령 퍼블리시
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_pub_->publish(stop_msg);
        }


        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            last_pose_ = pose_;
            pose_ = *msg;
        }


        void clearAround(P current_goal){ // 이거 의도대로 작동하는지 살펴보기
            int radius = 25;
            for(int i = -radius; i <= radius; i++){
                for(int j = -radius; j <= radius; j++){
                    visited[{(current_goal.first + i), (current_goal.second + j)}] = true;
                }
            }
        }

        bool isoutofrange(unsigned int pixel_y, unsigned int pixel_x){
            double coord_x = origin_x + pixel_x * map_->info.resolution;
            double coord_y = origin_y + pixel_y * map_->info.resolution;
            if(coord_y >= std::fabs(origin_y) - 3 || coord_x >= std::fabs(origin_x) - 3
                || coord_y <= origin_y + 3 || coord_x <= origin_x + 3){
                // RCLCPP_WARN(this->get_logger(), "OUT OF RANGE , y : %d , x : %d, height : %d, width : %d", pixel_y, pixel_x, map_->info.height, map_->info.width);
                return false;
            }

            // auto coord_on_map = pixeltocoord(pixel_y, pixel_x);
            return true;
        }

        void search(){
            //현재 지도의 데이터가 -1 인 idx의 픽셀좌표를 visited 맵에 저장
            //현재 지도의 데이터에서 -1 인 idx의 픽셀좌표를 저장
            RCLCPP_INFO(this->get_logger(), "Searching");
            auto &map = *map_;


            RCLCPP_INFO(this->get_logger(),"W : %d, H : %d, origin x : %f, origin y : %f", width, height, map.info.origin.position.x, map.info.origin.position.y);

            for(unsigned int y = 15; y < height - 15; y++){
                for(unsigned int x = 15; x < width - 15; x++){
                    unsigned int idx = pixelcoordtoidx(y, x, width);
                    if(map_->data[idx] == -1 && visited.find({y, x}) == visited.end()){
                        visited.insert({{y, x}, false});
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(),"Map size : %ld", visited.size());
            RCLCPP_INFO(this->get_logger(),"Map begin : %d, %d ", visited.begin()->first.first, visited.begin()->first.second);
        }

        void pubgoal(const P &goal){

            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";

            current_goal = goal;
            // visited[current_goal] = true;

            goal_msg.pose.position.y = origin_x + goal.first * map_->info.resolution;
            goal_msg.pose.position.x = origin_x + goal.second * map_->info.resolution;
            unsigned int idx_ = goal.first * width + goal.second;

            RCLCPP_INFO(this->get_logger(), "Goal : (%f, %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Checking pixel at (%d, %d): %d", goal.first, goal.second, map_->data[idx_]);

            goal_pub_->publish(goal_msg);
        }

        P setGoal(bool fail){
            P near_goal = {0, 0};
            P far_goal = {0, 0};
            unsigned int pixel_y = (pose_.pose.pose.position.y - origin_y) / map_->info.resolution;
            unsigned int pixel_x = (pose_.pose.pose.position.x - origin_x) / map_->info.resolution;
            P current_pixel_coord = {pixel_y, pixel_x};
            if(!fail){
                double min_dist = std::numeric_limits<double>::max();
                for(auto &itr : visited){
                    if(itr.second == false) {
                        unsigned int temp_idx = pixelcoordtoidx(itr.first.first, itr.first.second, width);
                        if(map_->data[temp_idx] != -1){
                            itr.second = true;
                            RCLCPP_INFO(this->get_logger(),"Already gone");
                            continue;
                        }
                        double dist = distance(itr.first, current_pixel_coord);
                        if(dist < min_dist){
                            min_dist = dist;
                            near_goal = itr.first;
                        }
                    }
                }
                if(near_goal == P{0, 0}){
                    RCLCPP_INFO(this->get_logger(),"All point visited");
                    return {0, 0};
                }else{
                    RCLCPP_INFO(this->get_logger(),"set near goal (%d, %d)", near_goal.first, near_goal.second);
                }
                return near_goal;
            }
            else{
                double max_dist = 0.0;
                for(auto &itr : visited){
                    if(itr.second == false){
                        unsigned int temp_idx = pixelcoordtoidx(itr.first.first, itr.first.second, width);
                        if(map_->data[temp_idx] != -1){
                            itr.second = true;
                            RCLCPP_INFO(this->get_logger(),"Already gone");
                            continue;
                        }
                        double dist = distance(itr.first, current_pixel_coord);
                        if(dist > max_dist){
                            max_dist = dist;
                            far_goal = itr.first;
                        }
                    }
                }
                if(far_goal == P{0, 0}){
                    RCLCPP_INFO(this->get_logger(),"All point visited");
                    return {0, 0};
                }else{
                    RCLCPP_INFO(this->get_logger(),"set far goal (%d, %d)", far_goal.first, far_goal.second);
                }
                return far_goal;
            }
        }

        // void explore(){

        //     if(last_width_ != map_->info.width) {
        //         RCLCPP_INFO(this->get_logger(),"change map info.");
        //         visited.clear();
        //         search();
        //         pubgoal(setGoal());
        //     }
        //     else{
        //         RCLCPP_INFO(this->get_logger(),"same map info.");
        //         pubgoal(setGoal());
        //     }
        //     // if(!nextGoalStack_.empty()){
        //     // // 새로운 목표를 검색하여 스택에 추가
        //     //     RCLCPP_INFO(this->get_logger(),"Use stack top");
        //     //     auto newGoal = nextGoalStack_.top();
        //     //     current_goal = newGoal;
        //     //     if (newGoal.first != 0 && newGoal.second != 0 && !visited[newGoal]){
        //     //         RCLCPP_INFO(this->get_logger(),"INSIDE");
        //     //         nextGoalStack_.pop();
        //     //         setgoal(newGoal);
        //     //     }
        //     // }
        // }

        ///C1 > C2
        double distance(P C1, P C2){
            return std::hypot((C1.first - C2.first), (C1.second - C2.first));
        }

        // p idxtopixelcoord(unsigned int idx, unsigned int width){
        //     return {idx / width, idx % width};
        // }

        unsigned int pixelcoordtoidx(unsigned int y, unsigned int x, unsigned int width){
            return (y * width + x);
        }



        int fail_count = 0;
        unsigned int last_width_;
        P current_goal;
        std::map<P, bool> visited;

        double origin_x = 0.0;
        double origin_y = 0.0;

        unsigned int width = 0.0;
        unsigned int height = 0.0;


        geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
        geometry_msgs::msg::PoseWithCovarianceStamped pose_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        geometry_msgs::msg::PoseStamped goal_msg;


        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration_map>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
