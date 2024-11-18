#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>


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

            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/dummy2/pose", 10, std::bind(&Exploration_map::pose_callback, this, std::placeholders::_1)
            );

            btl_sub = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
                "/behavior_tree_log", 5, std::bind(&Exploration_map::btl_callback, this, std::placeholders::_1)
            );

            laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&Exploration_map::scan_callback, this, std::placeholders::_1)
            );

            goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visited_points", 10);


        }

    protected:

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            P new_goal;

            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "Failed to compute pose.");
                    need_set_goal_sign = true;
                    // 박스 장애물 안 -1 Array 는 어떻게 처리해야 빨리 확인할까?
                    break;
                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    fail_count = 0;
                }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "Failed to move.");
                    fail_count++;
                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "READY 2 MOVE");
                    if(fail_count == 5){
                        // performReverse();
                        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        fail_count = 0;
                    } // 새로운 문제. 이동 완료했음에도 이미 전송된 좌표들이 많아서 그 자리서 왔다갔다함
                      // 맵 확장될 때 마다 visited map 초기화로 해결

                    need_set_goal_sign = true;
                }
            }
        }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            if(!need_scan_data_sign){ return; }
            laser_msg = msg;
            if(isObstacleinPath(last_pose_, current_goal)){
                visited[current_goal] = true;
                publishVisitedPoints();
                setGoal(last_pose_);
            }else{
                RCLCPP_INFO(this->get_logger(), "CAN MOVE");
                need_scan_data_sign = false;
                need_set_goal_sign = false;
                pubGoal(current_goal);
            }

        }

        bool isObstacleinPath(const geometry_msgs::msg::PoseStamped& current_pose, const P& goal_pixel){
            double goal_y = origin_y + goal_pixel.first * resolution;
            double goal_x = origin_x + goal_pixel.second * resolution;

            double dy = goal_y - current_pose.pose.position.y;
            double dx = goal_x - current_pose.pose.position.x;

            double angle_to_goal = std::atan2(dy, dx);
            int ranges_size = laser_msg->ranges.size();

            float scan_offset = 0.3;

            int angle_idx = (angle_to_goal - laser_msg->angle_min) / laser_msg->angle_increment;
            if(angle_idx < 0 || angle_idx >= ranges_size){
                return true;
            }
            double range_to_goal = std::sqrt(dx * dx + dy * dy);
            if(laser_msg->ranges[angle_idx] - scan_offset < range_to_goal){
                return true;
            }
            // RCLCPP_INFO(this->get_logger()," range = %f, goal = %f",laser_msg->ranges[angle_idx], range_to_goal );
            // RCLCPP_INFO(this->get_logger(), "angle : %f", angle_to_goal);
            return false;
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }
            if(!first_goal_rdy){ first_goal_rdy = true; }
            map_ = msg;


            if(last_width_ != map_->info.width || last_height_ != map_->info.height){
                origin_x = map_->info.origin.position.x;
                origin_y = map_->info.origin.position.y;
                width = map_->info.width;
                height = map_->info.height;
                resolution = map_->info.resolution;
                visited.clear();
                RCLCPP_INFO(this->get_logger(),"W : %d, H : %d, origin x : %f, origin y : %f", width, height, origin_x, origin_y);
                RCLCPP_INFO(this->get_logger(),"Map size : %ld", visited.size());
                RCLCPP_INFO(this->get_logger(),"Map begin : %d, %d ", visited.begin()->first.first, visited.begin()->first.second);
                last_width_ = width;
                last_height_ = height;
            }

            search();

            publishVisitedPoints();

        }

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            pose_ = *msg;
            last_pose_ = pose_;
            if(!first_goal_set && first_goal_rdy){
                first_goal_set = true;
                need_set_goal_sign = true;
            }
            if(need_set_goal_sign && !need_scan_data_sign){
                setGoal(pose_);
                need_set_goal_sign = false;
            }
        }

        void publishVisitedPoints() {


                visualization_msgs::msg::Marker points_visited;
                points_visited.header.frame_id = "map";
                points_visited.header.stamp = this->get_clock()->now();
                points_visited.ns = "visited_points";
                points_visited.id = 0;
                points_visited.type = visualization_msgs::msg::Marker::POINTS;
                points_visited.action = visualization_msgs::msg::Marker::ADD;
                points_visited.scale.x = 0.025;  // 점 크기 설정 (m 단위)
                points_visited.scale.y = 0.025;
                points_visited.color.a = 1.0;   // 점 투명도 설정
                points_visited.color.r = 1.0;
                points_visited.color.g = 0.0;
                points_visited.color.b = 0.0;

                for (const auto& itr : visited) {
                    if (itr.second == false) { // 아직 방문되지 않은 좌표만 추가
                        geometry_msgs::msg::Point p;
                        p.y = itr.first.first * resolution + origin_y;
                        p.x = itr.first.second * resolution + origin_x;
                        p.z = 0.0;
                        points_visited.points.push_back(p);
                    }
                }

                marker_pub_->publish(points_visited);
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


        void search(){ // 언제 다시 동기화 해야할지
            //현재 지도의 데이터가 -1 인 idx의 픽셀좌표를 visited 맵에 저장
            //현재 지도의 데이터에서 -1 인 idx의 픽셀좌표를 저장
            auto &map = *map_;
            RCLCPP_INFO(this->get_logger(),"search");
            for(unsigned int y = 10; y < height - 10; y++){
                for(unsigned int x = 10; x < width - 10; x++){
                    unsigned int idx = pixelcoordtoidx(y, x, width);
                    if(map.data[idx] == -1){//&& visited.find({y, x}) == visited.end()){
                        visited.insert({{y, x}, false}); // 맵이 업데이트될 때 좌표는 어떻게 변하지?
                                                        // 장애물 안쪽은 미탐색구역으로 남아있어서 초기화하면 다시 판별해야함
                                                        //수정필요

                    }else{
                        visited[{y, x}] = true;
                    }
                }
            }
        }

        void pubGoal(const P &goal){

            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";


            visited[goal] = true;

            goal_msg.pose.position.y = origin_y + goal.first * resolution;
            goal_msg.pose.position.x = origin_x + goal.second * resolution;

            RCLCPP_INFO(this->get_logger(), "Goal : (x : %f, y : %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);

            goal_pub_->publish(goal_msg);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        void setGoal(const geometry_msgs::msg::PoseStamped &pose_){
            P near_goal = {0, 0};
            unsigned int pixel_y = (pose_.pose.position.y - origin_y) / resolution;
            unsigned int pixel_x = (pose_.pose.position.x - origin_x) / resolution;
            P current_pixel_coord = {pixel_y, pixel_x};

            double min_dist = std::numeric_limits<double>::max();
            for(auto &itr : visited){
                if(itr.second == false) {
                    unsigned int temp_idx = pixelcoordtoidx(itr.first.first, itr.first.second, last_width_);
                    if(map_->data[temp_idx] != -1){
                        itr.second = true;
                        RCLCPP_INFO(this->get_logger(),"Already gone");
                        continue;
                    }
                    // RCLCPP_INFO(this->get_logger(),"visited[itr] = %d, map_->data[idx] = %d", itr.second, map_->data[temp_idx]);
                    // RCLCPP_INFO(this->get_logger(), "current coord = ( %d, %d )", itr.first.first, itr.first.second);
                    double dist = distance(itr.first, current_pixel_coord);
                    if(dist < min_dist){
                        min_dist = dist;
                        near_goal = itr.first;
                    }
                }
            }
            if(near_goal == P{0, 0}){
                RCLCPP_INFO(this->get_logger(),"All point visited");
                pubGoal({0, 0});
            }else{
                RCLCPP_INFO(this->get_logger(),"set near goal (%d, %d)", near_goal.first, near_goal.second);
                current_goal = near_goal;
                need_scan_data_sign = true;
                // pubGoal(near_goal);
            }
        }

        ///C1 > C2
        double distance(P C1, P C2){
            return std::hypot((C1.first - C2.first), (C1.second - C2.first));
        }

        unsigned int pixelcoordtoidx(unsigned int y, unsigned int x, uint32_t width){
            return (y * width + x);
        }



        int fail_count = 0;
        bool first_goal_rdy = false; // 처음 Map 을 수신 후 활성
        bool first_goal_set = false; // Map 을 처음 수신 후 pose를 처음 수신하면 활성
        bool need_set_goal_sign = false; // 활성 될 때 마다 setGoal 실행
        bool need_scan_data_sign = false; // 활성되면 scan data를 활용
        P current_goal;
        std::map<P, bool> visited;

        double origin_x = 0.0;
        double origin_y = 0.0;

        float resolution = 0.0;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t last_width_ = 0;
        uint32_t last_height_ = 0;


        geometry_msgs::msg::PoseStamped last_pose_;
        geometry_msgs::msg::PoseStamped pose_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        geometry_msgs::msg::PoseStamped goal_msg;
        sensor_msgs::msg::LaserScan::SharedPtr laser_msg;


        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
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
