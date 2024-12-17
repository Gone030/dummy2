#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <queue>
#include <cmath>
#include <map>
#include <stack>
#include <vector>
#include <string>
#include <utility>

using P = std::pair<unsigned int, unsigned int>;

class MapManager {
    private:

        MapManager(const MapManager&) = delete;
        MapManager& operator=(const MapManager&) = delete;
        MapManager(MapManager&&) = delete;
        MapManager& operator=(MapManager&&) = delete;


        std::map<P, bool> visited_;
        std::set<std::pair<double, double>> obstacle_mask_;
        MapManager() {}

    public:
        static MapManager& getInstance(){
            static MapManager instance;
            return instance;
        }

        bool isVisited(unsigned int y, unsigned int x){
            return visited_[{y, x}];
        }

        void setVisited(unsigned int y, unsigned int x, bool value){
            visited_[{y, x}] = value;
        }

        void setVisited(P pixel_coord, bool value){
            visited_[pixel_coord] = value;
        }

        void insertVisited(unsigned int y, unsigned int x, bool value){
            visited_.insert({{y, x}, value});
        }

        void markVisited(P pixel_coord){
            visited_[pixel_coord] = true;
        }

        void clearVisited(){
            visited_.clear();
        }

        void addObstacleMask(double y, double x){
            obstacle_mask_.insert({y, x});
        }

        std::map<P, bool>& getVisited() {
            return visited_;
        }

        const std::set<std::pair<double, double>>& getObstacleMask() const {
            return obstacle_mask_;
        }
};

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

            visit_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visited_points", 10);
            obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacle_points", 10);

            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Exploration_map::timer_callback, this));

        }

    protected:

        void timer_callback(){
            if(scan_init) { return; }
            RCLCPP_INFO(this->get_logger(),"System status : map = %d , pose = %d, jobdone = %d", i_sub_map, i_sub_pose, jobdone);
            if(i_sub_map && i_sub_pose & jobdone){
                setGoal(last_pose_, map_);
                i_sub_map = false;
                i_sub_pose = false;
                jobdone = false;

            }
        }

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "Failed to compute pose.");
                    fail_count++;
                    MapManager::getInstance().setVisited(current_pixel_goal, true);
                    if(fail_count == 3){
                        scan_init = true;
                        break;
                    }
                    else if(fail_count > 5){
                        FailFloodfill(current_pixel_goal.first, current_pixel_goal.second);
                        fail_count = 0;
                    }
                    // 박스 장애물 안 -1 Array 는 어떻게 처리해야 빨리 확인할까? => floodfill

                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "Failed to move.");
                    fail_count++;
                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "READY 2 MOVE");
                    MapManager::getInstance().setVisited(current_pixel_goal, true);
                     // 새로운 문제. 이동 완료했음에도 이미 전송된 좌표들이 많아서 그 자리서 왔다갔다함
                      // 맵 확장될 때 마다 visited map 초기화로 해결 <- 기존에 탐색한 장애물도 초기화돼서 다시 탐색해야하는 비효율적인 동작 발생
                      // transform 함수 새로 만들어서 해결

                    jobdone = true;
                }
            }
        }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            if(scan_init){
                RCLCPP_INFO(this->get_logger(),"scan init");
                laser_msg = msg;
                excludeObstacle(laser_msg, last_pose_, map_);//visited, map_);
            }
            else{ return; }
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }
            // RCLCPP_INFO(this->get_logger(),"Map sub");
            // nav_msgs::msg::OccupancyGrid map = *msg;
            map_ = *msg;
            i_sub_map = true;

            if(last_origin_x != map_.info.origin.position.x || last_origin_y != map_.info.origin.position.y){

                transformVisited(map_);
                applyMaskToMask(map_.info.origin.position.x, map_.info.origin.position.y, map_.info.resolution);
                last_origin_x = map_.info.origin.position.x;
                last_origin_y = map_.info.origin.position.y;
            }

            search(map_);

            publishVisitedPoints(map_);

        }

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            last_pose_ = *msg;
            i_sub_pose = true;
            if(!first_move && !first_scan){
                RCLCPP_INFO(this->get_logger(),"Pose first sub");
                first_move = true;
            }
        }

        // void setanothergoal(double goal_y, double goal_x, const sensor_msgs::msg::LaserScan::SharedPtr& laser_data, ){

        // }



        void FailFloodfill(unsigned int sp_y, unsigned int sp_x){ // 모든 미탐색구역 지우는 현상 해결
            if(sp_x >= map_.info.width || sp_y >= map_.info.height) return;
            MapManager& mm = MapManager::getInstance();
            if(mm.isVisited(sp_y, sp_x) || mm.isVisited(sp_y + 1, sp_x) || mm.isVisited(sp_y - 1, sp_x) || mm.isVisited(sp_y, sp_x + 1) || mm.isVisited(sp_y, sp_x - 1)) return;
            mm.markVisited({sp_y, sp_x});
            FailFloodfill(sp_y, sp_x + 1);
            FailFloodfill(sp_y, sp_x - 1);
            FailFloodfill(sp_y + 1, sp_x);
            FailFloodfill(sp_y - 1, sp_x);
        }

        std::vector<std::vector<P>> GetObstacleShapes(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan,
                                                      const geometry_msgs::msg::PoseStamped& robot_pose,
                                                      const nav_msgs::msg::OccupancyGrid& map){
            std::vector<std::vector<P>> obstacle;
            std::vector<P> current_obs;
            RCLCPP_INFO(this->get_logger(),"scan obstacle shapes");

            double robot_x = robot_pose.pose.position.x;
            double robot_y = robot_pose.pose.position.y;
            double prev_dist = 0.0;
            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            double width = map.info.width;
            double height = map.info.height;
            double resolution = map.info.resolution;

            tf2::Quaternion tf2_quat;
            tf2::fromMsg(robot_pose.pose.orientation, tf2_quat);
            double theta = tf2::getYaw(tf2_quat);

            for(size_t i = 0; i < laser_scan->ranges.size(); i++){
                double laser_dist = laser_scan->ranges[i];
                double laser_angle = laser_scan->angle_min + i * laser_scan->angle_increment;

                if (std::isnan(laser_dist) || std::isinf(laser_dist) ||
                    laser_dist < laser_scan->range_min || laser_dist > laser_scan->range_max) {
                    continue;
                }


                double sensor_offset_y = 0.10;
                double sensor_offset_x = 0.27;

                //로봇 기준 장애물 위치
                double obs_y_robot = laser_dist * sin(laser_angle) + sensor_offset_y;
                double obs_x_robot = laser_dist * cos(laser_angle) + sensor_offset_x;

                //지도 기준 장애물 위치
                double obs_y_map = robot_y + obs_x_robot * sin(theta) + obs_y_robot * cos(theta);
                double obs_x_map = robot_x + obs_x_robot * cos(theta) - obs_y_robot * sin(theta);

                if (obs_x_map < origin_x || obs_y_map < origin_y ||
                    obs_x_map >= origin_x + width * resolution ||
                    obs_y_map >= origin_y + height * resolution) {
                    continue;
                }

                MapManager::getInstance().addObstacleMask(obs_y_map, obs_x_map);

                unsigned int pixel_y = (obs_y_map - origin_y) / resolution;
                unsigned int pixel_x = (obs_x_map - origin_x) / resolution;
                if (pixel_x >= width || pixel_y >= height) {
                    continue; // 무효한 좌표는 무시
                }

                double angle_diff = laser_angle - (laser_scan->angle_min + (i - 1) * laser_scan->angle_increment);
                double distance_diff = fabs(laser_dist - prev_dist);
                if (!current_obs.empty() && (distance_diff > 0.5 || angle_diff > 0.1)) {
                    obstacle.push_back(current_obs);
                    current_obs.clear();
                }

                current_obs.emplace_back(pixel_y, pixel_x);
                prev_dist = laser_dist;
            }
            if(!current_obs.empty()){
                obstacle.push_back(current_obs);
            }
            return obstacle;
        }

        void ObstacleFloodFill(const std::vector<std::vector<P>>& obstacles,
                       const nav_msgs::msg::OccupancyGrid& map){
            RCLCPP_INFO(this->get_logger(),"flood fill");
            int temp_size = obstacles.size();
            MapManager& mm = MapManager::getInstance();
            RCLCPP_INFO(this->get_logger(),"obstacles size : %d ",temp_size);
            for(const auto& obstacle : obstacles){
                if(obstacle.size() < 3) continue;
                std::queue<P> q;

                unsigned int min_x = std::numeric_limits<unsigned int>::max();
                unsigned int min_y = std::numeric_limits<unsigned int>::max();
                unsigned int max_x = 0;
                unsigned int max_y = 0;
                for (const auto& point : obstacle) {
                    min_x = std::min(min_x, point.second);
                    min_y = std::min(min_y, point.first);
                    max_x = std::max(max_x, point.second);
                    max_y = std::max(max_y, point.first);
                }
                unsigned int seed_x = (min_x + max_x) / 2;
                unsigned int seed_y = (min_y + max_y) / 2;
                if(mm.isVisited(seed_y, seed_x)){
                    // RCLCPP_INFO(this->get_logger(),"already visited seed coord");
                    // RCLCPP_INFO(this->get_logger(), "seed_x : %d, seed_y : %d", seed_x, seed_y);
                    continue;
                }
                q.push({seed_y, seed_x});
                unsigned int temp_offset = 5;
                while(!q.empty()){
                    // int temp = q.size();
                    // RCLCPP_INFO(this->get_logger(), "q.size : %d", temp);
                    auto [y, x] = q.front();
                    q.pop();
                    if(x < min_x - temp_offset || x > max_x + temp_offset || y < min_y - temp_offset || y > max_y + temp_offset){
                        // RCLCPP_INFO(this->get_logger(),"out of range");
                        continue;
                    }
                    if(mm.isVisited(y, x)){
                        // RCLCPP_INFO(this->get_logger(),"already visited");
                        continue;
                    }
                    if (y >= map.info.height || x >= map.info.width){
                        // RCLCPP_INFO(this->get_logger(),"out of range2");
                        continue;
                    }
                    mm.markVisited({y, x});
                    // RCLCPP_INFO(this->get_logger(), "x : %d, y : %d, visit? : %d", x, y, visited[{y, x}]);
                    q.push({y + 1, x});
                    q.push({y - 1, x});
                    q.push({y, x + 1});
                    q.push({y, x - 1});
                }
            }
        }

        void excludeObstacle(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan,
                             const geometry_msgs::msg::PoseStamped& robot_pose,
                            //  std::map<P, bool>& visited,
                             const nav_msgs::msg::OccupancyGrid& map){
            RCLCPP_INFO(this->get_logger(),"scan obstacle");

            auto obstacles = GetObstacleShapes(laser_scan, robot_pose, map);

            publishObstaclePoints(obstacles, map);
            ObstacleFloodFill(obstacles, map);
            scan_init = false;
            jobdone = true;
            RCLCPP_INFO(this->get_logger(),"scan obstacle done");

        }

        void applyMaskToMask(double origin_x, double origin_y, double resolution){
            MapManager& mm = MapManager::getInstance();
            for(const auto& coord : mm.getObstacleMask()){
                unsigned int pixel_y = static_cast<unsigned int>(std::round((coord.first - origin_y) / resolution));
                unsigned int pixel_x = static_cast<unsigned int>(std::round((coord.second - origin_x) / resolution));

                mm.setVisited(pixel_y, pixel_x, true);
            }
        }


        void transformVisited(const nav_msgs::msg::OccupancyGrid& map){
            std::map<P, bool> transformed_visited; // 변환이 완전히 안되는지 탐색완료된 장애물 내부에 격자로 미탐색구역으로 남음
            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            double resolution = map.info.resolution;
            MapManager& mm = MapManager::getInstance();
            for(auto iter : mm.getVisited()){
                double actual_x = iter.first.second * resolution + last_origin_x;
                double actual_y = iter.first.first * resolution + last_origin_y;

                double dx = origin_x - last_origin_x;
                double dy = origin_y - last_origin_y;

                actual_x += dx;
                actual_y += dy;

                unsigned int pixel_x = static_cast<unsigned int>(std::round((actual_x - origin_x) / resolution));
                unsigned int pixel_y = static_cast<unsigned int>(std::round((actual_y - origin_y) / resolution));

                transformed_visited[{pixel_y, pixel_x}] = iter.second;
            }
            mm.clearVisited();
            mm.getVisited() = transformed_visited;
        }

        void publishVisitedPoints(const nav_msgs::msg::OccupancyGrid& map) {
            //visited 맵 시각화
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

            for (const auto& itr : MapManager::getInstance().getVisited()) {
                if (itr.second == false) { // 아직 방문되지 않은 좌표만 추가
                    geometry_msgs::msg::Point p;
                    p.y = itr.first.first * map.info.resolution + map.info.origin.position.y;
                    p.x = itr.first.second * map.info.resolution + map.info.origin.position.x;
                    p.z = 0.0;
                    points_visited.points.push_back(p);
                }
            }

            visit_marker_pub_->publish(points_visited);
        }
        void publishObstaclePoints(const std::vector<std::vector<P>> obstacles,
                                   const nav_msgs::msg::OccupancyGrid& map) {
            //장애물 시각화
            visualization_msgs::msg::Marker points_visited;
            points_visited.header.frame_id = "map";
            points_visited.header.stamp = this->get_clock()->now();
            points_visited.ns = "obstacle_points";
            points_visited.id = 0;
            points_visited.type = visualization_msgs::msg::Marker::POINTS;
            points_visited.action = visualization_msgs::msg::Marker::ADD;
            points_visited.scale.x = 0.025;  // 점 크기 설정 (m 단위)
            points_visited.scale.y = 0.025;
            points_visited.color.a = 1.0;   // 점 투명도 설정
            points_visited.color.r = 0.0;
            points_visited.color.g = 1.0;
            points_visited.color.b = 0.0;

            for (const auto& obstacle : obstacles) {
                for(const auto& point : obstacle){
                    geometry_msgs::msg::Point p;
                    p.y = point.first * map.info.resolution + map.info.origin.position.y;
                    p.x = point.second * map.info.resolution + map.info.origin.position.x;
                    p.z = 0.0;
                    points_visited.points.push_back(p);
                }
            }

            obstacle_marker_pub_->publish(points_visited);
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


        void search(const nav_msgs::msg::OccupancyGrid& map){ // 언제 다시 동기화 해야할지
            //현재 지도의 데이터가 -1 인 idx의 픽셀좌표를 visited 맵에 저장
            //현재 지도의 데이터에서 -1 인 idx의 픽셀좌표를 저장
            // RCLCPP_INFO(this->get_logger(),"search");
            MapManager& mm = MapManager::getInstance();
            for(unsigned int y = 10; y < map.info.height - 10; y++){
                for(unsigned int x = 10; x < map.info.width - 10; x++){
                    unsigned int idx = pixelcoordtoidx(y, x, map.info.width);
                    if(map.data[idx] == -1){
                        mm.insertVisited(y, x, false);
                    }else{
                        mm.setVisited(y, x, true);
                    }
                }
            }
        }

        void pubGoal(const P &goal, const nav_msgs::msg::OccupancyGrid& map){

            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";

            goal_msg.pose.position.y = map.info.origin.position.y + goal.first * map.info.resolution;
            goal_msg.pose.position.x = map.info.origin.position.y + goal.second * map.info.resolution;

            RCLCPP_INFO(this->get_logger(), "Goal : (x : %f, y : %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);
            jobdone = false;
            goal_pub_->publish(goal_msg);
        }

        void setGoal(const geometry_msgs::msg::PoseStamped& robot_pose,
                     const nav_msgs::msg::OccupancyGrid& map){
            MapManager& mm = MapManager::getInstance();
            RCLCPP_INFO(this->get_logger(),"set goal");
            P near_goal = {0, 0};
            unsigned int pixel_y = (robot_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
            unsigned int pixel_x = (robot_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
            P current_pixel_coord = {pixel_y, pixel_x};
            if (pixel_y >= map.info.height || pixel_x >= map.info.width) {
                RCLCPP_ERROR(this->get_logger(), "Invalid current_pixel_coord: (%u, %u)", pixel_y, pixel_x);
                return;
            }

            double min_dist = std::numeric_limits<double>::max();
            for(auto &itr : mm.getVisited()){
                if(itr.second == false) {
                    // RCLCPP_INFO(this->get_logger(),"setgoal2");

                    unsigned int temp_idx = pixelcoordtoidx(itr.first.first, itr.first.second, map.info.width);
                    if (temp_idx >= map.data.size()) {
                        RCLCPP_ERROR(this->get_logger(), "Invalid index: %u (map size: %zu)", temp_idx, map.data.size());
                        RCLCPP_WARN(this->get_logger(), "P Coord x : %d, y : %d, width : %d", itr.first.second, itr.first.first, map.info.width);
                        itr.second = true;
                        continue;
                    }
                    if(map.data[temp_idx] != -1){
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
                pubGoal({0, 0}, map);
            }else{
                RCLCPP_INFO(this->get_logger(),"set near goal (%d, %d)", near_goal.first, near_goal.second);
                current_pixel_goal = near_goal;
                pubGoal(near_goal, map);
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

        bool i_sub_pose = false; // pose 수신하면 활성
        bool i_sub_map = false;
        bool jobdone = true; // 동작 완료 후 활성
        bool scan_init = true;
        bool first_move = false;
        bool first_scan = false;
        P current_pixel_goal;
        P last_fail_point;

        double last_origin_x = 0.0;
        double last_origin_y = 0.0;


        geometry_msgs::msg::PoseStamped last_pose_;
        nav_msgs::msg::OccupancyGrid map_;
        geometry_msgs::msg::PoseStamped goal_msg;
        sensor_msgs::msg::LaserScan::SharedPtr laser_msg;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visit_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration_map>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
