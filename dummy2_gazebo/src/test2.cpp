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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include <sensor_msgs/msg/laser_scan.hpp>

#include <queue>
#include <cmath>
#include <map>
#include <limits>
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
        std::set<P> obstacle_mask_;
        MapManager() {}

    public:
        static MapManager& getInstance(){
            static MapManager instance;
            return instance;
        }

        bool isVisited(unsigned int y, unsigned int x){
            if(visited_.find({y, x}) == visited_.end()){
                return false;
            }else{
                return visited_[{y, x}];
            }
        }

        bool isCoordinthere(unsigned int y, unsigned int x){
            return (visited_.find({y, x}) == visited_.end()) ? false : true;
        }

        bool isCoordinthere(P coord){
            return (visited_.find(coord) == visited_.end()) ? false : true;
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

        void clearObstacleMast(){
            obstacle_mask_.clear();
        }

        void addObstacleMask(double y, double x){
            obstacle_mask_.insert({y, x});
        }

        int getVisitedSize(){
            return (int)visited_.size();
        }

        std::map<P, bool>& getVisited(){
            return visited_;
        }

        std::set<P>& getObstacleMask(){
            return obstacle_mask_;
        }
};

class Exploration_map : public rclcpp::Node
{
    public:

        Exploration_map() : rclcpp::Node("Exploration_map_Node"){

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
            near_obstacle = this->create_publisher<visualization_msgs::msg::Marker>("near_obstacle", 10);
            laser_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("laser_line", 10);
            goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_points", 10);

            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Exploration_map::timer_callback, this));

        }

    protected:

        void timer_callback(){
            // RCLCPP_INFO(this->get_logger(),"System status : map = %d , pose = %d, jobdone = %d", i_sub_map, i_sub_pose, jobdone);
            if(jobdone && map_data_stable && searching){
                i_sub_pose = false;
                jobdone = false;
                searching = false;

                if(map_expanding){
                    setGoalBasedonFrontier(map_, MapManager::getInstance(), last_pose_);
                }else{
                    setGoal(last_pose_, map_);
                }


            }
            if(!map_expanding)
                excludeObstacle(laser_msg_, last_pose_, map_);
        }

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "Failed to compute pose. Failcount : %d", fail_count);
                    // fail_count++; test
                    MapManager::getInstance().setVisited(current_pixel_goal, true);
                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    reverse_count = 0;
                }
                else if(node_name_ == "FollowPath" && current_status_ == "RUNNING" && previous_status_ == "IDLE"){
                    if(map_expanding && isMapExpanding(map_)){
                        transformCoord(map_);
                        map_expanded = true;
                    }
                }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "Failed to move.");

                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "READY 2 MOVE");
                    if(!map_expanded){
                        RCLCPP_ERROR(this->get_logger(), "map not expanded");
                        fail_count++;
                    }else{
                        map_expanded = false;
                        fail_count = 0;
                    }

                    if(fail_count > 5){
                        map_expanding = false;
                        RCLCPP_WARN(this->get_logger(), "frontier sq done, nomal sq");
                    }
                    clearAround(map_, MapManager::getInstance(), last_pose_);

                    jobdone = true;
                }
            }
        }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            laser_msg_ = *msg;


            for(size_t i = 0; i < msg->ranges.size(); i++){
                if(std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i])){
                    if(std::isnan(msg->ranges[i])){
                        RCLCPP_INFO(this->get_logger(), "NAN detected");
                        double Nan_angle_rad = msg->angle_min + i * msg->angle_increment;
                        double Nan_angle_deg = Nan_angle_rad * 180 * M_1_PI;
                        RCLCPP_INFO(this->get_logger(), "NAN angle : %f", Nan_angle_deg);
                    }
                    continue;
                } // 장애물 회피 알고리즘 추가
                if(msg->ranges[i] < 0.5){
                    RCLCPP_INFO(this->get_logger(), "Obstacle detected");
                    RCLCPP_INFO(this->get_logger(), "min range : %f", msg->ranges[i]);
                    RCLCPP_INFO(this->get_logger(), "index : %ld", i);

                    if(i > 120 && i < 240){
                        RCLCPP_INFO(this->get_logger(), "Obstacle detected in front");
                        reverse_count ++;
                    }
                    else if(i > 300 && i < 60){
                        RCLCPP_INFO(this->get_logger(), "Obstacle detected in back");
                        forword_count ++;
                    }

                    if(reverse_count > 5){
                        reverse_count = 0;
                        forword_count = 0;
                        performReverse();
                    }
                    else if(forword_count > 5){
                        reverse_count = 0;
                        forword_count = 0;
                        performFoword();
                    }
                    return;
                }
            }
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }

            map_ = *msg;

            if (!map_expanding && isMapExpanding(map_)){
                transformCoord(map_);  // 지도 확장 시에만
            }


            search(map_); // map.data 에 따른 미탐색구역 visited 맵으로 동기화

            // applyMaskToMask(map_.info.origin.position.x, map_.info.origin.position.y, map_.info.resolution); // 이미 판별한 장애물을 저장
            searching = true;
        }

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            last_pose_ = *msg;
            i_sub_pose = true;
            if(!first_move && !first_scan){
                RCLCPP_INFO(this->get_logger(),"Pose first sub");
                first_move = true;
            }
            if(!map_expanding){
                publishVisitedPoints(map_); // visited 맵 시각화
            }

        }



        bool isMapExpanding(const nav_msgs::msg::OccupancyGrid& map){
            static unsigned int last_width = map.info.width;
            static unsigned int last_height = map.info.height;
            static double last_origin_x = map.info.origin.position.x;
            static double last_origin_y = map.info.origin.position.y;

            if(last_width != map.info.width || last_height != map.info.height){
                last_width = map.info.width;
                last_height = map.info.height;
                return true;
            }
            if(last_origin_x != map.info.origin.position.x || last_origin_y != map.info.origin.position.y){
                last_origin_x = map.info.origin.position.x;
                last_origin_y = map.info.origin.position.y;
                return true;
            }
            return false;
        }

        void clearAround(const nav_msgs::msg::OccupancyGrid& map,
                         MapManager& mm,
                         const geometry_msgs::msg::PoseStamped& robot_pose){
            unsigned int pixel_y = (robot_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
            unsigned int pixel_x = (robot_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
            unsigned int clear_range = 10;
            for(unsigned int y = pixel_y - clear_range ; y < pixel_y + clear_range; y++){
                for(unsigned int x = pixel_x - clear_range ; x < pixel_x + clear_range; x++){
                    if(y >= map.info.height || x >= map.info.width || x < 5 || y < 5){
                        continue;
                    }
                    mm.markVisited({y, x});
                }
            }
        }

        std::vector<std::vector<P>> GetObstacleShapes(const sensor_msgs::msg::LaserScan& laser_scan,
                                                      const geometry_msgs::msg::PoseStamped& robot_pose,
                                                      const nav_msgs::msg::OccupancyGrid& map){
            std::vector<std::vector<P>> obstacle;
            std::vector<P> current_obs;
            // RCLCPP_INFO(this->get_logger(),"scan obstacle shapes");

            double robot_x = robot_pose.pose.position.x;
            double robot_y = robot_pose.pose.position.y;
            double prev_dist = 0.0;
            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            double width = map.info.width;
            double height = map.info.height;
            double resolution = map.info.resolution;
            // bool last_sign = false;

            tf2::Quaternion tf2_quat;
            tf2::fromMsg(robot_pose.pose.orientation, tf2_quat);
            double theta = tf2::getYaw(tf2_quat);

            for(size_t i = 0; i < laser_scan.ranges.size(); i++){
                double laser_dist = laser_scan.ranges[i];
                double laser_angle = laser_scan.angle_min + i * laser_scan.angle_increment;

                if (std::isnan(laser_dist) || std::isinf(laser_dist) ||
                    laser_dist < laser_scan.range_min || laser_dist > laser_scan.range_max) {
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


                unsigned int pixel_y = (obs_y_map - origin_y) / resolution;
                unsigned int pixel_x = (obs_x_map - origin_x) / resolution;
                if (pixel_x >= width || pixel_y >= height) {
                    continue; // 무효한 좌표는 무시
                }
                MapManager::getInstance().addObstacleMask(pixel_y, pixel_x);

                double angle_diff = laser_angle - (laser_scan.angle_min + (i - 1) * laser_scan.angle_increment);
                double sign_distance_diff = laser_dist - prev_dist;
                double distance_diff = fabs(sign_distance_diff);
                // bool sign = (sign_distance_diff > 0) ? true : false;



                if (!current_obs.empty() && (distance_diff > 0.5 || angle_diff > 0.1)) {
                    obstacle.push_back(current_obs);
                    current_obs.clear();
                }

                current_obs.emplace_back(pixel_y, pixel_x);
                prev_dist = laser_dist;

                // wall.insert({pixel_y * resolution + origin_y, pixel_x * resolution + origin_x});

            }
            if(!current_obs.empty()){
                obstacle.push_back(current_obs);
            }
            // publishObstaclePoints(wall);
            return obstacle;
        }

        void ObstacleFloodFill(const std::vector<std::vector<P>>& obstacles, // make_square 로
                       const nav_msgs::msg::OccupancyGrid& map){
            // RCLCPP_INFO(this->get_logger(),"flood fill");
            // int temp_size = obstacles.size();
            MapManager& mm = MapManager::getInstance();
            std::set<P> obstacle_mask = mm.getObstacleMask();
            // RCLCPP_INFO(this->get_logger(),"obstacles size : %d ",temp_size);
            for(const auto& obstacle : obstacles){
                if(obstacle.size() < 3) continue;
                std::queue<P> q;

                unsigned int min_x = std::numeric_limits<unsigned int>::max();
                unsigned int min_y = std::numeric_limits<unsigned int>::max();
                unsigned int max_x = 0;
                unsigned int max_y = 0;

                std::set<P> obstacle_set(obstacle.begin(), obstacle.end());

                for (const auto& point : obstacle) {
                    min_x = std::min(min_x, point.second);
                    min_y = std::min(min_y, point.first);
                    max_x = std::max(max_x, point.second);
                    max_y = std::max(max_y, point.first);
                }
                unsigned int seed_x = 0, seed_y = 0;
                bool inside = false;

                for(unsigned int y = min_y; y <= max_y; y++){
                    int crossings = 0;
                    for(unsigned int x = min_x; x <= max_x; x++){
                        if(obstacle_set.count({y, x})){
                            crossings++;
                        }else if(crossings % 2 == 1 && !obstacle_mask.count({y, x})){
                            seed_y = y;
                            seed_x = x;
                            inside = true;
                            break;
                        }
                    }
                    if (inside) break;
                }
                if(!inside) continue;

                q.push({seed_y, seed_x});
                unsigned int temp_offset = 10;
                while(!q.empty()){
                    // int temp = q.size();
                    // RCLCPP_INFO(this->get_logger(), "q.size : %d", temp);
                    auto [y, x] = q.front();
                    q.pop();
                    if(x < min_x - temp_offset || x > max_x + temp_offset || y < min_y - temp_offset || y > max_y + temp_offset){
                        // RCLCPP_INFO(this->get_logger(),"out of range");
                        continue;
                    }
                    if(y >= map.info.height || x >= map.info.width){
                        // RCLCPP_INFO(this->get_logger(),"out of range2");
                        continue;
                    }
                    if(obstacle_mask.count({y, x})){
                        continue;
                    }
                    if(mm.isVisited(y, x)){
                        // RCLCPP_INFO(this->get_logger(),"already visited");
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

        void excludeObstacle(const sensor_msgs::msg::LaserScan& laser_scan,
                             const geometry_msgs::msg::PoseStamped& robot_pose,
                            //  std::map<P, bool>& visited,
                             const nav_msgs::msg::OccupancyGrid& map){
            // RCLCPP_INFO(this->get_logger(),"scan obstacle");

            auto obstacles = GetObstacleShapes(laser_scan, robot_pose, map);

            ObstacleFloodFill(obstacles, map);

            // RCLCPP_INFO(this->get_logger(),"scan obstacle done");

        }



        void transformCoord(const nav_msgs::msg::OccupancyGrid& map){
            std::map<P, bool> transformed_visited;
            std::set<P> transformed_obstacle_mask;
            RCLCPP_INFO(this->get_logger(),"TRANSFORM COORD");
            static double last_origin_x = map.info.origin.position.x;
            static double last_origin_y = map.info.origin.position.y;
            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            double resolution = map.info.resolution;
            if(std::fabs(last_origin_x - origin_x) < 0.01 || std::fabs(last_origin_y == origin_y) < 0.01){return;}
            MapManager& mm = MapManager::getInstance();

            auto transformPoints = [&](const P& p) -> P{
                double actual_x = p.second * resolution + last_origin_x;
                double actual_y = p.first * resolution + last_origin_y;

                double dx = origin_x - last_origin_x;
                double dy = origin_y - last_origin_y;

                actual_x += dx;
                actual_y += dy;

                unsigned int pixel_x = static_cast<unsigned int>(std::round((actual_x - origin_x) / resolution));
                unsigned int pixel_y = static_cast<unsigned int>(std::round((actual_y - origin_y) / resolution));

                return {pixel_y, pixel_x};
            };

            for(const auto& iter : mm.getVisited()){
                transformed_visited[transformPoints(iter.first)] = iter.second;
            }

            for(const auto& p : mm.getObstacleMask()){
                transformed_obstacle_mask.insert(transformPoints(p));
            }

            last_origin_x = map.info.origin.position.x;
            last_origin_y = map.info.origin.position.y;
            mm.clearVisited();
            mm.clearObstacleMast();
            mm.getVisited() = transformed_visited;
            mm.getObstacleMask() = transformed_obstacle_mask;
        }

        void publishVisitedPoints(const nav_msgs::msg::OccupancyGrid& map) {
            // 미탐색 좌표 시각화
            visualization_msgs::msg::Marker points_unexplored;
            points_unexplored.header.frame_id = "map";
            points_unexplored.header.stamp = this->get_clock()->now();
            points_unexplored.ns = "unexplored_points";
            points_unexplored.id = 0;
            points_unexplored.type = visualization_msgs::msg::Marker::POINTS;
            points_unexplored.action = visualization_msgs::msg::Marker::ADD;
            points_unexplored.scale.x = 0.05;
            points_unexplored.scale.y = 0.05;
            points_unexplored.color.a = 1.0;
            points_unexplored.color.r = 1.0; // 빨간색
            points_unexplored.color.g = 0.0;

            // 탐색 완료된 좌표 시각화
            visualization_msgs::msg::Marker points_explored;
            points_explored.header.frame_id = "map";
            points_explored.header.stamp = this->get_clock()->now();
            points_explored.ns = "explored_points";
            points_explored.id = 1;
            points_explored.type = visualization_msgs::msg::Marker::POINTS;
            points_explored.action = visualization_msgs::msg::Marker::ADD;
            points_explored.scale.x = 0.05;
            points_explored.scale.y = 0.05;
            points_explored.color.a = 1.0;
            points_explored.color.r = 0.0;
            points_explored.color.g = 1.0; // 초록색

            // 점 생성 및 추가
            for (const auto& itr : MapManager::getInstance().getVisited()) {
                geometry_msgs::msg::Point p;
                p.y = itr.first.first * map.info.resolution + map.info.origin.position.y;
                p.x = itr.first.second * map.info.resolution + map.info.origin.position.x;
                p.z = 0.0;

                if (!itr.second) { // 미탐색 좌표
                    points_unexplored.points.push_back(p);
                } else { // 탐색 완료 좌표
                    points_explored.points.push_back(p);
                }
            }

            // 퍼블리시
            visit_marker_pub_->publish(points_unexplored);
            // visit_marker_pub_->publish(points_explored);
        }
        void publishscanPoints(const sensor_msgs::msg::LaserScan& laser_scan, //목표 방향 시각화
                               const int& index, const double& theta){
            //해당 인덱스의 레이저 스캔 좌표 시각화
            double range = laser_scan.ranges[index];


            visualization_msgs::msg::Marker laser_line;
            laser_line.header.frame_id = "map";
            laser_line.header.stamp = this->get_clock()->now();
            laser_line.ns = "laser_line";
            laser_line.id = 0;
            laser_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            laser_line.action = visualization_msgs::msg::Marker::ADD;
            laser_line.scale.x = 0.05;
            laser_line.color.a = 1.0;
            laser_line.color.r = 1.0;
            laser_line.color.g = 0.0;
            laser_line.color.b = 0.0;


            //tf 변환
            try{
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "map", "base_link", rclcpp::Time(0)
                );
                geometry_msgs::msg::PointStamped base_link_origin;
                base_link_origin.header.frame_id = "base_link";
                base_link_origin.header.stamp = this->get_clock()->now();
                base_link_origin.point.x = 0.215;
                base_link_origin.point.y = 0.0;
                base_link_origin.point.z = 0.0;

                geometry_msgs::msg::PointStamped map_origin;
                tf2::doTransform(base_link_origin, map_origin, transform);

                laser_line.points.clear();
                laser_line.points.push_back(map_origin.point);

                geometry_msgs::msg::Point base_link_point;
                base_link_point.x = cos(theta) * range;
                base_link_point.y = sin(theta) * range;
                base_link_point.z = 0.0;

                geometry_msgs::msg::PointStamped base_link_point_stamped;
                base_link_point_stamped.header.frame_id = "base_link";
                base_link_point_stamped.header.stamp = this->get_clock()->now();
                base_link_point_stamped.point = base_link_point;

                geometry_msgs::msg::PointStamped map_point;
                tf2::doTransform(base_link_point_stamped, map_point, transform);
                laser_line.points.push_back(map_point.point);

            }catch(tf2::TransformException &ex){
                RCLCPP_ERROR(this->get_logger(), "Transform error : %s", ex.what());
                return;
            }

            laser_marker_pub_->publish(laser_line);
        }

        void publishGoalPoints(const geometry_msgs::msg::PoseStamped& goal) {
            //장애물 시각화
            visualization_msgs::msg::Marker goal_point;
            goal_point.header.frame_id = "map";
            goal_point.header.stamp = this->get_clock()->now();
            goal_point.ns = "goal_points";
            goal_point.id = 0;
            goal_point.type = visualization_msgs::msg::Marker::POINTS;
            goal_point.action = visualization_msgs::msg::Marker::ADD;
            goal_point.scale.x = 0.2;  // 점 크기 설정 (m 단위)
            goal_point.scale.y = 0.2;
            goal_point.color.a = 1.0;   // 점 투명도 설정
            goal_point.color.r = 0.0;
            goal_point.color.g = 1.0;
            goal_point.color.b = 0.0;

            geometry_msgs::msg::Point p;
            p.y = goal.pose.position.y;
            p.x = goal.pose.position.x;
            p.z = 0.0;
            goal_point.points.push_back(p);


            goal_marker_pub_->publish(goal_point);
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

        void performFoword() {
            geometry_msgs::msg::Twist forward_msg;
            forward_msg.linear.x = 0.2;  // 양수 값으로 설정하여 후진
            forward_msg.angular.z = 0.0;  // 회전 없이 직진

            for(int i = 0; i < 10; i++){
                cmd_pub_->publish(forward_msg);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
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


            static std::vector<int8_t> last_map_data;

            if(last_map_data.empty()){
                last_map_data = map.data;
            }

            unsigned int height = map.info.height;
            unsigned int width = map.info.width;


            for(unsigned int y = 10; y < height - 10; y++){
                for(unsigned int x = 10; x < width - 10; x++){
                    unsigned int idx = pixelcoordtoidx(y, x, width);
                    if(last_map_data[idx] != map.data[idx]){
                        map_data_stable = false;
                        if(map.data[idx] == -1){
                            mm.insertVisited(y, x, false);
                        }else{
                            mm.setVisited(y, x, true);
                        }
                    }else{
                        map_data_stable = true;
                    }
                }
            }

            for(unsigned int y = 0; y <= 10; y++){
                for(unsigned int x = 0; x <= 10; x++){
                    mm.markVisited({y,x});
                }
            }

            for(unsigned int y = height - 10; y <= height ; y++){
                for(unsigned int x = width - 10; x <= width ; x++){
                    mm.markVisited({y, x});
                }
            }
            last_map_data = map.data;
            // RCLCPP_INFO(this->get_logger(),"search done");
        }

        void isOverObstacle(const sensor_msgs::msg::LaserScan& laser_scan,
                            const geometry_msgs::msg::PoseStamped& robot_pose,
                            const geometry_msgs::msg::PoseStamped& goal_pose){
            double sensor_offset_x = 0.27;
            double sensor_offset_y = 0.10;

            double robot_x = robot_pose.pose.position.x + sensor_offset_x;
            double robot_y = robot_pose.pose.position.y + sensor_offset_y;
            double goal_x = goal_pose.pose.position.x;
            double goal_y = goal_pose.pose.position.y;

            tf2::Quaternion q;
            tf2::fromMsg(robot_pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            double dx = goal_x - robot_x;
            double dy = goal_y - robot_y;

            double rotated_x = cos(-yaw) * dx - sin(-yaw) * dy;
            double rotated_y = sin(-yaw) * dx + cos(-yaw) * dy;

            double theta_with_robot_goal = std::atan2(rotated_y, rotated_x); // 로봇 기준 목표 방향 (rad)

            if(theta_with_robot_goal < 0){
                theta_with_robot_goal += 2 * M_PI; // 0 ~ 360 반시계방향, 정면이 0도
            }
            int theta_with_robot_goal_deg = theta_with_robot_goal * 180 * M_1_PI;
            size_t index = theta_with_robot_goal_deg;
            index = (index + 180) % 360; // 로봇기준에서 180도 회전됨, 정면 index = 180
            RCLCPP_WARN(this->get_logger(), "theta_with_robot_goal : %f ", theta_with_robot_goal * 180 * M_1_PI);

            publishscanPoints(laser_scan, index, theta_with_robot_goal);
            RCLCPP_INFO(this->get_logger(), "index : %ld", index);

            double range_at_angle = laser_scan.ranges[index];

            double dist_goal = std::hypot(dx, dy);
            RCLCPP_WARN(this->get_logger(), "dist_goal : %f ", dist_goal);
            RCLCPP_WARN(this->get_logger(), "range_at_angle : %f ", range_at_angle);

            if (std::isinf(range_at_angle) || std::isnan(range_at_angle)){
                RCLCPP_INFO(this->get_logger(), "No valid data at this angle.");
                publishGoalPoints(goal_msg);
                return;
            }
            double threshold = -0.1;
            if(dist_goal - range_at_angle > threshold){ // 목표지점이 장애물 너머일 경우
                RCLCPP_INFO(this->get_logger(), "is over obstacle");

                setGoalbesideObstacle(laser_scan, robot_pose, index, dist_goal);

                // isOverObstacle(laser_scan, robot_pose, goal_msg);

            }else{ // 아닐경우
                RCLCPP_ERROR(this->get_logger(), "is not over obstacle, Dg : %f , RaA : %f", dist_goal, range_at_angle);
                publishGoalPoints(goal_msg);
            }

        }

        void setGoalbesideObstacle(const sensor_msgs::msg::LaserScan& laser_scan,
                                   const geometry_msgs::msg::PoseStamped& robot_pose,
                                   const int& angle, double dist_goal // 인덱스가 로봇기준 방향
                                   ){
            double robot_x = robot_pose.pose.position.x;
            double robot_y = robot_pose.pose.position.y;

            double another_goal_x = 0.0;
            double another_goal_y = 0.0;
            double laser_angle = 0.0; // deg

            int index = angle;
            if(angle == 1 || angle == 359){
                index = 0;
            }

            double prev_dist_pos = laser_scan.ranges[index];
            double prev_dist_neg = laser_scan.ranges[index];



            for(int pos_dir = index + 1, neg_dir = index - 1;
                pos_dir  <= 360 && neg_dir >= 0;
                pos_dir++, neg_dir--){

                if(neg_dir == -1){
                    neg_dir = 359;
                }
                if (pos_dir == 360) {
                    pos_dir = 0;
                }

                double range_at_dist_pos = laser_scan.ranges[pos_dir];
                double range_at_dist_neg = laser_scan.ranges[neg_dir];

                double dist_diff_pos = std::abs(range_at_dist_pos - prev_dist_pos);
                double dist_diff_neg = std::abs(range_at_dist_neg - prev_dist_neg);

                int offset = 5;
                if(dist_diff_pos > 0.5){
                    laser_angle = pos_dir - 180 + offset;
                    RCLCPP_INFO(this->get_logger(),"positive side goal set");
                    break;
                }

                if(dist_diff_neg > 0.5){
                    laser_angle = neg_dir - 180 - offset;
                    RCLCPP_INFO(this->get_logger(),"negative side goal set");
                    break;
                }

                prev_dist_pos = range_at_dist_pos;
                prev_dist_neg = range_at_dist_neg;
            }

            double min_margin = 0.3;
            double max_margin = 1.5;
            double k = 0.2;
            double safe_margin = min_margin + k * dist_goal;
            safe_margin = std::clamp(safe_margin, min_margin, max_margin);

            another_goal_x = (dist_goal + safe_margin) * cos(laser_angle * M_PI / 180.0) + robot_x;
            another_goal_y = (dist_goal + safe_margin) * sin(laser_angle * M_PI / 180.0) + robot_y;

            double another_dist = std::hypot(another_goal_x - robot_x, another_goal_y - robot_y);
            if(another_dist > 20.0){
                RCLCPP_ERROR(this->get_logger(), "Too far..");
                return; //temp
            }

            RCLCPP_WARN(this->get_logger(), "Laser Angle: %f degree", laser_angle);
            RCLCPP_WARN(this->get_logger(), "dist_goal : %f ", dist_goal);
            RCLCPP_WARN(this->get_logger(), "robot coord: (x: %f, y: %f)", robot_x, robot_y);
            RCLCPP_WARN(this->get_logger(), "Cos(%f) = %f, Sin(%f) = %f", laser_angle, cos(laser_angle), laser_angle, sin(laser_angle));
            RCLCPP_WARN(this->get_logger(), "Calculated Goal: (x: %f, y: %f)", another_goal_x, another_goal_y);

            goal_msg.pose.position.x = another_goal_x;
            goal_msg.pose.position.y = another_goal_y;
            RCLCPP_INFO(this->get_logger(), "Goal : (x : %f, y : %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);

            publishGoalPoints(goal_msg);
        }

        void pubGoal( P goal, const nav_msgs::msg::OccupancyGrid& map){

            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";
            // RCLCPP_INFO(this->get_logger(), "Goal_pixel : (x : %d, y : %d)", goal.second, goal.first);
            goal_msg.pose.position.y = map.info.origin.position.y + goal.first * map.info.resolution;
            goal_msg.pose.position.x = map.info.origin.position.x + goal.second * map.info.resolution;
            // goal_msg.pose.position.x = 5.0;
            // goal_msg.pose.position.y = 0.0;
            if(!map_expanding){
                isOverObstacle(laser_msg_, last_pose_, goal_msg);
            }

            RCLCPP_INFO(this->get_logger(), "Goal : (x : %f, y : %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);
            publishGoalPoints(goal_msg);
            goal_pub_->publish(goal_msg);
        }

        std::pair<int,int> findNearGear(const nav_msgs::msg::OccupancyGrid& map, const P& goal){
            unsigned int map_width = (unsigned int)map.info.width;
            unsigned int map_height = (unsigned int)map.info.height;
            for(int dy = -10; dy <= 10; dy++){
                for(int dx = -10; dx <= 10; dx++){
                    unsigned int check_y = goal.first + dy;
                    unsigned int check_x = goal.second + dx;
                    if(check_y < map_height && check_x < map_width &&
                       map.data[pixelcoordtoidx(check_y, check_x, map_width)] == 100){
                        return {dy, dx};
                    }
                }
            }
            return {0, 0};
        }

        void setGoalBasedonFrontier(const nav_msgs::msg::OccupancyGrid& map, MapManager& mm, const geometry_msgs::msg::PoseStamped& robot_pose){
            const int directions[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, -1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
            RCLCPP_INFO(this->get_logger(),"set goal based on frontier");
            unsigned int map_width = (unsigned int)map.info.width;
            unsigned int map_height = (unsigned int)map.info.height;

            std::vector<P> frontier;

            for(unsigned int x = 0; x < map_width; x++){
                for(unsigned int y = 0; y < map_height; y++){
                    if(map.data[pixelcoordtoidx(y, x, map_width)] != 0 || mm.isVisited(y, x)){
                        // mm.markVisited({y, x});
                        continue;
                    }
                    for(const auto& dir : directions){
                        unsigned int nx = x + dir[0];
                        unsigned int ny = y + dir[1];
                        if(nx > 10 && ny > 10 && nx < map_width - 10 && ny < map_height - 10){
                            if(map.data[pixelcoordtoidx(ny, nx, map_width)] == 100 || mm.isVisited(ny, nx)){
                                frontier.push_back({y, x});
                                break;
                            }
                        }
                    }
                }
            }
            if(!frontier.empty()){
                int size = (int)frontier.size();
                RCLCPP_INFO(this->get_logger(),"frontier size : %d", size);
                P goal;
                goal.first = std::clamp(goal.first, (unsigned int)10, (unsigned int)map.info.height - 10);
                goal.second = std::clamp(goal.second, (unsigned int)10, (unsigned int)map.info.width - 10);
                unsigned int pixel_y = (robot_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
                unsigned int pixel_x = (robot_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
                sort(frontier.begin(), frontier.end(), [&](const P& a, const P& b){
                    return distance(a, {pixel_y, pixel_x}) > distance(b, {pixel_y, pixel_x});
                });
                double min_dist = 150.0; //temp
                double max_dist = 250.0;
                static P last_goal;

                int idx = 0;

                for(; idx < size; idx++){
                    goal = frontier[idx];
                    if(last_goal == goal){
                        RCLCPP_INFO(this->get_logger(),"Same goal");
                        continue;
                    }

                    double dist = distance(goal, {pixel_y, pixel_x});
                    if(dist > min_dist && dist < max_dist){
                        auto offset = findNearGear(map, goal);
                        RCLCPP_INFO(this->get_logger(),"Goal found, offset : (%d, %d)", offset.first, offset.second);
                        RCLCPP_INFO(this->get_logger(),"Goal : (%d, %d)", goal.first, goal.second);
                        RCLCPP_INFO(this->get_logger(),"robot : (%d, %d)", pixel_y, pixel_x);
                        RCLCPP_INFO(this->get_logger(),"dist : %f", dist);
                        last_goal = goal;

                        if(offset.first == 0 || offset.second == 0){
                            if(offset.first == 0){
                                goal.first += 10;
                                if(offset.second != 0){
                                    goal.second += (-offset.second) / offset.second * 20;
                                }
                            }
                            if(offset.second == 0){
                                goal.second += 10;
                                if(offset.first != 0){
                                    goal.first += (-offset.first) / offset.first * 20;
                                }
                            }
                        }else{
                            goal.first += (-offset.first) / offset.first * 30;
                            goal.second += (-offset.second) / offset.second * 30;
                        }

                        RCLCPP_INFO(this->get_logger(),"Fix Goal : (%d, %d)", goal.first, goal.second);

                        break;
                    }
                    if(idx == size - 1){
                        RCLCPP_INFO(this->get_logger(),"dist : %f", dist);
                        RCLCPP_INFO(this->get_logger(),"All goal is gear");
                        goal = frontier[0];
                        break;
                    }
                }
                RCLCPP_INFO(this->get_logger(),"frontier end");
                pubGoal(goal, map);
            }
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
                // RCLCPP_ERROR(this->get_logger(), "Invalid current_pixel_coord: (%u, %u)", pixel_y, pixel_x);
                return;
            }
            double min_dist = std::numeric_limits<double>::max();
            for(auto &itr : mm.getVisited()){
                if(itr.second == false) {
                    // RCLCPP_INFO(this->get_logger(),"setgoal2");
                    if(itr.first.first >= map.info.height || itr.first.second >= map.info.width){ continue; }
                    unsigned int temp_idx = pixelcoordtoidx(itr.first.first, itr.first.second, map.info.width);
                    if (temp_idx >= map.data.size()) {
                        // RCLCPP_ERROR(this->get_logger(), "Invalid index: %u (map size: %zu)", temp_idx, map.data.size());
                        // RCLCPP_WARN(this->get_logger(), "P Coord x : %d, y : %d, width : %d", itr.first.second, itr.first.first, map.info.width);
                        itr.second = true;
                        continue;
                    }
                    if(map.data[temp_idx] != -1){
                        itr.second = true;
                        // RCLCPP_INFO(this->get_logger(),"Already gone");
                        continue;
                    }
                    // RCLCPP_INFO(this->get_logger(),"visited[itr] = %d, map_->data[idx] = %d", itr.second, map_->data[temp_idx]);
                    // RCLCPP_INFO(this->get_logger(), "current coord = ( %d, %d )", itr.first.first, itr.first.second);
                    double dist = distance(itr.first, current_pixel_coord);
                    if( dist < min_dist){
                        min_dist = dist;
                        near_goal = itr.first;
                    }
                }
            }
            if(near_goal == P{0, 0}){
                // RCLCPP_INFO(this->get_logger(),"All point visited");
                pubGoal({0, 0}, map);
            }else{
                // RCLCPP_INFO(this->get_logger(),"current position (%f, %f)", robot_pose.pose.position.x, robot_pose.pose.position.y);
                // RCLCPP_INFO(this->get_logger(),"current pixel position (%d, %d)", pixel_x, pixel_y);
                // RCLCPP_INFO(this->get_logger(),"set near goal (%d, %d), dist : %f", near_goal.first, near_goal.second, min_dist);
                current_pixel_goal = near_goal;
                pubGoal(near_goal, map);
            }
        }


        ///C1 > C2
        double distance(P C1, P C2){
            return std::hypot(((int)C1.first - (int)C2.first), ((int)C1.second - (int)C2.second));
        }

        unsigned int pixelcoordtoidx(unsigned int y, unsigned int x, uint32_t width){
            return (y * width + x);
        }



        int fail_count = 0;
        int reverse_count = 0;
        int forword_count = 0;

        bool i_sub_pose = false; // pose 수신하면 활성
        bool i_sub_map = false;
        bool jobdone = true; // 동작 완료 후 활성 , 이동 없는 테스트는 비활성
        bool scan_init = false;
        bool searching = false;
        bool map_data_stable = false;
        bool map_expanding = true;
        bool map_expanded = false;
        bool first_move = false;
        bool first_scan = false;
        P current_pixel_goal;
        P last_fail_point;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


        geometry_msgs::msg::PoseStamped last_pose_;
        nav_msgs::msg::OccupancyGrid map_;
        geometry_msgs::msg::PoseStamped goal_msg;
        sensor_msgs::msg::LaserScan laser_msg_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visit_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr near_obstacle;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr laser_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration_map>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
