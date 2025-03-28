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

class MapManager{
    private:

        MapManager(const MapManager&) = delete;
        MapManager& operator=(const MapManager&) = delete;
        MapManager(MapManager&&) = delete;
        MapManager& operator=(MapManager&&) = delete;

        std::map<P, bool> visited_;
        std::set<P> singleton_set_;
        unsigned int min_x_ = std::numeric_limits<unsigned int>::max();
        unsigned int min_y_ = std::numeric_limits<unsigned int>::max();
        unsigned int max_x_ = 0;
        unsigned int max_y_ = 0;
        std::tuple<unsigned int*, unsigned int*, unsigned int*, unsigned int*> wall_ = {&min_y_, &min_x_, &max_y_, &max_x_};
        MapManager(){}

    public:
        static MapManager& getInstance(){
            static MapManager instance;
            return instance;
        }

        void updateWall(unsigned int min_y, unsigned int min_x, unsigned int max_y, unsigned int max_x){

            min_y_ = std::min(min_y_, min_y);
            min_x_ = std::min(min_x_, min_x);
            max_y_ = std::max(max_y_, max_y);
            max_x_ = std::max(max_x_, max_x);
        }

        std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> getWall(){
            std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> wall = {min_y_, min_x_, max_y_, max_x_};
            return wall;
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

        void clearSingletonset(){
            singleton_set_.clear();
        }

        void addOSingletonset(double y, double x){
            singleton_set_.insert({y, x});
        }

        int getVisitedSize(){
            return (int)visited_.size();
        }

        std::map<P, bool>& getVisited(){
            return visited_;
        }

        std::set<P>& getSingletonset(){
            return singleton_set_;
        }
};

class PoseManager{
    private:
        PoseManager(const PoseManager&) = delete;
        PoseManager& operator=(const PoseManager&) = delete;
        PoseManager(PoseManager&&) = delete;
        PoseManager& operator=(PoseManager&&) = delete;

        geometry_msgs::msg::PoseStamped last_pose_;
        std::mutex last_pose_mutex_;
        geometry_msgs::msg::PoseStamped goal_pose_;
        PoseManager(){}

    public:
        static PoseManager& getInstance(){
            static PoseManager instance;
            return instance;
        }

        void setLastPose(const geometry_msgs::msg::PoseStamped& pose){
            std::lock_guard<std::mutex> lock(last_pose_mutex_);
            last_pose_ = pose;
        }

        void setGoalPose(const geometry_msgs::msg::PoseStamped& pose){
            goal_pose_ = pose;
        }

        double getRobotdirection(){ //radian
            tf2::Quaternion q;
            tf2::fromMsg(last_pose_.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            double theta = yaw;

            return theta;
        }

        bool isitstuckhere(const geometry_msgs::msg::PoseStamped& pose){
            if(last_pose_.pose.position.x - pose.pose.position.x < 0.1 &&
                last_pose_.pose.position.y - pose.pose.position.y < 0.1)
            {
                return true;
            }
            return false;
        }


        double getDirectionWithRobotGoal(const double& sensor_offset_x = 0.27, const double& sensor_offset_y = 0.10){
            double robot_x = last_pose_.pose.position.x + sensor_offset_x;
            double robot_y = last_pose_.pose.position.y + sensor_offset_y;
            double goal_x = goal_pose_.pose.position.x;
            double goal_y = goal_pose_.pose.position.y;

            double dx = goal_x - robot_x;
            double dy = goal_y - robot_y;

            return atan2(dy, dx);
        }

        double getdistancewithRobotGoal(){
            double dx = goal_pose_.pose.position.x - last_pose_.pose.position.x;
            double dy = goal_pose_.pose.position.y - last_pose_.pose.position.y;
            return sqrt(dx * dx + dy * dy);
        }

        geometry_msgs::msg::PoseStamped& getLastPose(){
            std::lock_guard<std::mutex> lock(last_pose_mutex_);
            return last_pose_;
        }

        geometry_msgs::msg::PoseStamped& getGoalPose(){
            return goal_pose_;
        }
};

class LaserManager{
    private:
        LaserManager(const LaserManager&) = delete;
        LaserManager& operator=(const LaserManager&) = delete;
        LaserManager(LaserManager&&) = delete;
        LaserManager& operator=(LaserManager&&) = delete;

        sensor_msgs::msg::LaserScan laser_msg_;
        std::mutex laser_msg_mutex_;
        LaserManager(){}
    public:
        static LaserManager& getInstance(){
            static LaserManager instance;
            return instance;
        }
        sensor_msgs::msg::LaserScan& getLaserMsg(){
            std::lock_guard<std::mutex> lock(laser_msg_mutex_);
            return laser_msg_;
        }
        void setLaserMsg(const sensor_msgs::msg::LaserScan& msg){
            std::lock_guard<std::mutex> lock(laser_msg_mutex_);
            laser_msg_ = msg;
        }

};

class Utility{
    public:
        static double distance(P C1, P C2){
            return std::hypot(((int)C1.first - (int)C2.first), ((int)C1.second - (int)C2.second));
        }

        static unsigned int pixelcoordtoidx(unsigned int y, unsigned int x, uint32_t width){
            return (y * width + x);
        }

        static bool isSameTuple(std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> a,
                                 std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> b){
            int threshold = 5;
            if( std::abs((int)std::get<0>(a) - (int)std::get<0>(b)) > threshold ||
                std::abs((int)std::get<1>(a) - (int)std::get<1>(b)) > threshold ||
                std::abs((int)std::get<2>(a) - (int)std::get<2>(b)) > threshold ||
                std::abs((int)std::get<3>(a) - (int)std::get<3>(b)) > threshold){
                    return false;
               }
            return true;
        }
    };

class QuadtreeNode {
    public:
        QuadtreeNode(unsigned int min_y, unsigned int min_x, unsigned int max_y, unsigned int max_x)
            : min_y_(min_y), min_x_(min_x), max_y_(max_y), max_x_(max_x){}

        double evalute(const nav_msgs::msg::OccupancyGrid& map){
            double occupancy_rate = calculateOccupancyRate(map);
            double unexplored_ratio = calculateUnexploredRatio(map);
            double distance = calculateDisance(map, PoseManager::getInstance().getLastPose());
            // double visited_ratio = calculateVisitedRatio();

            // double visited_penalty = 0.0;
            // MapManager& mm = MapManager::getInstance();
            // unsigned int mid_y = (min_y_ + max_y_) / 2;
            // unsigned int mid_x = (min_x_ + max_x_) / 2;
            // if(mm.isVisited(mid_y, mid_x)){
            //     visited_penalty = 0.8;
            // }

            double score = unexplored_ratio * 0.7 + (1 - occupancy_rate) * 0.1 + distance * 0.2 ;// - visited_ratio * 0.1 - visited_penalty;

            return score;
        }
        bool merge(const nav_msgs::msg::OccupancyGrid& map){
            if(children_[0] == nullptr) return false; // 자식노드가 없으면 합칠 수 없음

            for(int i = 0; i < 4; i++){ // 자식노드가 모두 leaf 노드가 아니면 합칠 수 없음
                if(children_[i]->children(0) != nullptr) return false;
            }

            double unexplore_threshold = 0.1;
            for(int i = 0; i < 4; i++){
                if(children_[i]->calculateUnexploredRatio(map) > unexplore_threshold) return false;
            }
            for(int i = 0; i < 4; i++){
                children_[i] = nullptr;
            }
            return true;
        }

        bool split(const nav_msgs::msg::OccupancyGrid& map){
            double unexplore_rate = calculateUnexploredRatio(map);
            // RCLCPP_INFO(rclcpp::get_logger("Exploration_map_Node"), "unexplore_rate : %f", unexplore_rate);
            // RCLCPP_INFO(rclcpp::get_logger("Exploration_map_Node"), "max_x_ - min_x_: %d", max_x_ - min_x_);
            // RCLCPP_INFO(rclcpp::get_logger("Exploration_map_Node"), "max_y_ - min_y_: %d", max_y_ - min_y_);
            if(unexplore_rate > 0.1 && (max_x_ - min_x_ > 5) && (max_y_ - min_y_ > 5)){
                unsigned int mid_y = (min_y_ + max_y_) / 2;
                unsigned int mid_x = (min_x_ + max_x_) / 2;

                children_[0] = std::make_shared<QuadtreeNode>(min_y_, min_x_, mid_y, mid_x);
                children_[1] = std::make_shared<QuadtreeNode>(min_y_, mid_x, mid_y, max_x_);
                children_[2] = std::make_shared<QuadtreeNode>(mid_y, min_x_, max_y_, mid_x);
                children_[3] = std::make_shared<QuadtreeNode>(mid_y, mid_x, max_y_, max_x_);

                for(int i = 0; i < 4; i++){
                    children_[i]->split(map);
                }
                return true;
            }else{
                return merge(map);
            }
            return false;
        }

        double calculateOccupancyRate(const nav_msgs::msg::OccupancyGrid& map){
            int occupied_count = 0;
            int total_count = 0;
            for(unsigned int y = min_y_; y < max_y_; y++){
                for(unsigned int x = min_x_; x < max_x_; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, map.info.width);
                    if(map.data[idx] == 100){
                        occupied_count++;
                    }
                    total_count++;
                }
            }
            return (double)occupied_count / total_count;
        }

        double calculateUnexploredRatio(const nav_msgs::msg::OccupancyGrid& map){
            int unexplored_count = 0;
            int total_count = 0;
            for(unsigned int y = min_y_; y < max_y_; y++){
                for(unsigned int x = min_x_; x < max_x_; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, map.info.width);
                    if(map.data[idx] == -1){
                        unexplored_count++;
                    }
                    total_count++;
                }
            }
            return (double)unexplored_count / total_count;
        }

        double calculateDisance(const nav_msgs::msg::OccupancyGrid& map,
                                const geometry_msgs::msg::PoseStamped& robot_pose){
            double center_y = (min_y_ + max_y_) / 2.0 * map.info.resolution + map.info.origin.position.y;
            double center_x = (min_x_ + max_x_) / 2.0 * map.info.resolution + map.info.origin.position.x;
            double robot_y = robot_pose.pose.position.y;
            double robot_x = robot_pose.pose.position.x;

            return std::hypot(center_x - robot_x, center_y - robot_y);
        }


        void visualize(visualization_msgs::msg::MarkerArray& marker_array, int& id,
                       const nav_msgs::msg::OccupancyGrid& map){

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "sector_lines";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.05;
            marker.color.a = 1.0;
            marker.color.b = 1.0;

            double min_x = map.info.origin.position.x + min_x_ * map.info.resolution;
            double min_y = map.info.origin.position.y + min_y_ * map.info.resolution;
            double max_x = map.info.origin.position.x + max_x_ * map.info.resolution;
            double max_y = map.info.origin.position.y + max_y_ * map.info.resolution;

            geometry_msgs::msg::Point p1, p2, p3, p4;
            p1.x = min_x; p1.y = min_y; p1.z = 0.0;
            p2.x = min_x; p2.y = max_y; p2.z = 0.0;
            p3.x = max_x; p3.y = max_y; p3.z = 0.0;
            p4.x = max_x; p4.y = min_y; p4.z = 0.0;

            marker.points.push_back(p1); marker.points.push_back(p2);
            marker.points.push_back(p2); marker.points.push_back(p3);
            marker.points.push_back(p3); marker.points.push_back(p4);
            marker.points.push_back(p4); marker.points.push_back(p1);
            marker_array.markers.push_back(marker);

            for(int i = 0; i < 4; i++){
                if(children_[i] != nullptr){
                    children_[i]->visualize(marker_array, id, map);
                }
            }
        }

        unsigned int min_y() const { return min_y_; }
        unsigned int min_x() const { return min_x_; }
        unsigned int max_y() const { return max_y_; }
        unsigned int max_x() const { return max_x_; }

        std::shared_ptr<QuadtreeNode> children(int i) const { return children_[i]; }

    private:
        unsigned int min_y_;
        unsigned int min_x_;
        unsigned int max_y_;
        unsigned int max_x_;
        std::shared_ptr<QuadtreeNode> children_[4];
};

class FrontierExplorer{
    public:
        FrontierExplorer(rclcpp::Node* node)
            : node_(node){}

        void setFrontier(const nav_msgs::msg::OccupancyGrid& map){
            MapManager& mm = MapManager::getInstance();
            unsigned int width = map.info.width;

            for(unsigned int idx = 0; idx < map.data.size(); idx++){
                unsigned int temp_x = idx % width;
                unsigned int temp_y = idx / width;

                mm.updateWall(temp_y, temp_x, temp_y, temp_x);
            }
        }

        int getDesireCorner(){
            if(corner_queue.empty()) return 0;
            return corner_queue.front();
        }

        bool getQueueEmpty(){
            return corner_queue.empty();
        }
        bool popCornerQueue(){
            if(corner_queue.empty()) return false;
            corner_queue.pop();
            return true;
        }

        void pushCornerQueue(int sector_number[4]){
            for(int i = 0; i < 4; i++){
                corner_queue.push(sector_number[i]);
            }
        }

        P mapExpandingSequence(int sector){
            MapManager& mm = MapManager::getInstance();
            auto min_max = mm.getWall();
            unsigned int min_y = std::get<0>(min_max);
            unsigned int min_x = std::get<1>(min_max);
            unsigned int max_y = std::get<2>(min_max);
            unsigned int max_x = std::get<3>(min_max);

            unsigned int frontier_goal[4][2] = {{max_y, max_x}, {max_y, min_x}, {min_y, min_x}, {min_y, max_x}};

            P current_desire_goal = {frontier_goal[sector - 1][0], frontier_goal[sector - 1][1]};
            P fixed_goal = findSafeGoal(current_desire_goal, sector);
            return fixed_goal;
        }

        P findSafeGoal(P coord, int sector){
            int offset_points = 20;
            int offset[4][2] = {{-offset_points, -offset_points}, {-offset_points, offset_points},
                                {offset_points, offset_points}, {offset_points, -offset_points}};

            P fixed_goal = {coord.first + offset[sector - 1][0], coord.second + offset[sector - 1][1]};
            return fixed_goal;
        }

    private:
        rclcpp::Node* node_;
        std::queue<int> corner_queue;
};

class NotSearchedFirstExplorer{
    public:
        NotSearchedFirstExplorer(rclcpp::Node* node)
            : node_(node){}

        P setGoal(const nav_msgs::msg::OccupancyGrid& map){
            MapManager& mm = MapManager::getInstance();
            PoseManager& PM = PoseManager::getInstance();
            geometry_msgs::msg::PoseStamped robot_pose = PM.getLastPose();

            RCLCPP_INFO(node_->get_logger(),"set goal");
            P near_goal = {0, 0};
            unsigned int pixel_y = (robot_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
            unsigned int pixel_x = (robot_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
            P current_pixel_coord = {pixel_y, pixel_x};
            unsigned int max_y = std::get<2>(mm.getWall());
            unsigned int max_x = std::get<3>(mm.getWall());

            double min_dist = std::numeric_limits<double>::max();
            for(auto &itr : mm.getVisited()){
                if(itr.second == false){
                    // RCLCPP_INFO(this->get_logger(),"setgoal2");
                    if(itr.first.first >= max_y || itr.first.second >= max_x){ continue; }
                    unsigned int temp_idx = Utility::pixelcoordtoidx(itr.first.first, itr.first.second, map.info.width);
                    if(temp_idx >= map.data.size()){
                        // RCLCPP_ERROR(node_->get_logger(), "Invalid index: %u (map size: %zu)", temp_idx, map.data.size());
                        // RCLCPP_WARN(node_->get_logger(), "P Coord x : %d, y : %d, width : %d", itr.first.second, itr.first.first, map.info.width);
                        itr.second = true;
                        continue;
                    }
                    if(map.data[temp_idx] != -1){
                        itr.second = true;
                        // RCLCPP_INFO(node_->get_logger(),"Already gone");
                        continue;
                    }
                    // RCLCPP_INFO(node_->get_logger(),"visited[itr] = %d, map_->data[idx] = %d", itr.second, map_->data[temp_idx]);
                    // RCLCPP_INFO(node_->get_logger(), "current coord = ( %d, %d )", itr.first.first, itr.first.second);
                    double dist = Utility::distance(itr.first, current_pixel_coord);
                    if( dist < min_dist){
                        min_dist = dist;
                        near_goal = itr.first;
                    }
                }
            }
            if(near_goal == P{0, 0}){
                // RCLCPP_INFO(node_->get_logger(),"All point visited");
                return {0, 0}; // temp
            }else{
                // RCLCPP_INFO(node_->get_logger(),"current position (%f, %f)", robot_pose.pose.position.x, robot_pose.pose.position.y);
                // RCLCPP_INFO(node_->get_logger(),"current pixel position (%d, %d)", pixel_x, pixel_y);
                // RCLCPP_INFO(node_->get_logger(),"set near goal (%d, %d), dist : %f", near_goal.first, near_goal.second, min_dist);
                return {near_goal};
            }
        }


    private:
        rclcpp::Node* node_;

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


            visit_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visited_points", 10);
            sector_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("Sector_lines", 10);
            laser_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("laser_line", 10);
            goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_points", 10);
            frontier_explorer_ = std::make_shared<FrontierExplorer>(this);
            // not_searched_first_explorer_ = std::make_shared<NotSearchedFirstExplorer>(this);
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Exploration_map::timer_callback, this));

            int corner_number[4] = {3, 2, 4, 1};
            frontier_explorer_->pushCornerQueue(corner_number);
        }

    protected:

        void timer_callback(){
            // RCLCPP_INFO(this->get_logger(),"System status : map = %d , pose = %d, jobdone = %d", i_sub_map, i_sub_pose, jobdone);
            if(first_move && explore){
                RCLCPP_INFO(this->get_logger(), "First move");
                explore = false;
                firstMove();
            }else if(!first_move && explore && !force_move){
                if(expand_map_sq){
                    int sector = frontier_explorer_->getDesireCorner();
                    RCLCPP_INFO(this->get_logger(), "Explore Corner : %d", sector);

                    current_pixel_goal = frontier_explorer_->mapExpandingSequence(sector);
                }else{
                    // current_pixel_goal = not_searched_first_explorer_->setGoal(map_);
                    if(root_ != nullptr){
                        std::shared_ptr<QuadtreeNode> best_node = bestChild(root_, map_);
                        RCLCPP_INFO(this->get_logger(), "find new goal");

                        unsigned int goal_x = (best_node->min_x() + best_node->max_x()) / 2;
                        unsigned int goal_y = (best_node->min_y() + best_node->max_y()) / 2;

                        current_pixel_goal = {goal_y, goal_x};
                    }
                }
                RCLCPP_INFO(this->get_logger(), "pixel goal : %d, %d", current_pixel_goal.second, current_pixel_goal.first);
                pubGoal(current_pixel_goal, map_);
                explore = false;
                expanded = false;
            }
        }

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                fail_count = std::clamp(fail_count, 0, 20);
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "@@@@@@@@@@@@@@ Failed to compute pose. Failcount : %d @@@@@@@@@@@@@@", fail_count);
                    if(!expand_map_sq){
                        fail_count++;
                        // MapManager::getInstance().setVisited(current_pixel_goal, true);
                    }
                    fail_flag = true;
                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    fail_count = 0;
                    if(first_move) first_move = false;
                    fail_flag = false;

                    if(split_route && expand_map_sq){
                        RCLCPP_INFO(this->get_logger(), "Split");
                        pubGoal(current_pixel_goal, map_);
                        split_route = true;
                    }
                }
                // else if(node_name_ == "FollowPath" && current_status_ == "RUNNING"){
                // }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "@@@@@@@@@@@@@@ Failed to move. @@@@@@@@@@@@@@");

                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@@@@@ READY 2 MOVE @@@@@@@@@@@@@@");
                    RCLCPP_INFO(this->get_logger(),"split : %d", split_route);
                    auto wall = MapManager::getInstance().getWall();
                    static std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> last_wall;
                    if(!Utility::isSameTuple(last_wall, wall)){
                        last_wall = wall;
                        RCLCPP_INFO(this->get_logger(),"Map Expanded");
                        expanded = true;
                    }
                    else{
                        expanded = false;
                    }
                    RCLCPP_INFO(this->get_logger(),"min_x = %d, min_y = %d, max_x = %d, max_y = %d", std::get<1>(wall), std::get<0>(wall), std::get<3>(wall), std::get<2>(wall));
                    if(expand_map_sq){
                        if(!expanded && !fail_flag && !split_route){
                            RCLCPP_INFO(this->get_logger(), "CornerQueue pop");
                            frontier_explorer_->popCornerQueue();
                        }
                        if(frontier_explorer_->getDesireCorner() == 0){
                            RCLCPP_INFO(this->get_logger(), "Explore Finished");
                            expand_map_sq = false;
                        }
                    }

                    // if(!expand_map_sq){
                    //     MapManager::getInstance().setVisited(current_pixel_goal, true);
                    // }

                    explore = true;

                    if(fail_count == 20){
                        fail_count = 0;
                    }

                    if(explore){ RCLCPP_INFO(this->get_logger(), "Explore"); }
                    else { RCLCPP_INFO(this->get_logger(), "Not Explore"); }
                }
            }
        }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            sensor_msgs::msg::LaserScan laser_msg_ = *msg;
            LaserManager::getInstance().setLaserMsg(laser_msg_);
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }
            frontier_explorer_->setFrontier(map_);

            map_ = *msg;

            if(!expand_map_sq){
                MapManager& mm = MapManager::getInstance();
                auto min_max = mm.getWall();
                root_ = std::make_shared<QuadtreeNode>(std::get<0>(min_max), std::get<1>(min_max), std::get<2>(min_max), std::get<3>(min_max));
                root_->split(map_);

                publishSector(map_);
            }
        }

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            PoseManager::getInstance().setLastPose(*msg);
        }

        void firstMove(){
            geometry_msgs::msg::PoseStamped robot_pose = PoseManager::getInstance().getLastPose();

            geometry_msgs::msg::PoseStamped goal_pose_first;
            goal_pose_first.header.frame_id = "map";
            goal_pose_first.header.stamp = this->now();


            double robot_direction = PoseManager::getInstance().getRobotdirection();

            tf2::Quaternion q;
            q.setRPY(0, 0, robot_direction + M_PI);

            goal_pose_first.pose.orientation = tf2::toMsg(q);
            const double direction[4][2] = {{-2.5, 0}, {2.5, 0}, {0, -2.5}, {0, 2.5}};
            static int count = 0;
            goal_pose_first.pose.position.x = robot_pose.pose.position.x + direction[count][0];
            goal_pose_first.pose.position.y = robot_pose.pose.position.y + direction[count][1];

            goal_pub_->publish(goal_pose_first);
            count++;
        }

        std::shared_ptr<QuadtreeNode> bestChild(std::shared_ptr<QuadtreeNode> root, const nav_msgs::msg::OccupancyGrid& map){
            if(root == nullptr) return nullptr;

            if(root->children(0) == nullptr){
                return root;
            }

            std::shared_ptr<QuadtreeNode> best_child = nullptr;
            double best_score = -1.0;
            for(int i = 0; i < 4; i++){
                std::shared_ptr<QuadtreeNode> child = root->children(i);
                double score = child->evalute(map);
                if(score > best_score){
                    best_score = score;
                    best_child = child;
                }
            }
            return bestChild(best_child, map);
        }

        /*
        std::vector<std::vector<P>> GetObstacleShapes(const nav_msgs::msg::OccupancyGrid& map){
            std::vector<std::vector<P>> obstacle;
            std::vector<P> current_obs;
            // RCLCPP_INFO(this->get_logger(),"scan obstacle shapes");
            PoseManager& PM = PoseManager::getInstance();
            const geometry_msgs::msg::PoseStamped& robot_pose = PM.getLastPose();

            double robot_x = robot_pose.pose.position.x;
            double robot_y = robot_pose.pose.position.y;
            double prev_dist = 0.0;
            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            double width = map.info.width;
            double height = map.info.height;
            double resolution = map.info.resolution;
            // bool last_sign = false;

            double theta = PM.getRobotdirection();
            LaserManager& LM = LaserManager::getInstance();
            sensor_msgs::msg::LaserScan laser_scan = LM.getLaserMsg();


            for(size_t i = 0; i < laser_scan.ranges.size(); i++){
                double laser_dist = laser_scan.ranges[i];
                double laser_angle = laser_scan.angle_min + i * laser_scan.angle_increment;

                if(std::isnan(laser_dist) || std::isinf(laser_dist) ||
                    laser_dist < laser_scan.range_min || laser_dist > laser_scan.range_max){
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

                if(obs_x_map < origin_x || obs_y_map < origin_y ||
                    obs_x_map >= origin_x + width * resolution ||
                    obs_y_map >= origin_y + height * resolution){
                    continue;
                }


                unsigned int pixel_y = (obs_y_map - origin_y) / resolution;
                unsigned int pixel_x = (obs_x_map - origin_x) / resolution;
                if(pixel_x >= width || pixel_y >= height){
                    continue; // 무효한 좌표는 무시
                }
                MapManager::getInstance().addOSingletonset(pixel_y, pixel_x);

                double angle_diff = laser_angle - (laser_scan.angle_min + (i - 1) * laser_scan.angle_increment);
                double sign_distance_diff = laser_dist - prev_dist;
                double distance_diff = fabs(sign_distance_diff);
                // bool sign = (sign_distance_diff > 0) ? true : false;



                if(!current_obs.empty() && (distance_diff > 0.5 || angle_diff > 0.1)){
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

        void ObstacleFloodFill(const std::vector<std::vector<P>>& obstacles){
            // RCLCPP_INFO(this->get_logger(),"flood fill");
            // int temp_size = obstacles.size();
            MapManager& mm = MapManager::getInstance();
            std::set<P> obstacle_mask = mm.getSingletonset();
            // RCLCPP_INFO(this->get_logger(),"obstacles size : %d ",temp_size);
            for(const auto& obstacle : obstacles){
                if(obstacle.size() < 3) continue;
                std::queue<P> q;

                unsigned int min_x = std::numeric_limits<unsigned int>::max();
                unsigned int min_y = std::numeric_limits<unsigned int>::max();
                unsigned int max_x = 0;
                unsigned int max_y = 0;

                std::set<P> obstacle_set(obstacle.begin(), obstacle.end());

                for(const auto& point : obstacle){
                    min_x = std::min(min_x, point.second);
                    min_y = std::min(min_y, point.first);
                    max_x = std::max(max_x, point.second);
                    max_y = std::max(max_y, point.first);
                }

                std::vector<P> seed_points;
                for(unsigned int y = min_y; y <= max_y; y++){
                    for(unsigned int x = min_x; x <= max_x; x++){
                        if(!obstacle_set.count({y, x}) && !obstacle_mask.count({y, x})){
                            seed_points.push_back({y, x});
                        }
                    }
                }
                for(const auto& seed_point : seed_points){
                    q.push(seed_point);
                    while(!q.empty()){
                        // int temp = q.size();
                        // RCLCPP_INFO(this->get_logger(), "q.size : %d", temp);
                        auto [y, x] = q.front();
                        q.pop();
                        if(x < min_x || x > max_x || y < min_y || y > max_y){
                            // RCLCPP_INFO(this->get_logger(),"out of range");
                            continue;
                        }
                        if(y >= std::get<2>(mm.getWall()) || x >= std::get<3>(mm.getWall())){
                            // RCLCPP_INFO(this->get_logger(),"out of range2");
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
                        q.push({y + 1, x + 1});
                        q.push({y - 1, x - 1});
                        q.push({y + 1, x - 1});
                        q.push({y - 1, x + 1});
                    }
                }
            }
        }

        void excludeObstacle(const nav_msgs::msg::OccupancyGrid& map){
            // RCLCPP_INFO(this->get_logger(),"scan obstacle");

            auto obstacles = GetObstacleShapes(map);

            ObstacleFloodFill(obstacles);
        }
        */

        void publishSector(nav_msgs::msg::OccupancyGrid& map){
            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = "map";
            clear.header.stamp = this->get_clock()->now();
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            clear.ns = "sector_lines";
            marker_array.markers.push_back(clear);
            sector_marker_pub_->publish(marker_array);
            marker_array.markers.clear();

            int id = 0;
            root_->visualize(marker_array, id, map);
            sector_marker_pub_->publish(marker_array);

        }

        void publishVisitedPoints(const nav_msgs::msg::OccupancyGrid& map){
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
            points_unexplored.color.r = 1.0;
            points_unexplored.color.g = 0.0;

            points_unexplored.points.clear();

            for(const auto& itr : MapManager::getInstance().getVisited()){
                if(itr.second == false){
                    geometry_msgs::msg::Point p;
                    p.y = itr.first.first * map.info.resolution + map.info.origin.position.y;
                    p.x = itr.first.second * map.info.resolution + map.info.origin.position.x;
                    p.z = 0.0;
                    points_unexplored.points.push_back(p);
                }
            }
            visit_marker_pub_->publish(points_unexplored);
        }

        void publishscanPoints(const sensor_msgs::msg::LaserScan& laser_scan,
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
                tf2::doTransform(base_link_origin, map_origin, transform); // 로봇 좌표계 -> 지도 좌표계

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

        void publishGoalPoints(const geometry_msgs::msg::PoseStamped& goal){
            //목표 시각화
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

        void search(const nav_msgs::msg::OccupancyGrid& map){ // 언제 다시 동기화 해야할지
            //현재 지도의 데이터가 -1 인 idx의 픽셀좌표를 visited 맵에 저장
            //현재 지도의 데이터에서 -1 인 idx의 픽셀좌표를 저장
            // RCLCPP_INFO(this->get_logger(),"search");
            MapManager& mm = MapManager::getInstance();

            unsigned int height = std::get<2>(mm.getWall());
            unsigned int width = std::get<3>(mm.getWall());
            if(height < 10 || width < 10){
                return;
            }

            for(unsigned int y = 10; y <= height - 10; y++){
                for(unsigned int x = 10; x <= width - 10; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, map.info.width);
                    if(map.data[idx] == -1){
                        mm.insertVisited(y, x, false);
                    }else{
                        mm.setVisited(y, x, true);
                    }
                }
            }

        }

        std::pair<double, double> setGoal_Angleanddist(const nav_msgs::msg::OccupancyGrid& map, double laser_angle_rad, double distance){
            double robot_x = PoseManager::getInstance().getLastPose().pose.position.x;
            double robot_y = PoseManager::getInstance().getLastPose().pose.position.y;
            double robot_theta = PoseManager::getInstance().getRobotdirection();
            float offset = 0.75;

            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            float resolution = map.info.resolution;

            double robot_std_dir = laser_angle_rad - M_PI + robot_theta;

            double goal_x = distance * cos(robot_std_dir) + robot_x;
            goal_x = std::clamp(goal_x , origin_x + offset, origin_x + map.info.width * resolution - offset);
            double goal_y = distance * sin(robot_std_dir) + robot_y;
            goal_y = std::clamp(goal_y , origin_y + offset , origin_y + map.info.height * resolution - offset);

            return {goal_y, goal_x};
        }

        void isOverObstacle(const nav_msgs::msg::OccupancyGrid& map,
                             PoseManager& PM, LaserManager& LM, int fail_count){

            double angle_to_goal = PM.getDirectionWithRobotGoal();

            double robot_theta = PM.getRobotdirection();
            double relative_angle_to_goal = angle_to_goal - robot_theta; // 로봇 좌표계 기준

            relative_angle_to_goal = std::fmod(relative_angle_to_goal + M_PI, 2 * M_PI) - M_PI;

            double laser_angle_min = LM.getLaserMsg().angle_min;
            double laser_angle_increment = LM.getLaserMsg().angle_increment;
            int index = (relative_angle_to_goal - laser_angle_min) / laser_angle_increment;
            index -= 2; // temp
            publishscanPoints(LM.getLaserMsg(), index, relative_angle_to_goal);
            RCLCPP_INFO(this->get_logger(), "index at goal direction : %d", index);

            double range_at_angle = LM.getLaserMsg().ranges[index];

            double dist_goal = PM.getdistancewithRobotGoal();
            RCLCPP_WARN(this->get_logger(), "목표까지의 거리 : %f ", dist_goal);
            RCLCPP_WARN(this->get_logger(), "측정 거리 : %f ", range_at_angle);

            if(dist_goal < 1){
                split_route = false;
            }
            if(std::isinf(range_at_angle)){
                RCLCPP_INFO(this->get_logger(), "No valid data at this angle");
                double angle_rad = index * M_PI / 180.0;
                std::pair<double, double> another_goal;

                another_goal = setGoal_Angleanddist(map, angle_rad, dist_goal);

                PM.getGoalPose().pose.position.x = another_goal.second;
                PM.getGoalPose().pose.position.y = another_goal.first;
                publishGoalPoints(PM.getGoalPose());
            }

            double threshold = -0.1;
            if(dist_goal - range_at_angle > threshold){ // 목표지점이 장애물 너머일 경우
                RCLCPP_INFO(this->get_logger(), "is over obstacle");
                if(dist_goal > 15.0){
                    split_route = true;
                    range_at_angle /= 2.0;
                }
                setGoalbesideObstacle(LM.getLaserMsg(), map, index, range_at_angle, fail_count);
            }else{ // 아닐경우
                RCLCPP_ERROR(this->get_logger(), "is not over obstacle, Dg : %f , RaA : %f", dist_goal, range_at_angle);
                publishGoalPoints(PM.getGoalPose());
            }

        }

        void setGoalbesideObstacle(const sensor_msgs::msg::LaserScan& laser_scan,
                                   const nav_msgs::msg::OccupancyGrid& map,
                                   const int& start_index, double dist_goal, int fail_count
                                   ){
            double laser_angle = 0.0; // deg

            int index = start_index;
            if(start_index == 1 || start_index == 359){
                index = 0;
            }

            double prev_dist_pos = laser_scan.ranges[index];
            double prev_dist_neg = laser_scan.ranges[index];

            double min_angle_threshold = 0.25;
            int pos_dir = index + 1;
            int neg_dir = index - 1;

            while(true && rclcpp::ok()){
                if(neg_dir == -1){
                    neg_dir = 359;
                }
                if(pos_dir == 360){
                    pos_dir = 0;
                }

                double range_at_dist_pos = laser_scan.ranges[pos_dir];
                double range_at_dist_neg = laser_scan.ranges[neg_dir];

                double dist_diff_pos = range_at_dist_pos - prev_dist_pos;
                double dist_diff_neg = range_at_dist_neg - prev_dist_neg;

                prev_dist_pos = range_at_dist_pos;
                prev_dist_neg = range_at_dist_neg;

                if(dist_diff_pos < 0){
                    pos_dir++;
                    if(pos_dir == 360) pos_dir = 0;
                }
                if(dist_diff_neg < 0){
                    neg_dir--;
                    if(neg_dir == -1) neg_dir = 359;
                }

                int offset = 5 + fail_count % 5;
                if(offset == 0) offset = 5;
                if(dist_diff_pos > min_angle_threshold){
                    laser_angle = pos_dir + offset;
                    RCLCPP_INFO(this->get_logger(),"positive side goal set, laser_angle : %f", laser_angle);
                    break;
                }
                if(dist_diff_neg > min_angle_threshold){
                    laser_angle = neg_dir - offset;
                    RCLCPP_INFO(this->get_logger(),"negative side goal set, laser_angle : %f", laser_angle);
                    break;
                }
                pos_dir++;
                neg_dir--;

            }
            laser_angle = std::clamp(laser_angle, 0.0, 360.0);

            double min_margin = 0.75;
            double max_margin = 5.0;
            double k = fail_count * 0.1;
            k = std::clamp(k, 0.1, 4.5);
            double safe_margin = 0;
            if(dist_goal < 3.0){
                safe_margin = max_margin - k * dist_goal;
            }else{
                safe_margin = min_margin + k * dist_goal;
            }
            safe_margin = std::clamp(safe_margin, min_margin, max_margin);

            double laser_angle_rad = laser_angle * M_PI / 180.0 ;

            std::pair<double, double> another_goal = setGoal_Angleanddist(map, laser_angle_rad, safe_margin + dist_goal);

            PoseManager& PM = PoseManager::getInstance();

            PM.getGoalPose().pose.position.x = another_goal.second;
            PM.getGoalPose().pose.position.y = another_goal.first;

            publishGoalPoints(PM.getGoalPose());
            isOverObstacle(map, PM, LaserManager::getInstance(), fail_count + 1);
        }

        void pubGoal( P goal, const nav_msgs::msg::OccupancyGrid& map){
            if(test) return;
            PoseManager& pm = PoseManager::getInstance();
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";
            // RCLCPP_INFO(this->get_logger(), "Goal_pixel : (x : %d, y : %d)", goal.second, goal.first);
            goal_msg.pose.position.y = map.info.origin.position.y + goal.first * map.info.resolution;
            goal_msg.pose.position.x = map.info.origin.position.x + goal.second * map.info.resolution;
            // goal_msg.pose.position.x = 5.0; // for test
            // goal_msg.pose.position.y = 0.0;
            RCLCPP_INFO(this->get_logger(), "초기 목표 : (x : %f, y : %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);

            pm.setGoalPose(goal_msg);
            if(!first_move){
                isOverObstacle(map, pm, LaserManager::getInstance(), fail_count);
            }
            RCLCPP_INFO(this->get_logger(), "수정 목표 : (x : %f, y : %f)", pm.getGoalPose().pose.position.x, pm.getGoalPose().pose.position.y);
            publishGoalPoints(pm.getGoalPose());

            double goal_direction = pm.getDirectionWithRobotGoal();
            goal_direction = std::fmod(goal_direction + M_PI, 2 * M_PI) - M_PI;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_direction);
            pm.getGoalPose().pose.orientation = tf2::toMsg(q);
            goal_pub_->publish(pm.getGoalPose());
            pub_time = this->now();
        }


    private:
        int fail_count = 0;
        int force_move_count = 0;
        int force_move_threshold = 5;

        std::set<int> sector_set;

        bool test = false;
        rclcpp::Time pub_time;

        bool split_route = false;
        bool first_move = true;
        bool expanded = false;
        bool explore = true; // 동작 완료 후 활성 , 이동 없는 테스트는 비활성
        bool expand_map_sq = true;
        bool fail_flag = false;
        bool force_move = false;
        bool end_explore = false;
        P last_fail_point;
        P current_pixel_goal;

        std::shared_ptr<QuadtreeNode> root_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<FrontierExplorer> frontier_explorer_;
        // std::shared_ptr<NotSearchedFirstExplorer> not_searched_first_explorer_;


        nav_msgs::msg::OccupancyGrid map_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visit_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr laser_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sector_marker_pub_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration_map>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
