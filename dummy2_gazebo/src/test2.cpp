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
#include <functional>

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

        static double distance(std::pair<double, double> C1, std::pair<double, double> C2){
            return std::hypot(C1.first - C2.first, C1.second - C2.second);
        }

        static unsigned int pixelcoordtoidx(unsigned int y, unsigned int x, uint32_t width){
            return (y * width + x);
        }


        static std::pair<double, double> pixeltoreal(P pixel_coord, nav_msgs::msg::OccupancyGrid map){
            double real_x = (pixel_coord.second * map.info.resolution) + map.info.origin.position.x;
            double real_y = (pixel_coord.first * map.info.resolution) + map.info.origin.position.y;
            return {real_y, real_x};
        }
    };

class QuadtreeNode {
    public:
        QuadtreeNode(unsigned int min_y, unsigned int min_x, unsigned int max_y, unsigned int max_x)
            : min_y_(min_y), min_x_(min_x), max_y_(max_y), max_x_(max_x), visited_count_(0){
            for(int i = 0; i < 4; i++){
                children_[i] = nullptr;
            }
        }

        double evalute(const nav_msgs::msg::OccupancyGrid& map){
            // double obstacle_rate = calculateObstacleRate(map);
            double unexplored_rate = calculateUnexploredRate(map);
            double obstacle_rate = calculateObstacleRate(map);
            double explored_rate = calculateExploredRate(map);
            double distance = calculateDisance(map, PoseManager::getInstance().getLastPose());
            double score = 0;
            unsigned int size = max_x_ - min_x_;


            if(explored_rate < 0.01 || distance < 2.0){
                score -= 50;
                return score;
            }

            if(size > 50){
                if(explored_rate > 0.9){
                    score -= explored_rate * 5.0;
                }
                if(unexplored_rate > explored_rate){
                    if(unexplored_rate > 0.7){
                        score += 0.1;
                    }else if(unexplored_rate < 0.4){
                        score += unexplored_rate * 0.7;
                    }else{
                        score += unexplored_rate * 0.3;
                    }
                }else{
                    score += 0.02;
                }

                score -= visited_count_ * 0.8;
            }else{
                if(obstacle_rate > 0.01) return -40;
                if(explored_rate < 0.8){
                    score += explored_rate;
                }else{
                    score -= 15.0;
                }
                if(unexplored_rate > 0.2 && unexplored_rate < 0.8 ){
                    score += unexplored_rate * 2.5;
                }else{
                    score -= unexplored_rate * 2.5;
                }
                score -= visited_count_ * 0.5;
            }

            double distance_factor = size <= 30 ? 4.0 : 1.0;
            double distance_score = distance_factor / (distance - 1.5);
            double max_distance_scroe = size <= 30 ? 2.0 : 0.5;
            distance_score = std::min(distance_score, max_distance_scroe);
            score += distance_score;

            double size_score = 20.0 / (size + 5.0);
            double max_size_score = size <= 30 ? 1.8 : 0.4;
            size_score = std::min(size_score, max_size_score);
            score += size_score;

            return score;
        }

        bool merge(){
            if(children_[0] == nullptr){
                return false; // 자식노드가 없으면 합칠 수 없음
            }

            for(int i = 0; i < 4; i++){ // 자식노드가 모두 leaf 노드가 아니면 합칠 수 없음
                if(children_[i]->children(0) != nullptr){
                    // RCLCPP_INFO(rclcpp::get_logger("Exploration_map_Node"), "merge failed: child %d is not a leaf node", i);
                    return false;
                }
            }

            for(int i = 0; i < 4; i++){
                children_[i] = nullptr;
            }

            is_merged_ = true;

            return true;
        }

        bool split(const nav_msgs::msg::OccupancyGrid& map){
            double unexplore_rate = calculateUnexploredRate(map);

            if(unexplore_rate > 0.02 && (max_x_ - min_x_ >= 5) && (max_y_ - min_y_ >= 5)){
                unsigned int mid_y = (min_y_ + max_y_) / 2;
                unsigned int mid_x = (min_x_ + max_x_) / 2;

                if(children_[0] == nullptr) children_[0] = std::make_shared<QuadtreeNode>(min_y_, min_x_, mid_y, mid_x);
                if(children_[1] == nullptr) children_[1] = std::make_shared<QuadtreeNode>(min_y_, mid_x, mid_y, max_x_);
                if(children_[2] == nullptr) children_[2] = std::make_shared<QuadtreeNode>(mid_y, min_x_, max_y_, mid_x);
                if(children_[3] == nullptr) children_[3] = std::make_shared<QuadtreeNode>(mid_y, mid_x, max_y_, max_x_);

                for(int i = 0; i < 4; i++){
                    children_[i]->split(map);
                }
                return true;
            }else{
                bool merged = merge();
                return merged;
            }
            return false;
        }

        double calculateObstacleRate(const nav_msgs::msg::OccupancyGrid& map){
            int obstacle_count = 0;
            int total_count = 0;
            for(unsigned int y = min_y_; y < max_y_; y++){
                for(unsigned int x = min_x_; x < max_x_; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, map.info.width);
                    if(map.data[idx] == 100){
                        obstacle_count++;
                    }
                    total_count++;
                }
            }
            return (double)obstacle_count / total_count;
        }

        double calculateExploredRate(const nav_msgs::msg::OccupancyGrid& map){
            int explored_count = 0;
            int total_count = 0;
            for(unsigned int y = min_y_; y < max_y_; y++){
                for(unsigned int x = min_x_; x < max_x_; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, map.info.width);
                    if(map.data[idx] == 0){
                        explored_count++;
                    }
                    total_count++;
                }
            }
            return (double)explored_count / total_count;
        }

        double calculateUnexploredRate(const nav_msgs::msg::OccupancyGrid& map){
            int unexplored_count = 0;
            int total_count = 0;
            for(unsigned int y = min_y_; y < max_y_; y++){
                for(unsigned int x = min_x_; x < max_x_; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, map.info.width);
                    if(idx < map.data.size()&& map.data[idx] == -1){
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

        unsigned int min_y() const { return min_y_; }
        unsigned int min_x() const { return min_x_; }
        unsigned int max_y() const { return max_y_; }
        unsigned int max_x() const { return max_x_; }

        std::shared_ptr<QuadtreeNode> children(int i) const { return children_[i]; }

        void setChild(int i, std::shared_ptr<QuadtreeNode> child){
            if(i >= 0 && i < 4){
                children_[i] = child;
            }
        }

        void setVisitedCount(int count){ visited_count_ = count; }
        void increaseVisitedCount(int count = 1){ visited_count_ += count; }
        int getVisitedCount(){ return visited_count_; }

        void setIsMerged(bool is_merged){ is_merged_ = is_merged; }
        bool getIsMerged(){ return is_merged_;}

        QuadtreeNode(const QuadtreeNode&) = delete;
        QuadtreeNode& operator=(const QuadtreeNode&) = delete;

    private:
        unsigned int min_y_;
        unsigned int min_x_;
        unsigned int max_y_;
        unsigned int max_x_;
        bool is_merged_ = false;
        std::shared_ptr<QuadtreeNode> children_[4];
        int visited_count_;
        int marker_id_ = 0;
};

class FrontierExplorer{
    public:
        FrontierExplorer(rclcpp::Node* node)
            : node_(node){}

        int checkDataSize(const nav_msgs::msg::OccupancyGrid& map){
            int diff = (int)map.data.size() - map_data_size_;
            if(diff > 0)
                return map_data_size_ = (int)map.data.size();
            else
                return 0;
        }
        void setFrontier(const nav_msgs::msg::OccupancyGrid& map){
            MapManager& mm = MapManager::getInstance();
            unsigned int width = map.info.width;
            unsigned int height = map.info.height;
            if(width != 0 && height != 0)
                mm.updateWall(0, 0, height, width);
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
            // P fixed_goal = findSafeGoal(current_desire_goal, sector);
            return current_desire_goal;
        }


    private:
        rclcpp::Node* node_;
        std::queue<int> corner_queue;
        int map_data_size_ = 0;
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

class Exploration_map : public rclcpp::Node, public std::enable_shared_from_this<Exploration_map>
{
    public:

        Exploration_map() : rclcpp::Node("Exploration_map_Node"), root_(nullptr){

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
            sector_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("Sector_lines", 10);
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

                    current_pixel_goal_ = frontier_explorer_->mapExpandingSequence(sector);
                    auto temp_real_coord = Utility::pixeltoreal(current_pixel_goal_, map_);
                    auto temp_last_pose = PoseManager::getInstance().getLastPose();
                    double dist = Utility::distance(temp_real_coord, {temp_last_pose.pose.position.y, temp_last_pose.pose.position.x});
                    if(dist > 20.0){
                        split_route = true;
                    }else if(split_route && dist < 1.0){
                        split_route = false;
                        frontier_explorer_->popCornerQueue();
                        explore = true;
                        return;
                    }
                }else{
                    // current_pixel_goal = not_searched_first_explorer_->setGoal(map_);
                    if(root_ != nullptr){
                        std::shared_ptr<QuadtreeNode> best_node = bestChild(root_, map_);
                        RCLCPP_INFO(this->get_logger(), "find new goal");
                        double score = best_node->evalute(map_);
                        int visited_count = best_node->getVisitedCount();
                        unsigned int goal_x = (best_node->min_x() + best_node->max_x()) / 2;
                        unsigned int goal_y = (best_node->min_y() + best_node->max_y()) / 2;
                        RCLCPP_INFO(this->get_logger(), "score : %f, visited_count : %d", score, visited_count);
                        RCLCPP_INFO(this->get_logger(), "size : %d", best_node->max_x() - best_node->min_x());
                        RCLCPP_INFO(this->get_logger(), "unexplored rate : %f", best_node->calculateUnexploredRate(map_));
                        RCLCPP_INFO(this->get_logger(), "explored rate : %f", best_node->calculateExploredRate(map_));

                        publishGoalSector(map_, best_node->min_x(), best_node->min_y(), best_node->max_x(), best_node->max_y());
                        current_pixel_goal_ = {goal_y, goal_x};
                    }
                }
                // RCLCPP_INFO(this->get_logger(), "pixel goal : %d, %d", current_pixel_goal_.second, current_pixel_goal_.first);
                pubGoal(current_pixel_goal_, map_);
                explore = false;
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

                    fail_flag = true;
                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    fail_count = 0;
                    if(first_move) first_move = false;
                    fail_flag = false;
                }
                // else if(node_name_ == "FollowPath" && current_status_ == "RUNNING"){
                // }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "@@@@@@@@@@@@@@ Failed to move. @@@@@@@@@@@@@@");

                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@@@@@ READY 2 MOVE @@@@@@@@@@@@@@");

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
            static unsigned int last_width = 0, last_height = 0;
            if(msg->info.width - last_width > 5 || msg->info.height - last_height > 3){
                RCLCPP_WARN(this->get_logger(),
                           "Map size changed: %u x %u -> %u x %u",
                           last_width, last_height,
                           msg->info.width, msg->info.height);

                MapManager::getInstance().updateWall(0, 0, msg->info.height, msg->info.width);

                expanded = true;
                last_width = msg->info.width;
                last_height = msg->info.height;
            }

            map_ = *msg;

            if(!expand_map_sq && !first_move){
                MapManager& mm = MapManager::getInstance();
                auto min_max = mm.getWall();
                unsigned int min_y = std::get<0>(min_max);
                unsigned int min_x = std::get<1>(min_max);
                unsigned int max_y = std::get<2>(min_max);
                unsigned int max_x = std::get<3>(min_max);

                if(root_ == nullptr){
                    RCLCPP_INFO(this->get_logger(), "First root created");
                    root_ = std::make_shared<QuadtreeNode>(min_y, min_x, max_y, max_x);
                }
                else if(expanded){
                    // RCLCPP_ERROR(this->get_logger(), "지도 확장 처리 시작 - copyQuadtree 호출 직전");
                    node_visits_.clear();
                    saveNodeVisits(root_);
                    // RCLCPP_ERROR(this->get_logger(), "방문 정보 저장 완료: %zu 개의 노드", node_visits_.size());

                    std::shared_ptr<QuadtreeNode> new_root = std::make_shared<QuadtreeNode>(min_y, min_x, max_y, max_x);
                    // RCLCPP_ERROR(this->get_logger(), "새 루트 노드 생성: [%u,%u,%u,%u]", min_y, min_x, max_y, max_x);
                    copyQuadtree(root_, new_root);
                    // RCLCPP_ERROR(this->get_logger(), "copyQuadtree 함수 실행 완료");
                    root_ = new_root;
                    // RCLCPP_ERROR(this->get_logger(), "root_ 교체 완료, 방문 횟수: %d", root_->getVisitedCount());

                    expanded = false;
                }
                root_->split(map_);
                // else if(root_ != nullptr){
                //     root_->split(map_, [weak_this = std::enable_shared_from_this<Exploration_map>::weak_from_this()](){
                //             if(auto shared_this = weak_this.lock()){
                //                 shared_this->initializeRoot();
                //             }
                //         }
                //     );
                // }
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

        std::shared_ptr<QuadtreeNode> bestChild(std::shared_ptr<QuadtreeNode> root, const nav_msgs::msg::OccupancyGrid& map, int depth = 0,
                                                std::shared_ptr<QuadtreeNode> parent = nullptr, int child_idx = -1,
                                                std::set<std::shared_ptr<QuadtreeNode>> visited_siblings = {}){

            if(root == nullptr){
                RCLCPP_WARN(this->get_logger(),"Root is Null");
                return nullptr;
            }

            if(depth > 10){
                RCLCPP_WARN(this->get_logger(),"Depth limit reached");
                return root;
            }

            if(root->children(0) == nullptr){
                RCLCPP_INFO(this->get_logger(), "Leaf node BEFORE increment: count=%d, node=%p",
                root->getVisitedCount(), (void*)root.get());

                root->increaseVisitedCount();
                RCLCPP_INFO(this->get_logger(), "Leaf node AFTER increment: count=%d", root->getVisitedCount());
                // RCLCPP_INFO(this->get_logger(), "Leaf node, root address: %p, visited count: %d", (void*)root.get(), root->getVisitedCount());
                return root;
            }

            double size_bias = 1.0;
            if(depth > 3){
                size_bias = 1.5;
            }

            std::shared_ptr<QuadtreeNode> best_child = nullptr;
            double best_score = 0.0;
            bool all_children_invalid = true;
            for(int i = 0; i < 4; i++){
                std::shared_ptr<QuadtreeNode> child = root->children(i);
                if(!child) continue;

                double score = child->evalute(map);
                unsigned int child_size = child->max_x() - child->min_x();
                if(child_size < 20){
                    score += size_bias;
                }

                if(score > 0){
                    all_children_invalid = false;
                    if(score > best_score){
                        best_score = score;
                        best_child = child;
                        publishGoalSector(map_, best_child->min_x(), best_child->min_y(), best_child->max_x(), best_child->max_y());
                    }
                }
            }

            if(all_children_invalid){
                // RCLCPP_WARN(this->get_logger(),"All children invalid, visited count: %d", root->getVisitedCount());
                std::shared_ptr<QuadtreeNode> least_bad_child = nullptr;
                double least_bad_score = - std::numeric_limits<double>::max();
                const double MIN_SCORE = -10.0;

                for(int i = 0; i < 4; i++){
                    std::shared_ptr<QuadtreeNode> child = root->children(i);
                    if(!child) continue;
                    double score = child->evalute(map);
                    if(score > least_bad_score && score > MIN_SCORE){
                        least_bad_score = score;
                        least_bad_child = child;
                        publishGoalSector(map_, least_bad_child->min_x(), least_bad_child->min_y(), least_bad_child->max_x(), least_bad_child->max_y());
                    }
                }

                if(least_bad_child && (least_bad_child->max_x() - least_bad_child->min_x() < 30)){
                    least_bad_child->increaseVisitedCount();
                    return least_bad_child;
                }

                if(parent != nullptr && child_idx >= 0){
                    visited_siblings.insert(root);

                    // RCLCPP_INFO(this->get_logger(), "Trying to find a sibling node");
                    for(int i = 0; i < 4; i++){
                        if(i == child_idx) continue;

                        std::shared_ptr<QuadtreeNode> sibling = parent->children(i);
                        if(!sibling) continue;

                        if(visited_siblings.find(sibling) != visited_siblings.end()){
                            // RCLCPP_INFO(this->get_logger(), "Skipping already visited sibling");
                            continue;
                        }

                        double score = sibling->evalute(map);
                        if(score > 0){
                            publishGoalSector(map_ , sibling->min_x(), sibling->min_y(), sibling->max_x(), sibling->max_y());
                            return bestChild(sibling, map, depth, parent, i, visited_siblings);
                        }
                    }

                    if(visited_siblings.size() >= 3){ // 형제 노드 탐색 횟수 제한
                        // RCLCPP_INFO(this->get_logger(), "going to grandparent");

                        // 만약 부모의 부모가 있다면 해당 노드에서 재탐색
                        if(parent != nullptr && parent->getVisitedCount() < 3){ // 부모 노드 방문 횟수도 고려
                            parent->increaseVisitedCount();
                            return parent; // 부모 노드로 돌아감
                        }
                    }
                    // RCLCPP_INFO(this->get_logger(), "returning to mama");
                }

                root->increaseVisitedCount();
                return root;
            }

            if(best_child == nullptr){
                RCLCPP_WARN(this->get_logger(),"Best child is Null");
                return root;
            }

            int selected_child_idx = -1;
            for(int i = 0; i < 4; i++){
                if(root->children(i) == best_child){
                    selected_child_idx = i;
                    break;
                }
            }

            return bestChild(best_child, map, depth + 1, root, selected_child_idx, {});
        }

        void saveNodeVisits(std::shared_ptr<QuadtreeNode> node){
            if(node == nullptr) return;

            double real_min_y = node->min_y() * map_.info.resolution + map_.info.origin.position.y;
            double real_min_x = node->min_x() * map_.info.resolution + map_.info.origin.position.x;
            double real_max_y = node->max_y() * map_.info.resolution + map_.info.origin.position.y;
            double real_max_x = node->max_x() * map_.info.resolution + map_.info.origin.position.x;

            std::tuple<double, double, double, double> key =
                { real_min_y, real_min_x, real_max_y, real_max_x };
            node_visits_[key] = node->getVisitedCount();

            for(int i = 0; i < 4; i++){
                if(node->children(i) != nullptr){
                    saveNodeVisits(node->children(i));
                }
            }
        }

        void copyQuadtree(std::shared_ptr<QuadtreeNode> old_node, std::shared_ptr<QuadtreeNode> new_node){
            if(old_node == nullptr){
                // RCLCPP_ERROR(this->get_logger(), "old_node is null, 복사 중단");
                return;
            }
            if(new_node == nullptr){
                // RCLCPP_ERROR(this->get_logger(), "new_node is null, 복사 중단");
                return;
            }


            double new_real_min_y = new_node->min_y() * map_.info.resolution + map_.info.origin.position.y;
            double new_real_min_x = new_node->min_x() * map_.info.resolution + map_.info.origin.position.x;
            double new_real_max_y = new_node->max_y() * map_.info.resolution + map_.info.origin.position.y;
            double new_real_max_x = new_node->max_x() * map_.info.resolution + map_.info.origin.position.x;


            int best_visit_count = 0;
            double best_sync = 0.0;

            for(const auto& [key, visit_count] : node_visits_){
                double old_min_y = std::get<0>(key);
                double old_min_x = std::get<1>(key);
                double old_max_y = std::get<2>(key);
                double old_max_x = std::get<3>(key);

                double sync_min_x = std::max(new_real_min_x, old_min_x);
                double sync_max_x = std::min(new_real_max_x, old_max_x);
                double sync_min_y = std::max(new_real_min_y, old_min_y);
                double sync_max_y = std::min(new_real_max_y, old_max_y);

                if(sync_min_x < sync_max_x && sync_min_y < sync_max_y){
                    double sync_width = sync_max_x - sync_min_x;
                    double sync_height = sync_max_y - sync_min_y;
                    double sync_area = sync_width * sync_height;

                    double new_area = ((new_node->max_x() - new_node->min_x()) * (new_node->max_y() - new_node->min_y()));
                    double old_area = (old_max_x - old_min_x) * (old_max_y - old_min_y);
                    double ratio = static_cast<double>(sync_area) / std::min(new_area, old_area);

                    if(ratio > 0.6 && ratio > best_sync){
                        best_sync = ratio;
                        best_visit_count = visit_count;
                    }
                }
            }
            // RCLCPP_INFO(this->get_logger(),"ratio : %f, count : %d", best_sync, best_visit_count);
            if(best_visit_count > 0){
                new_node->setVisitedCount(best_visit_count);
                RCLCPP_WARN(this->get_logger(),
                           "Node visit count preserved: %d (sync: %.2f%%)",
                           best_visit_count, best_sync * 100.0);
            } else {
                // 적합한 방문 정보가 없으면 이전 방식 사용
                new_node->setVisitedCount(old_node->getVisitedCount());
                // RCLCPP_ERROR(this->get_logger(),
                //    "Fallback to old visit count: %d (no matching nodes found)",
                //     old_node->getVisitedCount());
            }

            for(int i = 0; i < 4; i++){
                if(old_node->children(i) != nullptr){
                    std::shared_ptr<QuadtreeNode> old_child = old_node->children(i);

                    // 새 부모 노드 기준으로 중간점 계산
                    unsigned int mid_y = (new_node->min_y() + new_node->max_y()) / 2;
                    unsigned int mid_x = (new_node->min_x() + new_node->max_x()) / 2;

                    // 4분면에 따라 자식 노드 좌표 계산
                    unsigned int min_y, min_x, max_y, max_x;
                    switch(i) {
                        case 0:
                            min_y = new_node->min_y();
                            min_x = new_node->min_x();
                            max_y = mid_y;
                            max_x = mid_x;
                            break;
                        case 1:
                            min_y = new_node->min_y();
                            min_x = mid_x;
                            max_y = mid_y;
                            max_x = new_node->max_x();
                            break;
                        case 2:
                            min_y = mid_y;
                            min_x = new_node->min_x();
                            max_y = new_node->max_y();
                            max_x = mid_x;
                            break;
                        case 3:
                            min_y = mid_y;
                            min_x = mid_x;
                            max_y = new_node->max_y();
                            max_x = new_node->max_x();
                            break;
                    }
                    std::shared_ptr<QuadtreeNode> new_child = std::make_shared<QuadtreeNode>(min_y, min_x, max_y, max_x);
                    new_node->setChild(i, new_child);

                    copyQuadtree(old_child, new_node->children(i));
                }
            }
        }

        void publishGoalSector(const nav_msgs::msg::OccupancyGrid& map, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y){
            visualization_msgs::msg::Marker sector_line;
            sector_line.header.frame_id = "map";
            sector_line.header.stamp = this->get_clock()->now();
            sector_line.ns = "sector_lines";
            sector_line.id = 0;
            sector_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            sector_line.action = visualization_msgs::msg::Marker::ADD;
            sector_line.scale.x = 0.05;
            sector_line.color.a = 1.0;
            sector_line.color.b = 1.0;
            sector_line.points.clear();

            geometry_msgs::msg::Point p1, p2, p3, p4;
            p1.x = map.info.origin.position.x + min_x * map.info.resolution;
            p1.y = map.info.origin.position.y + min_y * map.info.resolution;

            p2.x = map.info.origin.position.x + max_x * map.info.resolution;
            p2.y = map.info.origin.position.y + min_y * map.info.resolution;

            p3.x = map.info.origin.position.x + max_x * map.info.resolution;
            p3.y = map.info.origin.position.y + max_y * map.info.resolution;

            p4.x = map.info.origin.position.x + min_x * map.info.resolution;
            p4.y = map.info.origin.position.y + max_y * map.info.resolution;

            sector_line.points.push_back(p1);
            sector_line.points.push_back(p2);
            sector_line.points.push_back(p3);
            sector_line.points.push_back(p4);
            sector_line.points.push_back(p1);

            sector_marker_pub_->publish(sector_line);
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

        void search(const nav_msgs::msg::OccupancyGrid& map){
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

        void pubGoal(P goal, const nav_msgs::msg::OccupancyGrid& map){
            PoseManager& pm = PoseManager::getInstance();
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";
            goal_msg.pose.position.y = map.info.origin.position.y + goal.first * map.info.resolution;
            goal_msg.pose.position.x = map.info.origin.position.x + goal.second * map.info.resolution;

            // goal_msg.pose.position.x = 5.0; // for test
            // goal_msg.pose.position.y = 0.0;
            // RCLCPP_INFO(this->get_logger(), "초기 목표 : (x : %f, y : %f)", goal_msg.pose.position.x, goal_msg.pose.position.y);

            pm.setGoalPose(goal_msg);

            // RCLCPP_INFO(this->get_logger(), "수정 목표 : (x : %f, y : %f)", pm.getGoalPose().pose.position.x, pm.getGoalPose().pose.position.y);
            publishGoalPoints(pm.getGoalPose());

            double goal_direction = pm.getDirectionWithRobotGoal();
            goal_direction = std::fmod(goal_direction + M_PI, 2 * M_PI) - M_PI;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_direction);
            pm.getGoalPose().pose.orientation = tf2::toMsg(q);
            goal_pub_->publish(pm.getGoalPose());
        }


    private:
        std::map<std::tuple<double, double, double, double>, int> node_visits_;

        int fail_count = 0;

        bool split_route = false;
        bool first_move = false;
        bool expanded = false;
        bool explore = true; // 동작 완료 후 활성 , 이동 없는 테스트는 비활성
        bool expand_map_sq = false;
        bool fail_flag = false;
        bool force_move = false;
        bool end_explore = false;
        P current_pixel_goal_;

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
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sector_marker_pub_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration_map>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
