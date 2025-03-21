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
        P last_goal_pixel_coord_;
        PoseManager() {}

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
            if (last_pose_.pose.position.x - pose.pose.position.x < 0.1 &&
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

            double theta = getRobotdirection();

            double dx = goal_x - robot_x;
            double dy = goal_y - robot_y;

            double rotate_x = cos(-theta) * dx - sin(-theta) * dy;
            double rotate_y = sin(-theta) * dx + cos(-theta) * dy;

            return atan2(rotate_y, rotate_x);
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
        LaserManager() {}
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
};

class SectorManager{
    public:
        SectorManager() {}

        int sectorSize(){
            return (int)sector_set.size();
        }
        void sectorClear(){
            sector_set.clear();
        }
        std::set<int>& getSectorSet(){
            return sector_set;
        }
        void sectorInsert(int sector){
            sector_set.insert(sector);
        }

        struct sector_border{
            int sector;
            P min_border;
            P max_border;
        };


        bool sectorControl(int current_sector, bool reset, bool fix_sector){
            int sector = current_sector;
            if(reset){
                last_sector = 0;
                return false;
            }
            if(fix_sector){
                sector_set.clear();
                if(last_sector != sector){
                    // RCLCPP_INFO(this->get_logger(),"Not same sector");
                    return true;
                }else if(last_sector == sector){
                    // RCLCPP_INFO(this->get_logger(),"Same sector");
                    return false;
                }
            }else{
                if(last_sector == 0){
                    last_sector = sector;
                    return false;
                }
                if(last_sector != sector){
                    last_sector = sector;
                    return false; // different sector
                }else{
                    std::cout<< "  sadfadsf  " << last_sector<< "asdfass" <<std::endl;
                    return true; // same sector
                }
            }
            return false;
        }

        sector_border checkSector(const nav_msgs::msg::OccupancyGrid& map, P pixel_coord){
            double origin_x = map.info.origin.position.x;
            double origin_y = map.info.origin.position.y;
            double resolution = map.info.resolution;
            int sector = 0;


            unsigned int std_pixel_x = -origin_x / resolution;
            unsigned int std_pixel_y = -origin_y / resolution;
            P std_pixel = {std_pixel_y, std_pixel_x};
            // RCLCPP_INFO(this->get_logger(), "std_pixel : (%d, %d)", std_pixel.first, std_pixel.second);
            std::pair<int, int> temp;
            temp = {pixel_coord.second - std_pixel.second, pixel_coord.first - std_pixel.first};

            if(temp.first == 0) temp.first = 1;
            if(temp.second == 0) temp.second = 1;

            if(temp.first > 0 && temp.second > 0){
                sector = 1;
                P min_border = std_pixel;
                P max_border = {map.info.height, map.info.width};
                return {sector, min_border, max_border};
            }
            else if(temp.first < 0 && temp.second > 0){
                sector = 2;
                P min_border = {std_pixel_y, 0};
                P max_border = {map.info.height, std_pixel_x};
                return {sector, min_border, max_border};
            }
            else if(temp.first < 0 && temp.second < 0){
                sector = 3;
                P min_border = {0, 0};
                P max_border = std_pixel;
                return {sector, min_border, max_border};
            }
            else if(temp.first > 0 && temp.second < 0){
                sector = 4;
                P min_border = {0, std_pixel_x};
                P max_border = {std_pixel_y, map.info.width};
                return {sector, min_border, max_border};
            }
            return {0, {0, 0}, {0, 0}};
            // RCLCPP_WARN(this->get_logger(), "Sector : %d", sector);
        }

    private:
        int last_sector;
        std::set<int> sector_set;
};

class FrontierExplorer{
    public:
        FrontierExplorer(rclcpp::Node* node, MapManager& mm, PoseManager& pm)
            : node_(node), mm_(mm), pm_(pm), sm_() {}

            P setGoalBasedonFrontier(const nav_msgs::msg::OccupancyGrid& map, PoseManager& PM, bool fail_flag, bool expanded_flag){
                const int directions[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, -1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
                RCLCPP_INFO(node_->get_logger(),"set goal based on frontier");
                unsigned int map_width = (unsigned int)map.info.width;
                unsigned int map_height = (unsigned int)map.info.height;
                geometry_msgs::msg::PoseStamped robot_pose = PM.getLastPose();

                if(sm_.sectorSize() == 4){
                    RCLCPP_INFO(node_->get_logger(),"Sector reset");
                    sm_.sectorControl(0, true, false);
                    sm_.sectorClear();
                    MapManager::getInstance().clearSingletonset();
                    return {0, 0};
                }

                std::vector<P> frontier;

                for(unsigned int x = 0; x < map_width; x++){
                    for(unsigned int y = 0; y < map_height; y++){
                        if(map.data[Utility::pixelcoordtoidx(y, x, map_width)] != 0 ){
                            continue;
                        }
                        for(const auto& dir : directions){
                            unsigned int nx = x + dir[0];
                            unsigned int ny = y + dir[1];
                            if(nx > 10 && ny > 10 && nx < map_width - 10 && ny < map_height - 10){
                                if(map.data[Utility::pixelcoordtoidx(ny, nx, map_width)] >= 90){
                                    frontier.push_back({y, x});
                                    break;
                                }
                            }
                        }
                    }
                }
                if(frontier.empty()){
                    RCLCPP_INFO(node_->get_logger(),"frontier empty");
                    return {0, 0} ;
                }
                else{
                    int size = (int)frontier.size();
                    RCLCPP_INFO(node_->get_logger(),"frontier size : %d", size);
                    P goal;

                    unsigned int pixel_y = (robot_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
                    unsigned int pixel_x = (robot_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
                    sort(frontier.begin(), frontier.end(), [&](const P& a, const P& b){
                        if (Utility::distance(a, {pixel_y, pixel_x}) == Utility::distance(b, {pixel_y, pixel_x})) {
                            if (a.second == b.second) {
                                return a.first < b.first;
                            }
                            return a.second < b.second;
                        }
                        return Utility::distance(a, {pixel_y, pixel_x}) > Utility::distance(b, {pixel_y, pixel_x});
                    });
                    double min_dist = 100.0; //temp

                    static bool last_expanded = false;
                    SectorManager::sector_border sector;
                    if(fail_flag){
                        expanded_flag = last_expanded;
                    }
                    for(int idx = 0; idx < size; idx++){
                        goal = frontier[idx];
                        if(idx == size - 1){
                            goal = frontier[0];
                            RCLCPP_INFO(node_->get_logger(),"All goal is gear");
                            break;
                        }
                        double dist = Utility::distance(goal, {pixel_y, pixel_x});
                        if(dist > min_dist){
                            if(MapManager::getInstance().getSingletonset().count(goal) == 1){
                                continue;
                            }
                            sector = sm_.checkSector(map, goal);

                            if(expanded_flag){
                                if(sm_.sectorControl(sector.sector, false, true)){
                                    continue;
                                }
                            }else{
                                if(sm_.getSectorSet().find(sector.sector) != sm_.getSectorSet().end()){
                                    continue;
                                }
                                if(sm_.sectorControl(sector.sector, false, false)){
                                    continue;
                                }
                            }
                            RCLCPP_INFO(node_->get_logger(),"expanded_flag : %d", expanded_flag);
                            RCLCPP_INFO(node_->get_logger(),"sector : %d", sector.sector);
                            // RCLCPP_INFO(node_->get_logger(),"index : %d , size : %d", idx, size);
                            RCLCPP_INFO(node_->get_logger(),"Goal : (%d, %d)", goal.first, goal.second);
                            RCLCPP_INFO(node_->get_logger(),"dist : %f", dist);
                            break;
                        }
                    }
                    MapManager::getInstance().addOSingletonset(goal.first, goal.second);

                    {
                        auto offset = findNearGear(map, goal);

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
                    }
                    int border_offset = 15;
                    goal.first = std::clamp(goal.first, sector.min_border.first + border_offset, sector.max_border.first - border_offset);
                    goal.second = std::clamp(goal.second, sector.min_border.second + border_offset, sector.max_border.second - border_offset);

                    last_expanded = expanded_flag;

                    RCLCPP_INFO(node_->get_logger(),"Fix Goal : (%d, %d)", goal.first, goal.second);
                    RCLCPP_INFO(node_->get_logger(),"frontier end");
                    sm_.sectorInsert(sector.sector);

                    for(auto i : sm_.getSectorSet()){
                        RCLCPP_INFO(node_->get_logger(),"sector_set : %d", i);
                    }
                    return goal;
                }
            }

    private:
        rclcpp::Node* node_;
        MapManager& mm_;
        PoseManager& pm_;
        SectorManager sm_;

        std::pair<int, int> findNearGear(const nav_msgs::msg::OccupancyGrid map, const P& goal){
            unsigned int map_width = (unsigned int)map.info.width;
            unsigned int map_height = (unsigned int)map.info.height;
            for(int dy = -10; dy <= 10; dy++){
                for(int dx = -10; dx <= 10; dx++){
                    unsigned int check_y = goal.first + dy;
                    unsigned int check_x = goal.second + dx;
                    if(check_y < map_height && check_x < map_width &&
                        map.data[Utility::pixelcoordtoidx(check_y, check_x, map_width)] == 100){
                        return {dy, dx};
                    }
                }
            }
            return {0, 0};
        }
};
/*
class ObstacleAvoider {
    public:
    ObstacleAvoider(rclcpp::Node* node) : node_(node) {
            cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }

        bool checkSurroundingsAndEvacuate(const nav_msgs::msg::OccupancyGrid& map) {
            PoseManager& PM = PoseManager::getInstance();
            LaserManager& LM = LaserManager::getInstance();
            bool done = false;

            auto heading = findSafeHeading(map, PM, LM);
            if(checkSurrounding(0.5)){
                done = evacuate(heading);
            // }else{
            //     done = goAnotherway(map);
            }
            return done;
        }

    private:

        bool checkSurrounding(double threshold){
            sensor_msgs::msg::LaserScan laser_msg = LaserManager::getInstance().getLaserMsg();
            for(size_t i = 0; i < laser_msg.ranges.size(); i++){
                if(laser_msg.ranges[i] < threshold){
                    return true;
                }
            }
            return false;
        }


        std::pair<double, double> findSafeHeading(const nav_msgs::msg::OccupancyGrid& map,
                               PoseManager& PM,
                               LaserManager& LM){

            double theta = PM.getRobotdirection();
            int digree = theta * 180 * M_1_PI;

            double last_max_range = std::numeric_limits<double>::min();
            double last_min_range = std::numeric_limits<double>::max();
            int max_range_index = std::numeric_limits<int>::min();
            int min_range_index = std::numeric_limits<int>::max();

            for(int index = 0; index < (int)LM.getLaserMsg().ranges.size(); index++){
                double range = LM.getLaserMsg().ranges[index];
                if(range > last_max_range){
                    last_max_range = range;
                    max_range_index = index;
                }
                if(range < last_min_range){
                    last_min_range = range;
                    min_range_index = index;
                }
            }
            int max_range_dir = (max_range_index + 180) % 360;
            if(max_range_dir > 180){
                max_range_dir -= 360;
            }
            int min_range_dir = (min_range_index + 180) % 360;
            if(min_range_dir > 180){
                min_range_dir -= 360;
            }




            double best_heading = max_range_dir * M_PI / 180;
            double obstacle_heading = min_range_dir * M_PI / 180;
            // RCLCPP_INFO(node_->get_logger(), "robot heading : %d (digree), %f (radian)", digree, theta);
            // RCLCPP_INFO(node_->get_logger(), "최소 측정 거리 방향: %d (digree), %f (radian)", min_range_dir, obstacle_heading);
            // RCLCPP_INFO(node_->get_logger(), "권장 회피기동 방향: %d (digree), %f (radian) ", max_range_dir, best_heading );

            return {best_heading, obstacle_heading};
        }

        bool evacuate(std::pair<double, double> heading) {
            RCLCPP_INFO(node_->get_logger(), "장애물 피하기 시작");
            geometry_msgs::msg::Twist cmd_vel;
            double best_heading = heading.first;
            double obstacle_heading = heading.second;
            double robot_heading = PoseManager::getInstance().getRobotdirection();
            bool done = false;
            rclcpp::Rate loop_rate(10); // 10Hz

            if(obstacle_heading < tf2Radians(60) && obstacle_heading > tf2Radians(-60)){
                RCLCPP_INFO(node_->get_logger(), "장애물이 정면에 있습니다.");
                cmd_vel.linear.x = -0.2;
                cmd_vel.angular.z = 0.0;
            }
            else if(obstacle_heading < tf2Radians(120) && obstacle_heading > tf2Radians(60))
                RCLCPP_INFO(node_->get_logger(), "장애물이 왼쪽에 있습니다."); // 미구현
            else if(obstacle_heading > tf2Radians(-120) && obstacle_heading < tf2Radians(-60))
                RCLCPP_INFO(node_->get_logger(), "장애물이 오른쪽에 있습니다."); // 미구현
            else if(obstacle_heading > tf2Radians(120) || obstacle_heading < tf2Radians(-120)){
                RCLCPP_INFO(node_->get_logger(), "장애물이 뒤에 있습니다.");
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.0;
            }
            geometry_msgs::msg::PoseStamped initial_pose = PoseManager::getInstance().getLastPose();
            geometry_msgs::msg::PoseStamped initial_pose_copy = initial_pose;

            while(rclcpp::ok() && !done){
                cmd_pub_->publish(cmd_vel);
                rclcpp::sleep_for(std::chrono::milliseconds(100));

                geometry_msgs::msg::PoseStamped current_pose = PoseManager::getInstance().getLastPose();
                double distance = sqrt(pow(current_pose.pose.position.x - initial_pose_copy.pose.position.x, 2) +
                                       pow(current_pose.pose.position.y - initial_pose_copy.pose.position.y, 2));
                RCLCPP_INFO(node_->get_logger(), "distance : %f", distance);
                if(!checkSurrounding(0.5) && distance > 0.5){
                    done = true;
                    RCLCPP_INFO(node_->get_logger(), "장애물 회피 성공");
                }
                loop_rate.sleep();
            }

            RCLCPP_INFO(node_->get_logger(), "장애물 회피 성공");
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_pub_->publish(cmd_vel);
            // 방향이 반대면 후진
            // 방향이 일치할때까지 조향
            // 끼임상태 탈출 할 때 까지 기동

            return done;
        }

        bool goAnotherway(const nav_msgs::msg::OccupancyGrid& map){ // 끼이진 않았는데 경로찾기가 실패했을 경우 다른 위치에서 경로찾기 위함
            // 마지막 설정 목표에 도달하기 위해 주변 장애물을 피해 다른 경로로 이동이 목표

            geometry_msgs::msg::Twist cmd_vel;
            PoseManager& PM = PoseManager::getInstance();

            double goal_heading = PM.getDirectionWithRobotGoal(0.0, 0.0);
            double robot_heading = PM.getRobotdirection();
            double goal_distance = PM.getdistancewithRobotGoal();
            bool done = false;

            auto safe_heading = findSafeHeading(map, PM, LaserManager::getInstance());

        }

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    };
*/

class NotSearchedFirstExplorer{
    public:
        NotSearchedFirstExplorer(rclcpp::Node* node)
            : node_(node) {}

        P setGoal(const nav_msgs::msg::OccupancyGrid& map){
            MapManager& mm = MapManager::getInstance();
            PoseManager& PM = PoseManager::getInstance();
            geometry_msgs::msg::PoseStamped robot_pose = PM.getLastPose();

            RCLCPP_INFO(node_->get_logger(),"set goal");
            P near_goal = {0, 0};
            unsigned int pixel_y = (robot_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;
            unsigned int pixel_x = (robot_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
            P current_pixel_coord = {pixel_y, pixel_x};

            double min_dist = std::numeric_limits<double>::max();
            for(auto &itr : mm.getVisited()){
                if(itr.second == false) {
                    // RCLCPP_INFO(this->get_logger(),"setgoal2");
                    if(itr.first.first >= map.info.height || itr.first.second >= map.info.width){ continue; }
                    unsigned int temp_idx = Utility::pixelcoordtoidx(itr.first.first, itr.first.second, map.info.width);
                    if (temp_idx >= map.data.size()) {
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
            best_heading_dir_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("safe_way", 10);
            laser_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("laser_line", 10);
            goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_points", 10);
            // obstacle_avoider_ = std::make_shared<ObstacleAvoider>(this);
            frontier_explorer_ = std::make_shared<FrontierExplorer>(this, MapManager::getInstance(), PoseManager::getInstance());
            not_searched_first_explorer_ = std::make_shared<NotSearchedFirstExplorer>(this);
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Exploration_map::timer_callback, this));

        }

    protected:

        void timer_callback(){
            // RCLCPP_INFO(this->get_logger(),"System status : map = %d , pose = %d, jobdone = %d", i_sub_map, i_sub_pose, jobdone);
            if(explore && map_data_stable && searching && !force_move){
                explore = false;
                searching = false;
                P goal;

                if(expand_map_sq){
                    goal = frontier_explorer_->setGoalBasedonFrontier(map_, PoseManager::getInstance(), fail_flag, expanded);
                    if(goal.first == 0 && goal.second == 0) expand_map_sq = false;
                    expanded = false;
                    fail_flag = false;
                }
                else{
                    goal = not_searched_first_explorer_->setGoal(map_);
                    if(goal == P{0, 0}){
                        RCLCPP_INFO(this->get_logger(), "All point visited");
                        end_explore = true;
                    }
                    current_pixel_goal = goal;
                }
                if(!end_explore){
                    pubGoal(goal, map_);
                }
            }
            if(!expand_map_sq && map_data_stable)
                excludeObstacle(map_);
        }

        void btl_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg){
            for(size_t i = 0; i < msg->event_log.size(); i ++){
                std::string node_name_ = msg->event_log[i].node_name;
                std::string current_status_ = msg->event_log[i].current_status;
                std::string previous_status_ = msg->event_log[i].previous_status;
                fail_count = std::clamp(fail_count, 0, 20);
                if(node_name_ == "ComputePathToPose" && current_status_ == "FAILURE"){
                    RCLCPP_WARN(this->get_logger(), "Failed to compute pose. Failcount : %d", fail_count);
                    if(!expand_map_sq){
                        fail_count++;
                    }
                    fail_flag = true;

                    rclcpp::Time fail_time = this->now();
                    if(fail_time.seconds() - pub_time.seconds() < 1 ){
                        force_move_count++;
                    }else{
                        force_move_count = 0;
                    }
                    if(force_move_count == force_move_threshold){
                        RCLCPP_WARN(this->get_logger(), "Force move");
                        // force_move = true;
                        // explore = false;
                    }
                    MapManager::getInstance().setVisited(current_pixel_goal, true);
                }
                else if(node_name_ == "FollowPath" && current_status_ == "SUCCESS"){
                    RCLCPP_INFO(this->get_logger(), "SUCCEED to move.");
                    fail_count = 0;
                }
                else if(node_name_ == "FollowPath" && current_status_ == "RUNNING"){

                }
                else if(node_name_ == "FollowPath" && current_status_ == "FAILURE" && previous_status_ == "RUNNING"){
                    RCLCPP_WARN(this->get_logger(), "Failed to move.");

                }
                if(node_name_ == "RateController" && current_status_ == "IDLE"){
                    RCLCPP_INFO(this->get_logger(), "READY 2 MOVE");

                    if(!expand_map_sq){
                        MapManager::getInstance().setVisited(current_pixel_goal, true);
                    }
                    if(fail_count == 20){
                        fail_count = 0;
                    }
                    clearAround(map_, MapManager::getInstance(), PoseManager::getInstance());

                    explore = force_move ? false : true;
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

            map_ = *msg;
            if (expand_map_sq && isMapExpanding(map_)){
                RCLCPP_WARN(this->get_logger(), "지도 확장");
                transformCoord(map_);  // 지도 확장 시에만
                expanded = true;
            }


            search(map_); // map.data 에 따른 미탐색구역 visited 맵으로 동기화

            searching = true;
        }

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            PoseManager::getInstance().setLastPose(*msg);

            // if(force_move){
            //     force_move = !obstacle_avoider_->checkSurroundingsAndEvacuate(map_);
            // }
            if(!expand_map_sq){
                publishVisitedPoints(map_); // visited 맵 시각화
            }
        }



        bool isMapExpanding(const nav_msgs::msg::OccupancyGrid& map){
            static unsigned int last_width = map.info.width;
            static unsigned int last_height = map.info.height;
            static double last_origin_x = map.info.origin.position.x;
            static double last_origin_y = map.info.origin.position.y;

            double width_height_threshold = 1.5;
            double origin_threshold = 1.0;

            if(last_width - map.info.width > width_height_threshold ||
               last_height - map.info.height > width_height_threshold){
                last_width = map.info.width;
                last_height = map.info.height;
                return true;
            }
            if(last_origin_x - map.info.origin.position.x > origin_threshold ||
               last_origin_y - map.info.origin.position.y > origin_threshold){
                last_origin_x = map.info.origin.position.x;
                last_origin_y = map.info.origin.position.y;
                return true;
            }
            return false;
        }

        void clearAround(const nav_msgs::msg::OccupancyGrid& map,
                         MapManager& mm,
                         PoseManager& PM){
            geometry_msgs::msg::PoseStamped robot_pose = PM.getLastPose();
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
                MapManager::getInstance().addOSingletonset(pixel_y, pixel_x);

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

        void ObstacleFloodFill(const std::vector<std::vector<P>>& obstacles,
                       const nav_msgs::msg::OccupancyGrid& map){
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

                for (const auto& point : obstacle) {
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
                        if(y >= map.info.height || x >= map.info.width){
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

            ObstacleFloodFill(obstacles, map);

            // RCLCPP_INFO(this->get_logger(),"scan obstacle done");

        }

        void transformCoord(const nav_msgs::msg::OccupancyGrid& map){
            std::map<P, bool> transformed_visited;
            std::set<P> transformed_singleton_set;
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

            for(const auto& p : mm.getSingletonset()){
                transformed_singleton_set.insert(transformPoints(p));
            }

            last_origin_x = map.info.origin.position.x;
            last_origin_y = map.info.origin.position.y;
            mm.clearVisited();
            mm.clearSingletonset();
            mm.getVisited() = transformed_visited;
            mm.getSingletonset() = transformed_singleton_set;
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
            points_unexplored.color.r = 1.0;
            points_unexplored.color.g = 0.0;

            points_unexplored.points.clear();

            for (const auto& itr : MapManager::getInstance().getVisited()) {
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


            static std::vector<int8_t> last_map_data;

            if(last_map_data.empty()){
                last_map_data = map.data;
            }

            unsigned int height = map.info.height;
            unsigned int width = map.info.width;


            for(unsigned int y = 10; y < height - 10; y++){
                for(unsigned int x = 10; x < width - 10; x++){
                    unsigned int idx = Utility::pixelcoordtoidx(y, x, width);
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
            last_map_data = map.data;
            // RCLCPP_INFO(this->get_logger(),"search done");
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

            double robot_x = PM.getLastPose().pose.position.x;
            double robot_y = PM.getLastPose().pose.position.y;
            double goal_x = PM.getGoalPose().pose.position.x;
            double goal_y = PM.getGoalPose().pose.position.y;


            double angle_to_goal = std::atan2(goal_y - robot_y, goal_x - robot_x);

            double robot_theta = PM.getRobotdirection();
            double relative_angle_to_goal = angle_to_goal - robot_theta;

            relative_angle_to_goal = std::fmod(relative_angle_to_goal + M_PI, 2 * M_PI) - M_PI;

            double laser_angle_min = LM.getLaserMsg().angle_min;
            double laser_angle_increment = LM.getLaserMsg().angle_increment;
            int index = (relative_angle_to_goal - laser_angle_min) / laser_angle_increment;

            publishscanPoints(LM.getLaserMsg(), index, relative_angle_to_goal);
            RCLCPP_INFO(this->get_logger(), "index at goal direction : %d", index);

            double range_at_angle = LM.getLaserMsg().ranges[index];

            double dist_goal = PM.getdistancewithRobotGoal();
            RCLCPP_WARN(this->get_logger(), "목표까지의 거리 : %f ", dist_goal);
            RCLCPP_WARN(this->get_logger(), "측정 거리 : %f ", range_at_angle);

            if (std::isinf(range_at_angle)){
                RCLCPP_INFO(this->get_logger(), "No valid data at this angle.");
                double angle_rad = index * M_PI / 180.0;
                std::pair<double, double> another_goal = setGoal_Angleanddist(map, angle_rad, 5); // 10 = temp;
                PM.getGoalPose().pose.position.x = another_goal.second;
                PM.getGoalPose().pose.position.y = another_goal.first;

                publishGoalPoints(PM.getGoalPose());
                return;
            }
            double threshold = -0.1;
            if(dist_goal - range_at_angle > threshold){ // 목표지점이 장애물 너머일 경우
                RCLCPP_INFO(this->get_logger(), "is over obstacle");

                setGoalbesideObstacle(LM.getLaserMsg(), map, index, dist_goal, fail_count);

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
            if(!expand_map_sq){
                isOverObstacle(map, pm, LaserManager::getInstance(), fail_count);
            }

            RCLCPP_INFO(this->get_logger(), "수정 목표 : (x : %f, y : %f)", pm.getGoalPose().pose.position.x, pm.getGoalPose().pose.position.y);
            publishGoalPoints(pm.getGoalPose());

            double goal_direction = pm.getDirectionWithRobotGoal();
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
        int temp_i = 0;

        bool expanded = false;
        bool explore = true; // 동작 완료 후 활성 , 이동 없는 테스트는 비활성
        bool searching = false;
        bool map_data_stable = false;
        bool expand_map_sq = true;
        bool fail_flag = false;
        bool force_move = false;
        bool end_explore = false;
        P last_fail_point;
        P current_pixel_goal;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // std::shared_ptr<ObstacleAvoider> obstacle_avoider_;
        std::shared_ptr<FrontierExplorer> frontier_explorer_;
        std::shared_ptr<NotSearchedFirstExplorer> not_searched_first_explorer_;


        nav_msgs::msg::OccupancyGrid map_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr btl_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visit_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_heading_dir_pub_;
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
