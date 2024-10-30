#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <nav_msgs/msg/path.hpp>
// #include <nav2_costmap_2d/costmap_2d_ros.hpp>


#include <queue>
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <set>
#include <random>

class Exploration : public rclcpp::Node
{
    public:

        Exploration() : rclcpp::Node("Exploration_Node"){

            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&Exploration::map_callback, this, std::placeholders::_1));

            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/pose", 10, std::bind(&Exploration::pose_callback, this, std::placeholders::_1));

            marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
                "/marker", 1, std::bind(&Exploration::marker_callback, this, std::placeholders::_1));

            goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

            this->setNewGoal();
            timer_= this->create_wall_timer(std::chrono::seconds(3), std::bind(&Exploration::explore, this));

        }


    protected:
        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            last_pose_ = pose_;
            pose_ = *msg;
            // RCLCPP_INFO_SKIPFIRST(this->get_logger(), "pose_sub");
        }

        void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
            if(msg->markers.back().color.r == 1.0)   Collision = true;
            else Collision = false;
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if(msg->data.empty()){
                RCLCPP_WARN(this->get_logger(), "Empty map");
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "get map");
            map_ = msg;
        }

        double last_dist = 0.0;
        bool isGoalReached(const geometry_msgs::msg::PoseWithCovarianceStamped &current_pose, const geometry_msgs::msg::PoseStamped &goal_pose){
            if(Collision){
                RCLCPP_INFO(this->get_logger(), "Collision ahead. Need new goal.");
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "Verifying that the move is complete");

            if(goal_pose.pose.position.x == 0 && goal_pose.pose.position.y == 0){
                RCLCPP_INFO(this->get_logger(), "goal is 0,0");
                return true;
            }

            double dx = current_pose.pose.pose.position.x - goal_pose.pose.position.x;
            double dy = current_pose.pose.pose.position.y - goal_pose.pose.position.y;
            double distance = sqrt(dx * dx + dy * dy);

            if (last_dist == distance) {
                RCLCPP_INFO(this->get_logger(), "Robot is stuck, resetting goal.");
                v[{goal_pose.pose.position.y, goal_pose.pose.position.y}] = true;
                return true; // 목표 재설정 필요
            }
            last_dist = distance;
            double threshold = 0.5;
            std::cout << "distance : " << distance << " threshold : " << threshold << std::endl;
            return (distance < threshold);
        }


        geometry_msgs::msg::PoseStamped original_goal;

        void setNewGoal(){

            RCLCPP_INFO(this->get_logger(), "Set New goal sequence");
            goal.header.stamp = this->now();
            goal.header.frame_id = "map";
            if (!map_) {
                RCLCPP_WARN(this->get_logger(), "Map is not yet received, skipping goal setting.");
                return;
            }
            auto &map = *map_;

            unsigned int current_x = (pose_.pose.pose.position.x - map.info.origin.position.x) / map.info.resolution;
            unsigned int current_y = (pose_.pose.pose.position.y - map.info.origin.position.y) / map.info.resolution;

            // std::cout << "now pose x : " << pose_.pose.pose.position.x << "\n"
            //           << "now pose y : " << pose_.pose.pose.position.y << "\n"
            //           << "x on map : "   << current_x << "\n"
            //           << "y on map : "   << current_y << "\n"
            // << std::endl;

            std::vector<std::vector<double>> distances = dijkstra(current_x, current_y);


            auto goal_ = findBestGoal(distances);

            if(goal_.first != 0 && goal_.second != 0){
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis(-0.5, 0.5); // -0.5 ~ 0.5 사이의 무작위 실수

                double random_offset_x = dis(gen);
                double random_offset_y = dis(gen);

                // 원래 목표 좌표 저장 (변위 적용 전)
                original_goal.pose.position.x = map.info.origin.position.x + goal_.first * map.info.resolution;
                original_goal.pose.position.y = map.info.origin.position.y + goal_.second * map.info.resolution;
                // original_goal.pose.position.z = 0.0;

                goal.pose.position.x = original_goal.pose.position.x + random_offset_x;
                goal.pose.position.y = original_goal.pose.position.y + random_offset_y;

                // goal_pose.pose.orientation.w = 1.0;
                RCLCPP_INFO(this->get_logger(), "Attempting to set new goal: x=%f, y=%f", goal.pose.position.x, goal.pose.position.y);
                v[{goal.pose.position.y, goal.pose.position.x}] = true;
                goal_pub_->publish(goal);
            }
        }

        std::vector<std::vector<double>> dijkstra(unsigned int st_x, unsigned int st_y){
            unsigned int width = map_->info.width;
            unsigned int height = map_->info.height;
            RCLCPP_INFO(this->get_logger(), "dijkstra algorithm activating");

            std::vector<std::vector<double>> dist(height, std::vector<double>(width, std::numeric_limits<double>::infinity()));
            std::priority_queue<std::tuple<double, unsigned int, unsigned int>, std::vector<std::tuple<double, unsigned int, unsigned int>>, std::greater<>> pq;

            pq.emplace(0.0, st_x, st_y);
            dist[st_y][st_x] = 0;
            // int t = 1;
            int failure_count = 0;  // 탐색 실패 횟수를 추적
            const int max_failures = 50;  // 최대 실패 허용 횟수
            std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};

            while (!pq.empty()){
                auto [current_dist, x, y] = pq.top();
                pq.pop();
                int count = 0;
                if(current_dist > dist[y][x]) continue;

                for(const auto& [dx, dy] : directions){
                    int nx = x + dx;
                    int ny = y + dy;
                    if(isUnexploredAndNavigable(nx, ny)){
                        double new_dist = sqrt(std::pow(std::abs(nx - int(st_x)), 2) + std::pow(std::abs(ny - int(st_y)), 2));
                        // RCLCPP_INFO(this->get_logger(), "Enter first if, new dist : %f , current dist : %f", new_dist, current_dist);
                        if(Collision){
                            RCLCPP_ERROR(this->get_logger(), "Collision.");
                            Collision = false;
                            continue;
                        }

                        if(new_dist < dist[ny][nx]){
                            dist[ny][nx] = new_dist;
                            // RCLCPP_INFO(this->get_logger(), "distance set : %f", new_dist);
                            // std::cout << "set new_dist : " << ny << nx << new_dist << std::endl;
                            // visited[{ny, nx}] = true;
                            pq.emplace(new_dist, nx, ny);
                            count = 0;
                        }
                    }
                    else{
                        // RCLCPP_WARN(this->get_logger(), "Count : %d", count);
                        // RCLCPP_WARN(this->get_logger(), "t : %d", t);
                        count++;
                    }
                    if(count == 8){
                        failure_count++;
                        if (failure_count >= max_failures) {
                            RCLCPP_WARN(this->get_logger(), "Pathfinding failed after %d attempts, terminating.", failure_count);
                            return dist;
                        }
                    }
                }
            }
            return dist;
        }

        std::map<std::pair<unsigned int,unsigned int>, bool> visited;

        std::pair<unsigned int, unsigned int> findBestGoal(const std::vector<std::vector<double>> distances){
            // printVisitedMap();
            unsigned int best_x = 0, best_y = 0;
            double max_distance = 0;
            // std::cout << "height cell : " << distances.size() << " width cell : " << distances[0].size() << std::endl;
            for(unsigned int y = 0; y < distances.size(); y++){
                for(unsigned int x = 0; x < distances[y].size(); x++){
                    if(visited[{y, x}] || distances[y][x] == std::numeric_limits<double>::infinity()) continue;
                    if(distances[y][x] > max_distance && !isSurroundingExplored(y, x)){
                        max_distance = distances[y][x];
                        best_y = y;
                        best_x = x;
                    }
                }
            }
            // 유효한 목표가 없는 경우 처리
            if (max_distance == 0) {
                RCLCPP_ERROR(this->get_logger(), "max_distance = 0");
                // 예외 처리 로직 (예: 기본값 반환, 오류 메시지 출력 등)
                return {0, 0};
            }
            visited[{best_y,best_x}] = true;
            return {best_x, best_y};
        }

        bool isSurroundingExplored(unsigned int y, unsigned int x) {
            std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

            for (const auto& [dx, dy] : directions) {
                if (visited[{y + dy, x + dx}]) {
                    RCLCPP_ERROR(this->get_logger(), "Already visited.");
                    return true; // 주변에 방문한 좌표가 있을 경우
                }
            }
            return false;
        }

        void explore(){
            if (!map_) {
                RCLCPP_WARN(this->get_logger(), "Map is not yet received, skipping exploration.");
                return;
            }

            if(isGoalReached(pose_, goal)){
                setNewGoal();
                RCLCPP_INFO(this->get_logger(), "new goal set");
            }
        }
        std::map<std::pair<unsigned int, unsigned int>, bool> v;
        bool isUnexploredAndNavigable(unsigned int x, unsigned int y) {

            if (x >= map_->info.width - 10  || y >= map_->info.height - 10 || x <= 10 || y <= 10 ) {
                // RCLCPP_WARN(this->get_logger(), "Out of range (1)");
                return false;
            }

            double x_low_limit = map_->info.origin.position.x;
            double x_high_limit = map_->info.origin.position.x + map_->info.width * map_->info.resolution;
            double y_low_limit = map_->info.origin.position.y;
            double y_high_limit = map_->info.origin.position.y + map_->info.height * map_->info.resolution;

            double temp_x = map_->info.origin.position.x + x * map_->info.resolution;
            double temp_y = map_->info.origin.position.y + y * map_->info.resolution;

            // std::cout << x_low_limit << "<" <<temp_x << "<" <<x_high_limit << " , "<< y_low_limit << "<" <<temp_y << "<" << y_high_limit

            //           << std::endl;
            if(v[{temp_y, temp_x}] && !v.empty()){

                // RCLCPP_WARN(this->get_logger(), "Visited");
                // RCLCPP_WARN(this->get_logger(), "%f , %f", temp_x, temp_y);
                return false;
            }

            if(temp_x >= x_high_limit - 1 || temp_x <= x_low_limit + 1 || temp_y >= y_high_limit - 1 || temp_y <= y_low_limit + 1){
                // RCLCPP_WARN(this->get_logger(), "Out of range (2)");
                return false;
            }

            int index = y * map_->info.width + x;
            // -1: 미탐색, 0: 비어 있음, 100: 장애물
            if(map_->data[index] == -1 ){//|| map_->data[index] == 0){
                // RCLCPP_INFO(this->get_logger(), "Find new goal : %d, %d", x, y);
                // RCLCPP_INFO(this->get_logger(), "Find new goal : %f, %f", temp_x, temp_y);
                return true;
            }else{
                return false;
            }
        }

        bool Collision = false;

        geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
        geometry_msgs::msg::PoseStamped goal;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        geometry_msgs::msg::PoseWithCovarianceStamped pose_;


        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exploration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
