#include "exploration_node.hpp"

ExplorationNode::ExplorationNode() : Node("exploration_node"){
    RCLCPP_INFO(this->get_logger(), "Node init");


    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&ExplorationNode::initialpose_callback, this, std::placeholders::_1));

}

void ExplorationNode::initializeCostmap(){

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("exploration","","");
    if (costmap_ros_) {
        RCLCPP_INFO(this->get_logger(), "Costmap initialized successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize costmap");
    }
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());
}

std::vector<geometry_msgs::msg::PoseStamped> ExplorationNode::dijkstra(const nav2_costmap_2d::Costmap2D* costmap,
                                                                       const geometry_msgs::msg::PoseStamped& start,
                                                                       const geometry_msgs::msg::PoseStamped& goal)
{
    std::priority_queue<std::pair<double, std::pair<unsigned int, unsigned int>>, std::vector<std::pair<double, std::pair<unsigned int, unsigned int>>>, std::greater<>> pq;
    std::unordered_map<int, double> distances;
    std::unordered_map<int, std::pair<int, int>> previous;

    unsigned int start_wx, start_wy, goal_wx, goal_wy;
    costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_wx, start_wy);
    costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_wx, goal_wy);

    pq.push({0.0, {start_wx, start_wy}});
    distances[start_wx * costmap->getSizeInCellsY() + start_wy] = 0.0;

    while (!pq.empty())
    {
        auto [current_cost, current_node] = pq.top();

        unsigned int x = current_node.first;
        unsigned int y = current_node.second;

        if(x == goal_wx && y == goal_wy){
            std::vector<geometry_msgs::msg::PoseStamped> path;
            while (previous.find(x * costmap->getSizeInCellsY() + y) != previous.end()) {
                geometry_msgs::msg::PoseStamped pose;
                costmap->mapToWorld(x, y, pose.pose.position.x, pose.pose.position.y);
                pose.pose.orientation.w = 1.0;
                path.push_back(pose);

                auto [prev_x, prev_y] = previous[x * costmap->getSizeInCellsY() + y];
                x = prev_x;
                y = prev_y;

            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for(int dx = -1; dx <= 1; dx++){
            for(int dy = -1; dy <= 1; dy++){
                if(dx == 0 && dy == 0) continue;

                unsigned int new_x = x + dx;
                unsigned int new_y = y + dy;

                if(costmap->getCost(new_x, new_y) != nav2_costmap_2d::LETHAL_OBSTACLE){
                    double new_cost = current_cost + 1.0;
                    if(distances.find(new_x * costmap->getSizeInCellsY() + new_y) == distances.end() || new_cost < distances[new_x * costmap->getSizeInCellsY() + new_y]){
                        distances[new_x * costmap->getSizeInCellsY() + new_y] = new_cost;
                        previous[new_x * costmap->getSizeInCellsY() + new_y] = {x, y};
                        pq.push({new_cost, {new_x, new_y}});
                    }
                }
            }
        }
    }

    return {};
}

void ExplorationNode::explore() {
    auto costmap = costmap_ros_->getCostmap();

    // 시작 위치와 목표 위치 설정
    geometry_msgs::msg::PoseStamped start;
    start.pose.position.x = initial_pose_.pose.pose.position.x; // 초기 x 위치
    start.pose.position.y = initial_pose_.pose.pose.position.y; // 초기 y 위치

    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 5.0; // 목표 x 위치
    goal.pose.position.y = 5.0; // 목표 y 위치

    // 경로 계산
    auto path = dijkstra(costmap, start, goal);

    // 계산된 경로 출력
    for (const auto& pose : path) {
        RCLCPP_INFO(this->get_logger(), "Path: (%f, %f)", pose.pose.position.x, pose.pose.position.y);
    }
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
