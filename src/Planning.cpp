#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
        "/plan_path",
        std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // Publisher pro dilatovanou mapu — pro vizualizaci v RVizu
    dilated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/dilated_map", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(), "Planning node started.");

    while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "Waiting for map server...");
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = map_client_->async_send_request(request,
        std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received: %d x %d, resolution: %.3f",
            map_.info.width, map_.info.height, map_.info.resolution);
        dilateMap();
         map_timer_ = this->create_wall_timer(std::chrono::seconds(2),
            [this]() { dilated_map_pub_->publish(map_); });
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                             std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    path_.poses.clear();
    path_.header.frame_id = "map";
    path_.header.stamp = this->now();

    aStar(request->start, request->goal);
    smoothPath();

    response->plan = path_;
    path_pub_->publish(path_);
}

void PlanningNode::dilateMap() {
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    int w = map_.info.width;
    int h = map_.info.height;
    int radius = 10;

    int count_before = 0;
    for (int i = 0; i < w * h; i++) {
        if (map_.data[i] == 100) count_before++;
    }

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (map_.data[y * w + x] == 100) {
                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                        if (dx * dx + dy * dy <= radius * radius) {
                            dilatedMap.data[ny * w + nx] = 100;
                        }
                    }
                }
            }
        }
    }

    int count_after = 0;
    for (int i = 0; i < w * h; i++) {
        if (dilatedMap.data[i] == 100) count_after++;
    }
    RCLCPP_INFO(get_logger(), "Dilation: before=%d occupied, after=%d occupied, radius=%d",
        count_before, count_after, radius);

    map_ = dilatedMap;

    // Publikuj dilatovanou mapu pro vizualizaci
    dilated_map_pub_->publish(map_);

    RCLCPP_INFO(get_logger(), "Map dilated with radius %d cells.", radius);
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start,
                          const geometry_msgs::msg::PoseStamped &goal) {
    int sx = (start.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int sy = (start.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;
    int gx = (goal.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int gy = (goal.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

    int w = map_.info.width;
    int h = map_.info.height;

    if (sx < 0 || sx >= w || sy < 0 || sy >= h ||
        gx < 0 || gx >= w || gy < 0 || gy >= h) {
        RCLCPP_ERROR(get_logger(), "Start or goal out of map bounds.");
        return;
    }

    // Kopie mapy
    auto planning_map = map_.data;

    // Vyčisti okolí startu a cíle z dilatace
    for (int dy2 = -3; dy2 <= 3; dy2++) {
        for (int dx2 = -3; dx2 <= 3; dx2++) {
            int nsx = sx + dx2, nsy = sy + dy2;
            int ngx = gx + dx2, ngy = gy + dy2;
            if (nsx >= 0 && nsx < w && nsy >= 0 && nsy < h)
                planning_map[nsy * w + nsx] = 0;
            if (ngx >= 0 && ngx < w && ngy >= 0 && ngy < h)
                planning_map[ngy * w + ngx] = 0;
        }
    }

    RCLCPP_INFO(get_logger(), "A* from grid [%d,%d] to [%d,%d], start_val=%d, goal_val=%d",
        sx, sy, gx, gy, planning_map[sy * w + sx], planning_map[gy * w + gx]);

    auto cStart = std::make_shared<Cell>(sx, sy);
    cStart->g = 0.0f;
    cStart->h = std::hypot(gx - sx, gy - sy);
    cStart->f = cStart->g + cStart->h;

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(h * w, false);

    openList.push_back(cStart);

    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    float dc[] = {1, 1, 1, 1, 1.414f, 1.414f, 1.414f, 1.414f};

    while (!openList.empty() && rclcpp::ok()) {
        auto it = std::min_element(openList.begin(), openList.end(),
            [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b) {
                return a->f < b->f;
            });
        auto current = *it;
        openList.erase(it);

        if (current->x == gx && current->y == gy) {
            std::vector<geometry_msgs::msg::PoseStamped> poses;
            auto cell = current;
            while (cell != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = cell->x * map_.info.resolution + map_.info.origin.position.x;
                pose.pose.position.y = cell->y * map_.info.resolution + map_.info.origin.position.y;
                pose.pose.orientation.w = 1.0;
                poses.push_back(pose);
                cell = cell->parent;
            }
            std::reverse(poses.begin(), poses.end());
            path_.poses = poses;
            RCLCPP_INFO(get_logger(), "Path found with %ld waypoints.", poses.size());
            return;
        }

        closedList[current->y * w + current->x] = true;

        for (int i = 0; i < 8; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (closedList[ny * w + nx]) continue;
            if (planning_map[ny * w + nx] != 0) continue;

            float newG = current->g + dc[i];

            auto existing = std::find_if(openList.begin(), openList.end(),
                [nx, ny](const std::shared_ptr<Cell> &c) {
                    return c->x == nx && c->y == ny;
                });

            if (existing != openList.end()) {
                if (newG < (*existing)->g) {
                    (*existing)->g = newG;
                    (*existing)->f = newG + (*existing)->h;
                    (*existing)->parent = current;
                }
            } else {
                auto neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = newG;
                neighbor->h = std::hypot(gx - nx, gy - ny);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                openList.push_back(neighbor);
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) return;

    float alpha = 0.5f;
    float beta = 0.3f;
    float tolerance = 0.0001f;
    int maxIter = 500;

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;

    for (int iter = 0; iter < maxIter; iter++) {
        float change = 0.0f;

        for (size_t i = 1; i < newPath.size() - 1; i++) {
            float origX = path_.poses[i].pose.position.x;
            float origY = path_.poses[i].pose.position.y;
            float currX = newPath[i].pose.position.x;
            float currY = newPath[i].pose.position.y;
            float prevX = newPath[i - 1].pose.position.x;
            float prevY = newPath[i - 1].pose.position.y;
            float nextX = newPath[i + 1].pose.position.x;
            float nextY = newPath[i + 1].pose.position.y;

            float newX = currX + alpha * (origX - currX) + beta * (prevX + nextX - 2 * currX);
            float newY = currY + alpha * (origY - currY) + beta * (prevY + nextY - 2 * currY);

            change += std::abs(newX - currX) + std::abs(newY - currY);

            newPath[i].pose.position.x = newX;
            newPath[i].pose.position.y = newY;
        }

        if (change < tolerance) break;
    }

    path_.poses = newPath;
    RCLCPP_INFO(get_logger(), "Path smoothed.");
}

Cell::Cell(int c, int r) {
    x = c;
    y = r;
    f = 0.0f;
    g = 0.0f;
    h = 0.0f;
    parent = nullptr;
}
