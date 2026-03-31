#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

    // Client for map
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Service for path
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
        "/plan_path",
        std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));

    // Publisher for path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(get_logger(), "Planning node started.");

    // Connect to map server
    while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "Waiting for map server...");
    }

    // Request map
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
    int radius = 3; // 8 * 0.05m = 0.4m odstup

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

    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Map dilated with radius %d cells.", radius);
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start,
                          const geometry_msgs::msg::PoseStamped &goal) {
    // Převod světových souřadnic na souřadnice mřížky
    int sx = (start.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int sy = (start.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;
    int gx = (goal.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int gy = (goal.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;
    

// RCLCPP_INFO(get_logger(), "Start grid: [%d, %d] val: %d", sx, sy, map_.data[sy * w + sx]);
// RCLCPP_INFO(get_logger(), "Goal grid: [%d, %d] val: %d", gx, gy, map_.data[gy * w + gx]);

    int w = map_.info.width;
    int h = map_.info.height;

    // Kontrola mezí
    if (sx < 0 || sx >= w || sy < 0 || sy >= h ||
        gx < 0 || gx >= w || gy < 0 || gy >= h) {
        RCLCPP_ERROR(get_logger(), "Start or goal out of map bounds.");
        return;
    }

    auto cStart = std::make_shared<Cell>(sx, sy); // vytvořím si cell
    cStart->g = 0.0f;
    cStart->h = std::hypot(gx - sx, gy - sy);
    cStart->f = cStart->g + cStart->h;

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(h * w, false);

    openList.push_back(cStart);

    // 8 směrů: x, y, cena
    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    float dc[] = {1, 1, 1, 1, 1.414f, 1.414f, 1.414f, 1.414f};

    while (!openList.empty() && rclcpp::ok()) {
        // Najdi uzel s nejnižším f
        auto it = std::min_element(openList.begin(), openList.end(),
            [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b) {
                return a->f < b->f;
            });
        auto current = *it;
        openList.erase(it);

        // Cíl nalezen
        if (current->x == gx && current->y == gy) {
            // Rekonstrukce cesty
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

        // Expanze sousedů
        for (int i = 0; i < 8; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (closedList[ny * w + nx]) continue;
            if (map_.data[ny * w + nx] != 0) continue; // překážka nebo neznámé

            float newG = current->g + dc[i];

            // Zkontroluj jestli už je v open listu
            auto existing = std::find_if(openList.begin(), openList.end(),
                [nx, ny](const std::shared_ptr<Cell> &c) {
                    return c->x == nx && c->y == ny;
                });
            // pokud se najde shoda, porovná se nalezený prvek s novým
            if (existing != openList.end()) { // end se vrací pokud se nenašela žádná shoda (ukazatel za poslední prvek)
                if (newG < (*existing)->g) {
                    (*existing)->g = newG;
                    (*existing)->f = newG + (*existing)->h;
                    (*existing)->parent = current;
                }
            } else { // pokud není shoda, přidá se prvek do open listu
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
    if (path_.poses.size() < 3) return; // nemá smysl vyhlazovat

    float alpha = 0.1f;  // datová síla (drží blízko originálu)
    float beta = 0.6f;   // vyhlazovací síla (táhne ke středu sousedů)
    float tolerance = 0.0001f;
    int maxIter = 500;

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;

    for (int iter = 0; iter < maxIter; iter++) {
        float change = 0.0f;

        for (size_t i = 1; i < newPath.size() - 1; i++) { // start a cíl fixní
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
