// #include "mpc_rbt_simulator/RobotConfig.hpp"
// #include "MotionControl.hpp"

// MotionControlNode::MotionControlNode() :
//     rclcpp::Node("motion_control_node") {

//         // Subscribers for odometry and laser scans
//         // add code here
        
//         // Publisher for robot control
//         // add code here

//         // Client for path planning
//         plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

//         // Počkej na dostupnost služby
//         while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
//             RCLCPP_INFO(get_logger(), "Waiting for planning service...");
//         }
//         RCLCPP_INFO(get_logger(), "Planning service available.");

//         // Action server
//         nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
//         this, "/go_to_goal",
//         std::bind(&MotionControlNode::navHandleGoal, this,
//                 std::placeholders::_1, std::placeholders::_2),
//         std::bind(&MotionControlNode::navHandleCancel, this,
//                 std::placeholders::_1),
//         std::bind(&MotionControlNode::navHandleAccepted, this,
//                 std::placeholders::_1));

//         // // Testovací request
//         // auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
//         // request->start.pose.position.x = 0.0;
//         // request->start.pose.position.y = 2.5;  // počáteční pozice robota
//         // request->start.header.frame_id = "map";
//         // request->goal.pose.position.x = 1.0;
//         // request->goal.pose.position.y = 1.0;
//         // request->goal.header.frame_id = "map";

//         // auto future = plan_client_->async_send_request(request,
//         // std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));

//         // Subscriber pro odometrii
//         odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10,
//         std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

//         // publisher pro nastavení rychlosti
//         twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//         RCLCPP_INFO(get_logger(), "Motion control node started succesfuly.");

//         // Connect to path planning service server
//         // add code here
//     }

// void MotionControlNode::checkCollision() {
//     // add code here

//     // ********
//     // * Help *
//     // ********
//     /*
//     if (laser_scan_.ranges[i] < thresh) {
//         geometry_msgs::msg::Twist stop;
//         twist_publisher_->publish(stop);
//     }
//     */
// }

// void MotionControlNode::updateTwist() {
//     if (path_.poses.empty()) return;

//     // 1. Yaw z quaternionu
//     tf2::Quaternion q;
//     tf2::fromMsg(current_pose_.pose.orientation, q);
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

//     double rx = current_pose_.pose.position.x;
//     double ry = current_pose_.pose.position.y;

//     // 2. Najdi nejbližší bod na trase
//     int closest_idx = 0;
//     double closest_dist = std::numeric_limits<double>::max();

//     for (size_t i = 0; i < path_.poses.size(); i++) {
//         double d = std::hypot(path_.poses[i].pose.position.x - rx,
//                               path_.poses[i].pose.position.y - ry);
//         if (d < closest_dist) {
//             closest_dist = d;
//             closest_idx = i;
//         }
//     }

//     // 3. Najdi lookahead point od nejbližšího bodu dopředu
//     double lookahead = 0.4;
//     int target_idx = -1;

//     for (size_t i = closest_idx; i < path_.poses.size(); i++) {
//         double d = std::hypot(path_.poses[i].pose.position.x - rx,
//                               path_.poses[i].pose.position.y - ry);
//         if (d >= lookahead) {
//             target_idx = i;
//             break;
//         }
//     }

//     if (target_idx < 0) {
//         target_idx = path_.poses.size() - 1;
//     }

//     double gx = path_.poses[target_idx].pose.position.x;
//     double gy = path_.poses[target_idx].pose.position.y;

//     // 4. Transformace do souřadnic robota
//     double dx = gx - rx;
//     double dy = gy - ry;
//     double x_local =  cos(yaw) * dx + sin(yaw) * dy;
//     double y_local = -sin(yaw) * dx + cos(yaw) * dy;
//     //double y_local =  sin(yaw) * dx - cos(yaw) * dy;

//     // 5. Křivost
//     double L = std::hypot(x_local, y_local);
//     double curvature = (L > 0.01) ? 2.0 * y_local / (L * L) : 0.0;

//     // 6. Rychlosti
//     double v_max = 0.3;
//     double v = v_max;
//     double omega = v * curvature;

//     // 7. Saturace
//     double d_wheels = 0.271;
//     double max_wheel_speed = 10.1523 * 0.0985;

//     double v_left  = v - omega * d_wheels / 2.0;
//     double v_right = v + omega * d_wheels / 2.0;
//     double max_val = std::max(std::abs(v_left), std::abs(v_right));

//     if (max_val > max_wheel_speed) {
//         double scale = max_wheel_speed / max_val;
//         v *= scale;
//         omega *= scale;
//     }

//     // Publikuj
//     geometry_msgs::msg::Twist twist;
//     twist.linear.x = v;
//     twist.angular.z = omega;
//     twist_publisher_->publish(twist);

//     RCLCPP_INFO(get_logger(), "pos=[%.2f,%.2f] yaw=%.2f target=[%.2f,%.2f] y_local=%.2f v=%.2f omega=%.2f",
//         rx, ry, yaw, gx, gy, y_local, v, omega);
// }

// // void MotionControlNode::updateTwist() {
// //     if (path_.poses.empty()) return;

// //     // Yaw z quaternionu
// //     tf2::Quaternion q;
// //     tf2::fromMsg(current_pose_.pose.orientation, q);
// //     double roll, pitch, yaw;
// //     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

// //     double rx = current_pose_.pose.position.x;
// //     double ry = current_pose_.pose.position.y;

// //     // Najdi nejbližší bod
// //     int closest_idx = 0;
// //     double closest_dist = std::numeric_limits<double>::max();
// //     for (size_t i = 0; i < path_.poses.size(); i++) {
// //         double d = std::hypot(path_.poses[i].pose.position.x - rx,
// //                               path_.poses[i].pose.position.y - ry);
// //         if (d < closest_dist) {
// //             closest_dist = d;
// //             closest_idx = i;
// //         }
// //     }

// //     // Cílový bod — pár waypointů dopředu
// //     int target_idx = std::min(closest_idx + 5, (int)path_.poses.size() - 1);

// //     double gx = path_.poses[target_idx].pose.position.x;
// //     double gy = path_.poses[target_idx].pose.position.y;

// //     // Úhel k cíli
// //     double angle_to_goal = atan2(gy - ry, gx - rx);

// //     // Úhlová chyba — rozdíl mezi kam míříme a kam chceme
// //     double angle_error = angle_to_goal - yaw;

// //     // Normalizace do [-π, π]
// //     while (angle_error > M_PI)  angle_error -= 2.0 * M_PI;
// //     while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

// //     // P regulátor
// //     double Kp = 2.0;
// //     double omega = Kp * angle_error;

// //     // Dopředná rychlost — snižuj v zatáčkách
// //     double v_max = 0.3;
// //     double v = v_max * (1.0 - std::min(std::abs(angle_error) / M_PI, 0.9));

// //     // Saturace
// //     double d_wheels = 0.271;
// //     double max_wheel_speed = 10.1523 * 0.0985;

// //     double v_left  = v - omega * d_wheels / 2.0;
// //     double v_right = v + omega * d_wheels / 2.0;
// //     double max_val = std::max(std::abs(v_left), std::abs(v_right));

// //     if (max_val > max_wheel_speed) {
// //         double scale = max_wheel_speed / max_val;
// //         v *= scale;
// //         omega *= scale;
// //     }

// //     geometry_msgs::msg::Twist twist;
// //     twist.linear.x = v;
// //     twist.angular.z = omega;
// //     twist_publisher_->publish(twist);

// //     RCLCPP_INFO(get_logger(), "pos=[%.2f,%.2f] yaw=%.2f target=[%.2f,%.2f] angle_err=%.2f v=%.2f omega=%.2f",
// //         rx, ry, yaw, gx, gy, angle_error, v, omega);
// // }

// rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
//     (void)uuid;
//     RCLCPP_INFO(get_logger(), "Goal received: [%.2f, %.2f]",
//         goal->pose.pose.position.x, goal->pose.pose.position.y);
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
//     (void)goal_handle;
//     RCLCPP_INFO(get_logger(), "Goal cancel requested.");
//     return rclcpp_action::CancelResponse::ACCEPT;
// }

// void MotionControlNode::navHandleAccepted(
//     const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
// {
    
//     (void)goal_handle;
//     goal_handle_ = goal_handle;
//     goal_pose_ = goal_handle->get_goal()->pose;

//     auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
//     request->start = current_pose_;  // aktuální pozice z odometrie
//     request->goal = goal_pose_;

//     auto future = plan_client_->async_send_request(request,
//         std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));

//     RCLCPP_INFO(get_logger(), "Planning from [%.2f, %.2f] to [%.2f, %.2f]",
//     current_pose_.pose.position.x, current_pose_.pose.position.y,
//     goal_pose_.pose.position.x, goal_pose_.pose.position.y);

//     RCLCPP_INFO(get_logger(), "Plan requested.");
// }

// void MotionControlNode::execute() 
// {
//     RCLCPP_INFO(get_logger(), "Executing navigation...");
//     rclcpp::Rate loop_rate(10.0);
//     auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

//     while (rclcpp::ok()) {
//         // Kontrola zrušení
//         if (goal_handle_->is_canceling()) {
//             auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
//             goal_handle_->canceled(result);
//             RCLCPP_INFO(get_logger(), "Navigation canceled.");
//             return;
//         }

//         // Kontrola dosažení cíle
//         double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
//         double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
//         double dist = std::hypot(dx, dy);

//         if (dist < 0.15) {
//             auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
//             goal_handle_->succeed(result);
//             RCLCPP_INFO(get_logger(), "Goal reached!");
//             return;
//         }

//         // Feedback
//         feedback->current_pose = current_pose_; // current_pose přichází z odometrie
//         goal_handle_->publish_feedback(feedback);
//         loop_rate.sleep();
//     }
// }

// void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) 
// {
//     auto response = future.get();
//     if (response && response->plan.poses.size() > 0) {
//         path_ = response->plan;
//         RCLCPP_INFO(get_logger(), "Path received: %zu waypoints.", path_.poses.size());
//         std::thread(&MotionControlNode::execute, this).detach();
//     } else {
//         RCLCPP_ERROR(get_logger(), "No path found, aborting.");
//         auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
//         goal_handle_->abort(result);
//     }
// }

// void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
//     current_pose_.header = msg.header;
//     current_pose_.pose = msg.pose.pose;
//     updateTwist();
// }

// void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
//     // add code here
// }


#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscriber pro odometrii
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

        // Subscriber pro lidar
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/tiago_base/Hokuyo_URG_04LX_UG01", rclcpp::SensorDataQoS(),
            std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher pro řízení
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client pro plánování
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Počkej na dostupnost služby
        while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(get_logger(), "Waiting for planning service...");
        }
        RCLCPP_INFO(get_logger(), "Planning service available.");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this, "/go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this,
                    std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this,
                    std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this,
                    std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");
    }

void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty()) return;
    if (!goal_handle_ || !goal_handle_->is_active()) return;

    double thresh = 0.3;

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        float r = laser_scan_.ranges[i];
        float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;

        // Kontrola předního kužele ±45°
        if (angle > -0.78 && angle < 0.78) {
            if (std::isinf(r) || std::isnan(r)) continue;

            if (r <= thresh) {
                RCLCPP_WARN(get_logger(), "Collision detected! Distance: %.2f m, angle: %.2f rad", r, angle);
                geometry_msgs::msg::Twist stop;
                twist_publisher_->publish(stop);
                path_.poses.clear();

                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
                return;
            }
        }
    }
}

void MotionControlNode::updateTwist() {
    // Guard — nic neděláme pokud nemáme aktivní akci nebo cestu
    if (!goal_handle_ || !goal_handle_->is_active() || path_.poses.empty()) return;

    // 1. Yaw z quaternionu
    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double rx = current_pose_.pose.position.x;
    double ry = current_pose_.pose.position.y;

    // 2. Najdi nejbližší bod na trase
    int closest_idx = 0;
    double closest_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_.poses.size(); i++) {
        double d = std::hypot(path_.poses[i].pose.position.x - rx,
                              path_.poses[i].pose.position.y - ry);
        if (d < closest_dist) {
            closest_dist = d;
            closest_idx = i;
        }
    }

    // Smaž projeté wayopinty
    if (closest_idx > 0) {
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + closest_idx);
    }

    // 3. Najdi lookahead point
    double lookahead = 0.4;
    int target_idx = -1;

    for (size_t i = 0; i < path_.poses.size(); i++) {
        double d = std::hypot(path_.poses[i].pose.position.x - rx,
                              path_.poses[i].pose.position.y - ry);
        if (d >= lookahead) {
            target_idx = i;
            break;
        }
    }

    if (target_idx < 0) {
        target_idx = path_.poses.size() - 1;
    }

    double gx = path_.poses[target_idx].pose.position.x;
    double gy = path_.poses[target_idx].pose.position.y;

    // 4. Transformace do souřadnic robota
    double dx = gx - rx;
    double dy = gy - ry;
    double x_local =  cos(yaw) * dx + sin(yaw) * dy;
    double y_local = -sin(yaw) * dx + cos(yaw) * dy;

    // 5. Úhel k cíli v lokálních souřadnicích
    double alpha = std::atan2(y_local, x_local);

    // 6. Rychlosti
    double v_max = 0.7;
    double max_angular = 0.8;
    double v, omega;

    if (std::abs(alpha) > 0.5) {
        // Cíl je moc bokem — otoč se na místě
        v = 0.0;
        omega = (alpha > 0) ? max_angular : -max_angular;
    } else {
        // Pure pursuit
        double L = std::hypot(x_local, y_local);
        double curvature = (L > 0.01) ? 2.0 * y_local / (L * L) : 0.0;

        // Zpomalení v zatáčkách
        v = v_max * (1.0 - std::min(std::abs(y_local) / lookahead, 0.9));
        if (v < 0.05) v = 0.05;

        omega = v * curvature;

        // Saturace omega
        if (omega > max_angular) omega = max_angular;
        if (omega < -max_angular) omega = -max_angular;
    }

    // 7. Saturace — diferenciální podvozek
    double d_wheels = 2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS;
    double max_wheel_speed = 10.1523 * robot_config::WHEEL_RADIUS;

    double v_left  = v - omega * d_wheels / 2.0;
    double v_right = v + omega * d_wheels / 2.0;
    double max_val = std::max(std::abs(v_left), std::abs(v_right));

    if (max_val > max_wheel_speed) {
        double scale = max_wheel_speed / max_val;
        v *= scale;
        omega *= scale;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = v;
    twist.angular.z = omega;
    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Goal received: [%.2f, %.2f]",
        goal->pose.pose.position.x, goal->pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Goal cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle->get_goal()->pose;

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;

    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Planning from [%.2f, %.2f] to [%.2f, %.2f]",
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        goal_pose_.pose.position.x, goal_pose_.pose.position.y);
}

void MotionControlNode::execute()
{
    RCLCPP_INFO(get_logger(), "Executing navigation...");
    rclcpp::Rate loop_rate(10.0);
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok()) {
        if (!goal_handle_->is_active()) return;

        if (goal_handle_->is_canceling()) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->canceled(result);
            RCLCPP_INFO(get_logger(), "Navigation canceled.");
            return;
        }

        double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
        double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist < 0.2) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal reached!");
            return;
        }

        feedback->current_pose = current_pose_;
        feedback->distance_remaining = dist;
        goal_handle_->publish_feedback(feedback);
        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
{
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        RCLCPP_INFO(get_logger(), "Path received: %zu waypoints.", path_.poses.size());
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(get_logger(), "No path found, aborting.");
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;
    checkCollision();
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}