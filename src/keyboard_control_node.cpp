
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include "KeyboardControl.hpp"

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") {

//...

twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&KeyboardControlNode::timerCallback, this));

this->declare_parameter("speed", 1.0);



// Set terminal settings to non-blocking
tcgetattr(STDIN_FILENO, &old_termios_);
struct termios new_termios = old_termios_;
new_termios.c_lflag &= ~(ICANON | ECHO);
tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");

//...

}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    geometry_msgs::msg::Twist twist{};
    char c;
    double _speed = this->get_parameter("speed").as_double();
    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sequence (arrow keys)
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2)
                    return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A':
                            twist.linear.x = _speed;  // up arrow
                            break;
                        case 'B':
                            twist.linear.x = - _speed; // down arrow
                            break;
                        case 'C':
                            twist.angular.z = _speed; // right arrow
                            break;
                        case 'D':
                            twist.angular.z = - _speed;  // left arrow
                            break;
                    }
                }
            }
  
        }
    }

    twist_publisher_->publish(twist);
    // vyprázdni zbytek bufferu
    while (read(STDIN_FILENO, &c, 1) == 1);
    // else no data was available, do nothing
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControlNode>());
    rclcpp::shutdown();
    return 0;
}
