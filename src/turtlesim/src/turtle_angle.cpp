#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cstdlib> // Untuk konversi dari string ke tipe data numerik
#include <cmath>

class TurtleTrackerNode : public rclcpp::Node {
public:
    TurtleTrackerNode(double x, double y) : Node("turtle_tracker") {
        // Set parameter values
        x_target_ = x;
        y_target_ = y;

        // Subscribe to the turtle's pose
        turtle_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleTrackerNode::poseCallback, this, std::placeholders::_1)
        );
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose_msg) {
        double x_turtle = pose_msg->position.x;
        double y_turtle = pose_msg->position.y;

        // Calculate the angle between turtle and (x, y) using atan2
        double angle_rad = std::atan2(y_target_ - y_turtle, x_target_ - x_turtle);
        double angle_deg = angle_rad * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(), "Received turtle pose data.");
        RCLCPP_INFO(this->get_logger(), "Turtle is at (%.2f, %.2f)", x_turtle, y_turtle);
        RCLCPP_INFO(this->get_logger(), "Angle to target (%.2f, %.2f): %.2f degrees", x_target_, y_target_, angle_deg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr turtle_pose_sub_;
    double x_target_;
    double y_target_;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("turtle_tracker"), "Usage: %s <x> <y>", argv[0]);
        return 1;
    }

    double x_target = std::atof(argv[1]);
    double y_target = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleTrackerNode>(x_target, y_target));
    rclcpp::shutdown();
    return 0;
}

