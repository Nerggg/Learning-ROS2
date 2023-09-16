#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class TurtleMoverNode : public rclcpp::Node {
public:
    TurtleMoverNode(double x, double y) : Node("turtle_mover"), x_target_(x), y_target_(y), is_rotating_(true) {
        // Create a publisher to control the turtle's movement
        turtle_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Subscribe to the turtle's pose
        turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleMoverNode::poseCallback, this, std::placeholders::_1)
        );
    }

private:
    void poseCallback(const turtlesim::msg::Pose::SharedPtr pose_msg) {
        double x_turtle = pose_msg->x;
        double y_turtle = pose_msg->y;

        if (is_rotating_) {
            // Calculate the angle to the target point
            double angle_rad = std::atan2(y_target_ - y_turtle, x_target_ - x_turtle);
            double angle_deg = angle_rad * 180.0 / M_PI;

            // Determine the angular velocity to rotate toward the target
            double angular_speed = 1.0; // Adjust as needed
            double angular_vel = angular_speed * (angle_rad - pose_msg->theta);

            // Create and publish a Twist message to control rotation
            auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_msg->angular.z = angular_vel;

            turtle_pub_->publish(std::move(cmd_msg));

            // Check if the turtle is facing the target
            if (std::abs(angle_deg) < 1.0) {
                is_rotating_ = false;
            }
        } else {
            // Calculate the distance to the target point
            double distance = std::sqrt(std::pow(x_target_ - x_turtle, 2) + std::pow(y_target_ - y_turtle, 2));

            // Determine the linear velocity to move toward the target
            double linear_speed = 1.0;  // Adjust as needed
            double linear_vel = linear_speed * distance;

            // Create and publish a Twist message to control forward movement
            auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_msg->linear.x = linear_vel;

            turtle_pub_->publish(std::move(cmd_msg));

            // Check if the turtle has reached the target
            if (distance < 0.1) {
                RCLCPP_INFO(this->get_logger(), "Turtle has reached the target.");
                rclcpp::shutdown();
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    double x_target_;
    double y_target_;
    bool is_rotating_;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <x> <y>" << std::endl;
        return 1;
    }

    double x_target = std::atof(argv[1]);
    double y_target = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMoverNode>(x_target, y_target));
    rclcpp::shutdown();
    return 0;
}
