#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_srvs/srv/empty.hpp"
#include <cmath>

class TurtleMoverNode : public rclcpp::Node {
public:
    TurtleMoverNode(double x, double y) : Node("turtle_mover"), x_target_(x), y_target_(y), is_rotating_(true) {
        turtle_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleMoverNode::poseCallback, this, std::placeholders::_1)
        );
        clear_client_ = this->create_client<std_srvs::srv::Empty>("clear");
    }

private:
    void poseCallback(const turtlesim::msg::Pose::SharedPtr pose_msg) {
        double x_turtle = pose_msg->x;
        double y_turtle = pose_msg->y;

        if (is_rotating_) {
            double angle_rad = std::atan2(y_target_ - y_turtle, x_target_ - x_turtle);
            double angle_deg = angle_rad * 180.0 / M_PI;

            double angular_speed = 1.0; // Kecepatan angular yang tetap
            double angular_vel = angular_speed * (angle_rad - pose_msg->theta);

            auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_msg->angular.z = angular_vel;
            cmd_msg->linear.x = 0; // Set linear.x ke 0 agar berhenti bergerak maju

            turtle_pub_->publish(std::move(cmd_msg));

            if (std::abs(angle_deg) < 1.0) {
                is_rotating_ = false;
                clearDrawing();
            }
        } else {
            double distance = std::sqrt(std::pow(x_target_ - x_turtle, 2) + std::pow(y_target_ - y_turtle, 2));

            double linear_speed = 1.0;  // Kecepatan linear yang tetap
            double linear_vel = linear_speed * distance;

            auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_msg->linear.x = linear_vel;
            cmd_msg->angular.z = 0; // Set angular.z ke 0 agar berhenti berputar

            turtle_pub_->publish(std::move(cmd_msg));

            if (distance < 0.1) {
                RCLCPP_INFO(this->get_logger(), "Kura-kura telah mencapai target.");
                rclcpp::shutdown();
            }
        }
    }

    void clearDrawing() {
        auto clear_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_client_->async_send_request(clear_request);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    double x_target_;
    double y_target_;
    bool is_rotating_;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Penggunaan: " << argv[0] << " <x> <y>" << std::endl;
        return 1;
    }

    double x_target = std::atof(argv[1]);
    double y_target = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMoverNode>(x_target, y_target));
    rclcpp::shutdown();
    return 0;
}
