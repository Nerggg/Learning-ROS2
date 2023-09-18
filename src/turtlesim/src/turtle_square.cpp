#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp" // Untuk layanan clear
#include <cstdlib> // Untuk konversi dari string ke tipe data numerik
#include <cmath>

class TurtleSquareNode : public rclcpp::Node {
public:
    TurtleSquareNode(double length, double width) : Node("turtle_square"), length_(length), width_(width), is_rotating_(true) {
        // Create a publisher to control the turtle's movement
        turtle_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Create a client to call the clear service
        clear_client_ = this->create_client<std_srvs::srv::Empty>("clear");
    }

private:
    void startMoving() {
        // Call the clear service to clear the drawing
        auto clear_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_client_->async_send_request(clear_request);

        is_rotating_ = false;
        moveStraight();
    }

    void moveStraight() {
        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->linear.x = 1.0; // Adjust linear speed as needed

        // Move forward for the specified length
        double distance = 0.0;
        auto start_time = this->now();
        while (distance < length_ && rclcpp::ok()) {
            cmd_msg->linear.x = 1.0; // Adjust linear speed as needed
            turtle_pub_->publish(std::move(cmd_msg));
            rclcpp::spin_some(shared_from_this());
            distance = (this->now() - start_time).seconds() * cmd_msg->linear.x;
        }

        // Stop moving
        cmd_msg->linear.x = 0.0;
        turtle_pub_->publish(std::move(cmd_msg));
        rclcpp::sleep_for(std::chrono::seconds(1)); // Pause for 1 second

        // Rotate to change direction
        rotate();
    }

    void rotate() {
        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->angular.z = M_PI / 2.0; // Rotate 90 degrees

        // Rotate for 90 degrees
        double rotated_angle = 0.0;
        auto start_time = this->now();
        while (rotated_angle < M_PI / 2.0 && rclcpp::ok()) {
            cmd_msg->angular.z = M_PI / 2.0; // Adjust angular speed as needed
            turtle_pub_->publish(std::move(cmd_msg));
            rclcpp::spin_some(shared_from_this());
            rotated_angle = (this->now() - start_time).seconds() * cmd_msg->angular.z;
        }

        // Stop rotating
        cmd_msg->angular.z = 0.0;
        turtle_pub_->publish(std::move(cmd_msg));
        rclcpp::sleep_for(std::chrono::seconds(1)); // Pause for 1 second

        // Move straight again for the specified width
        moveStraightWidth();
    }

    void moveStraightWidth() {
        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->linear.x = 1.0; // Adjust linear speed as needed

        // Move forward for the specified width
        double distance = 0.0;
        auto start_time = this->now();
        while (distance < width_ && rclcpp::ok()) {
            cmd_msg->linear.x = 1.0; // Adjust linear speed as needed
            turtle_pub_->publish(std::move(cmd_msg));
            rclcpp::spin_some(shared_from_this());
            distance = (this->now() - start_time).seconds() * cmd_msg->linear.x;
        }

        // Stop moving
        cmd_msg->linear.x = 0.0;
        turtle_pub_->publish(std::move(cmd_msg));
        rclcpp::sleep_for(std::chrono::seconds(1)); // Pause for 1 second

        // Rotate to complete the square
        rotateComplete();
    }

    void rotateComplete() {
        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->angular.z = M_PI / 2.0; // Rotate 90 degrees

        // Rotate for 90 degrees to complete the square
        double rotated_angle = 0.0;
        auto start_time = this->now();
        while (rotated_angle < M_PI / 2.0 && rclcpp::ok()) {
            cmd_msg->angular.z = M_PI / 2.0; // Adjust angular speed as needed
            turtle_pub_->publish(std::move(cmd_msg));
            rclcpp::spin_some(shared_from_this());
            rotated_angle = (this->now() - start_time).seconds() * cmd_msg->angular.z;
        }

        // Stop rotating
        cmd_msg->angular.z = 0.0;
        turtle_pub_->publish(std::move(cmd_msg));
        rclcpp::sleep_for(std::chrono::seconds(1)); // Pause for 1 second

        // Move back to the starting position
        moveBack();
    }

    void moveBack() {
        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->linear.x = -1.0; // Move backward

        // Move backward to the starting position
        double distance = 0.0;
        auto start_time = this->now();
        while (distance > -length_ && rclcpp::ok()) {
            cmd_msg->linear.x = -1.0; // Adjust linear speed as needed
            turtle_pub_->publish(std::move(cmd_msg));
            rclcpp::spin_some(shared_from_this());
            distance = (this->now() - start_time).seconds() * cmd_msg->linear.x;
        }

        // Stop moving
        cmd_msg->linear.x = 0.0;
        turtle_pub_->publish(std::move(cmd_msg));

        RCLCPP_INFO(this->get_logger(), "Turtle has completed the square.");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_pub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    double length_;
    double width_;
    bool is_rotating_;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <length> <width>" << std::endl;
        return 1;
    }

    double length = std::atof(argv[1]);
    double width = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSquareNode>(length, width));
    rclcpp::shutdown();
    return 0;
}
