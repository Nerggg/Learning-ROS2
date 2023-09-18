#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp" // Untuk layanan clear
#include <cstdlib> // Untuk konversi dari string ke tipe data numerik

class TurtleControllerNode : public rclcpp::Node {
public:
    TurtleControllerNode(double linear_speed, double angular_speed) : Node("turtle_controller") {
        // Set parameter values
        linear_speed_ = linear_speed;
        angular_speed_ = angular_speed;
        has_cleared_drawing_ = false; // Tandai bahwa drawing belum dihapus

        // Create a publisher to control the turtle's movement
        turtle_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Create a timer to publish velocity commands periodically
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TurtleControllerNode::publishCommand, this));

        // Create a client to call the clear service (only call it once)
        clear_client_ = this->create_client<std_srvs::srv::Empty>("clear");
    }

private:
    void publishCommand() {
        if (!has_cleared_drawing_) {
            // Call the clear service to clear the drawing (only once)
            auto clear_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = clear_client_->async_send_request(clear_request);
            has_cleared_drawing_ = true; // Tandai bahwa drawing sudah dihapus
        }

        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->linear.x = linear_speed_;
        cmd_msg->angular.z = angular_speed_;

        turtle_pub_->publish(std::move(cmd_msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    double linear_speed_;
    double angular_speed_;
    bool has_cleared_drawing_;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <linear_speed> <angular_speed>" << std::endl;
        return 1;
    }

    // Konversi argumen dari string ke tipe data numerik
    double linear_speed = std::atof(argv[1]);
    double angular_speed = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControllerNode>(linear_speed, angular_speed));
    rclcpp::shutdown();
    return 0;
}
