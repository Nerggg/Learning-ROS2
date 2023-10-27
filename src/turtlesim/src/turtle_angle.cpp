#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cstdlib> // Untuk konversi dari string ke tipe data numerik
#include <cmath>
#include <iostream> // Untuk output ke terminal

class TurtleTrackerNode : public rclcpp::Node {
public:
    TurtleTrackerNode() : Node("turtle_tracker") {
      // Ini udah ga dipake lagi, soalnya pakai request dari service
      //        // Set parameter values
//        x_target_ = x;
//        y_target_ = y;

        // TODO: ganti turtlesim::srv::Theta dengan service yang sesuai karena asumsiku ini nama packageny turtlesim (bisa dilihat di folder srv terdapat Theta.srv)
        service_ = this->create_service<turtlesim::srv::Theta>(
            "turtle_angle", std::bind(&TurtleTrackerNode::thetaCallback, this, std::placeholders::_1, std::placeholders::_2)
            )

        // Subscribe to the turtle's pose
        turtle_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleTrackerNode::poseCallback, this, std::placeholders::_1)
        );
    }

private:
  void thetaCallback(const std::shared_ptr<turtlesim::srv::Theta::Request> request, std::shared_ptr<turtlesim::srv::Theta::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Request: x=%f, y=%f", request->x, request->y);

      float goal_x = request->x;
      float goal_y = request->y;

      // pendekatan ini gembel tapi ngehandle kuadran tiga
      if (goal_x - this->x_turtle == 0) {
        if (goal_y - this->y_turtle > 0) {
          response->theta = 1.5708;
        } else {
          response->theta = -1.5708;
        }
      } else {
        response->theta = atan((goal_y - this->y_turtle) / (goal_x - this->x_turtle));
      }

      if (goal_x - this->x_turtle < 0 && goal_y - this->y_turtle < 0) {
        response->theta += 3.14159;
      }

      if (response->theta < 0) {
        response->theta += 6.28319;
      }
      if (response->theta > 6.28319) {
        response->theta -= 6.28319;
      }

      RCLCPP_INFO(this->get_logger(), "Sending back response: [%f]", response->theta);
  }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose_msg) {
      x_turtle = pose_msg->x;
      y_turtle = pose_msg->y;

        // Pendekatan ini bisa salah karena tidak memperhitungkan kuadran ketiga (x < 0, y < 0)
//        // Calculate the angle between turtle and (x, y) using atan2
//        double angle_rad = std::atan2(y_target_ - y_turtle, x_target_ - x_turtle);
//        double angle_deg = angle_rad * 180.0 / M_PI;

      // Ini kuhilangkan biar ga kebanyakan outputnya
//        std::cout << "Turtle is at (" << x_turtle << ", " << y_turtle << ")" << std::endl;
//        std::cout << "Angle to target (" << x_target_ << ", " << y_target_ << "): " << angle_deg << " degrees" << std::endl;

    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr turtle_pose_sub_;
    rclcpp::Service<turtlesim::srv::Theta>::SharedPtr service_; // TODO: ganti juga ini turtlesim::srv::Theta dengan yang sesuai

    // x_target dan y_target nanti dimasukkan di request dari service
//    double x_target_;
//    double y_target_;

    // ku gini biar nanti bisa diakses di thetaCallback dengan lebih enak aja
    double x_turtle;
    double y_turtle;
};

int main(int argc, char** argv) {

  // terima request dari service
//    if (argc < 3) {
//        RCLCPP_ERROR(rclcpp::get_logger("turtle_tracker"), "Usage: %s <x> <y>", argv[0]);
//        return 1;
//    }

//    double x_target = std::atof(argv[1]);
//    double y_target = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
