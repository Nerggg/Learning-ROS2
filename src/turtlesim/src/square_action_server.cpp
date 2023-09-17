#include <rclcpp/rclcpp.hpp>
#include <turtlesim/action/square.hpp>
#include <turtlesim/srv/teleport_relative.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SquareActionServer : public rclcpp::Node
{
public:
    SquareActionServer()
        : Node("square_action_server"),
          action_server_(rclcpp_action::create_server<turtlesim::action::Square>(
              this,
              "square",
              std::bind(&SquareActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
              std::bind(&SquareActionServer::handle_cancel, this, std::placeholders::_1),
              std::bind(&SquareActionServer::handle_accepted, this, std::placeholders::_1)))
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        reset_turtle_client_ = this->create_client<turtlesim::srv::TeleportRelative>("turtle1/teleport_relative");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const turtlesim::action::Square::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtlesim::action::Square>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtlesim::action::Square>> goal_handle);

    void reset_turtle_to_center();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp_action::Server<turtlesim::action::Square>::SharedPtr action_server_;
    rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr reset_turtle_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double side_length_;
    double width_;
};

rclcpp_action::GoalResponse SquareActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const turtlesim::action::Square::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received square action goal");
    side_length_ = goal->side_length;
    width_ = goal->width;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SquareActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtlesim::action::Square>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Square action goal canceled");
    reset_turtle_to_center(); // Reset the turtle to the center
    pub_->publish(geometry_msgs::msg::Twist()); // Stop the turtle
    goal_handle->canceled();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SquareActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtlesim::action::Square>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing square action");
    auto feedback = std::make_shared<turtlesim::action::Square::Feedback>();
    auto &cmd_vel = feedback->cmd_vel;

    // Calculate the time needed to complete one side of the square
    double side_time = side_length_ / 1.0; // Adjust speed as needed
    geometry_msgs::msg::Twist cmd;

    // Set the initial direction
    cmd.linear.x = 1.0; // Move forward
    cmd.angular.z = 0.0;

    // Create a timer to switch directions
    timer_ = this->create_wall_timer(std::chrono::duration<double>(side_time), [this, cmd]() {
        if (cmd.linear.x > 0) {
            cmd.linear.x = 0.0;
            cmd.angular.z = -1.0; // Turn left
        } else if (cmd.angular.z < 0) {
            cmd.linear.x = 1.0; // Move forward
            cmd.angular.z = 0.0;
        }
        pub_->publish(cmd);
    });

    // When the timer completes, finish the action
    timer_->set_oneshot();

    // Reset the turtle to the center when the action is done
    goal_handle->set_succeeded(feedback);
    reset_turtle_to_center();
}

void SquareActionServer::reset_turtle_to_center()
{
    auto request = std::make_shared<turtlesim::srv::TeleportRelative::Request>();
    request->linear = 0.0;
    request->angular = 0.0;
    reset_turtle_client_->async_send_request(request);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareActionServer>());
    rclcpp::shutdown();
    return 0;
}
