#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/action/MoveTurtle.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class TurtleActionServer : public rclcpp::Node
{
public:
    TurtleActionServer()
        : Node("turtle_action_server"), action_server_(rclcpp_action::create_server<turtle_control::action::MoveTurtle>(
              this,
              "move_turtle",
              std::bind(&TurtleActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
              std::bind(&TurtleActionServer::handle_cancel, this, std::placeholders::_1),
              std::bind(&TurtleActionServer::handle_accepted, this, std::placeholders::_1)))
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const turtle_control::action::MoveTurtle::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtle_control::action::MoveTurtle>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtle_control::action::MoveTurtle>> goal_handle);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp_action::Server<turtle_control::action::MoveTurtle>::SharedPtr action_server_;
};

rclcpp_action::GoalResponse TurtleActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const turtle_control::action::MoveTurtle::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtleActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtle_control::action::MoveTurtle>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtle_control::action::MoveTurtle>> goal_handle)
{
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    auto feedback = std::make_shared<turtle_control::action::MoveTurtle::Feedback>();
    auto &cmd_vel = feedback->cmd_vel;

    const auto goal = goal_handle->get_goal();
    const auto target_x = goal->x;
    const auto target_y = goal->y;

    // Define the maximum linear and angular velocities
    const double max_linear_velocity = 0.5;    // You can adjust this value
    const double max_angular_velocity = 1.0;   // You can adjust this value

    // Calculate linear and angular errors
    double linear_error = target_x - current_x; // Assuming you have current_x and current_y
    double angular_error = atan2(target_y - current_y, target_x - current_x) - current_yaw; // Assuming you have current_yaw

    // Ensure the linear and angular velocities are within limits
    double linear_velocity = std::min(max_linear_velocity, std::max(-max_linear_velocity, kp_linear * linear_error));
    double angular_velocity = std::min(max_angular_velocity, std::max(-max_angular_velocity, kp_angular * angular_error));

    // Create and publish the cmd_vel message
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_velocity;
    cmd_vel_msg.angular.z = angular_velocity;
    pub_->publish(cmd_vel_msg);

    // When the goal is achieved, set the status as succeeded
    goal_handle->succeed(std::make_shared<turtle_control::action::MoveTurtle::Result>());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleActionServer>());
    rclcpp::shutdown();
    return 0;
}
