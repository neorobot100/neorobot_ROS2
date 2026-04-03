#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class SafetyManager : public rclcpp::Node
{
public:
  SafetyManager() : Node("safety_manager")
  {
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_cmd", 10,
      std::bind(&SafetyManager::cmdCallback, this, std::placeholders::_1));

    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/collision_detected", 10,
      std::bind(&SafetyManager::collisionCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    action_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SafetyManager::publishLoop, this));

    RCLCPP_INFO(get_logger(), "Industrial Safety Manager Ready");

    state_start_time_ = now();   // ⭐ 이것 추가
  }

private:
  enum class State { NORMAL, STOPPING, BACKING, LOCKED };

  State state_ = State::NORMAL;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time state_start_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
  }

  void collisionCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && state_ == State::NORMAL)
    {
      RCLCPP_ERROR(get_logger(), "COLLISION DETECTED → EMERGENCY BACKUP");

      cancelNav2Goal();
      state_ = State::STOPPING;
      state_start_time_ = now();
    }
  }

  void publishLoop()
  {
    geometry_msgs::msg::Twist out;

    double elapsed = 0.0;
    if (state_ != State::NORMAL)
            elapsed = (now() - state_start_time_).seconds();

    //auto elapsed = (now() - state_start_time_).seconds();

    switch (state_)
    {
      case State::NORMAL:
        out = last_cmd_;
        break;

      case State::STOPPING:
        out.linear.x = 0.0;
        out.angular.z = 0.0;

        if (elapsed > 0.3)
        {
          state_ = State::BACKING;
          state_start_time_ = now();
        }
        break;

      case State::BACKING:
        out.linear.x = -0.15;  // 후진 속도
        out.angular.z = 0.0;

        if (elapsed > 0.5)
        {
          state_ = State::LOCKED;
          state_start_time_ = now();
        }
        break;

      case State::LOCKED:
        out.linear.x = 0.0;
        out.angular.z = 0.0;
        break;
    }

    cmd_pub_->publish(out);
  }

  void cancelNav2Goal()
  {
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500)))
      return;

    action_client_->async_cancel_all_goals();
    RCLCPP_WARN(get_logger(), "Nav2 Goal Cancelled");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyManager>());
  rclcpp::shutdown();
  return 0;
}