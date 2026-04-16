#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class DockControl : public rclcpp::Node
{
public:
    DockControl() : Node("dock_control")
    {
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&DockControl::joy_cb, this, std::placeholders::_1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&DockControl::pose_cb, this, std::placeholders::_1));

        dock_stat_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/dock_stat", 10,
            std::bind(&DockControl::dock_stat_callback, this, std::placeholders::_1));

        undock_stat_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/undock_stat", 10,
            std::bind(&DockControl::undock_stat_callback, this, std::placeholders::_1));


        dock_cmd_pub_ = create_publisher<std_msgs::msg::Int8>("/dock_cmd", 10);

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        timer_ = create_wall_timer(100ms,
            std::bind(&DockControl::update, this));

        RCLCPP_INFO(get_logger(), "Dock Control Node Started");
    }

private:
    /* ===== 상태 ===== */
    enum State
    {
        IDLE,
        CANCEL_NAV,
        GO_HOME,
        WAIT_ARRIVAL,
        DOCKING,
        UNDOCKING
    };

    State state_ = IDLE;

    /* ===== ROS ===== */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr dock_stat_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr undock_stat_sub_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr dock_cmd_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ===== 변수 ===== */
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::PoseStamped dock_pose_;

    bool dock_request_ = false;
    bool undock_request_ = false;

    bool dock_pose_saved_ = false;

    uint8_t docking_status = 0;
    uint8_t undocking_status = 0;

    uint8_t docking_status_last_ = 0;
    uint8_t undocking_status_last_ = 0;
    /* ===== JOY ===== */
    void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // BACK 버튼 → DOCK
        if(msg->buttons[6] == 1)
        {
            RCLCPP_WARN(get_logger(), "JOY DOCK");
            dock_request_ = true;
        }

        // START 버튼 → UNDOCK
        if(msg->buttons[7] == 1)
        {
            RCLCPP_WARN(get_logger(), "JOY UNDOCK");
            undock_request_ = true;
        }
    }

    /* ===== 위치 ===== */
    void pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }

    /* ===== 도킹 상태 ===== */
    void dock_stat_callback(const std_msgs::msg::Int8::SharedPtr msg)    {
        docking_status = msg->data;
        docking_status_last_ = docking_status;
       // RCLCPP_INFO(get_logger(), "Dock Status: %d", docking_status);
    }
    
    void undock_stat_callback(const std_msgs::msg::Int8::SharedPtr msg)    {
        undocking_status = msg->data;
        if(undocking_status != undocking_status_last_)
        {
            if(undocking_status == 1)
                RCLCPP_WARN(get_logger(), "Undocking Started");
            else if(undocking_status == 2){
               
                if(is_pose_valid(current_pose_))
                {
                    save_dock_pose(); // ⭐ 현재 위치 저장

                    RCLCPP_WARN(get_logger(), "Undocking Completed");
                    RCLCPP_INFO(get_logger(), "Dock Pose Saved: x=%.2f y=%.2f",
                        dock_pose_.pose.position.x,
                        dock_pose_.pose.position.y);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Invalid pose → skip saving");
                }
            }
            else if(undocking_status == 3)
                RCLCPP_WARN(get_logger(), "Undocking Failed");  

            // RCLCPP_INFO(get_logger(), "Undock Status Changed: %d -> %d", undocking_status_last_, undocking_status);
        }
        undocking_status_last_ = undocking_status;
        //RCLCPP_INFO(get_logger(), "Undock Status: %d", undocking_status);
    }   



    /* ===== 상태머신 ===== */
    void update()
    {
        switch(state_)
        {
        case IDLE:
            if(dock_request_)
                state_ = CANCEL_NAV;
            else if(undock_request_)
                state_ = UNDOCKING;
            break;

        case CANCEL_NAV:
            cancel_nav();
            //save_dock_pose(); // ⭐ 현재 위치 저장
            state_ = GO_HOME;
            break;

        case GO_HOME:
            send_goal();
            state_ = WAIT_ARRIVAL;
            break;

        case WAIT_ARRIVAL:
            if(arrived())
                state_ = DOCKING;
            break;

        case DOCKING:
            send_dock_cmd();
            dock_request_ = false;
            state_ = IDLE;
            break;

        case UNDOCKING:
            send_undock_cmd();
            undock_request_ = false;
            state_ = IDLE;
            break;
        }
    }

    /* ===== Nav2 ===== */
    void send_goal()
    {
        if (!nav_client_->wait_for_action_server(1s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 not available");
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose = dock_pose_;

        nav_client_->async_send_goal(goal);

        RCLCPP_INFO(get_logger(), "Going to Dock Pose");
    }

    void cancel_nav()
    {
        nav_client_->async_cancel_all_goals();
        RCLCPP_WARN(get_logger(), "Nav2 canceled");
    }

    /* ===== 도착 판단 ===== */
    bool arrived()
    {
        double dx = current_pose_.position.x - dock_pose_.pose.position.x;
        double dy = current_pose_.position.y - dock_pose_.pose.position.y;

        double dist = sqrt(dx*dx + dy*dy);

        return dist < 0.2; // 20cm
    }

    /* ===== 위치 저장 ===== */
    void save_dock_pose()
    {
        if(!dock_pose_saved_)
        {
            dock_pose_.header.frame_id = "map";
            dock_pose_.pose = current_pose_;
            dock_pose_saved_ = true;

            RCLCPP_INFO(get_logger(), "Dock pose saved");
        }
    }

    /* ===== UART 명령 ===== */
    void send_dock_cmd()
    {
        std_msgs::msg::Int8 msg;
        msg.data = 2; // DOCK
        dock_cmd_pub_->publish(msg);

        RCLCPP_WARN(get_logger(), "DOCK CMD");
    }

    void send_undock_cmd()
    {
        std_msgs::msg::Int8 msg;
        msg.data = 3; // UNDOCK
        dock_cmd_pub_->publish(msg);

        RCLCPP_WARN(get_logger(), "UNDOCK CMD");
    }

    bool is_pose_valid(const geometry_msgs::msg::Pose& pose)
    {
        // NaN 체크
        if(std::isnan(pose.position.x) || std::isnan(pose.position.y))
            return false;

        // 너무 큰 값 방지 (맵 튐)
        if(fabs(pose.position.x) > 50.0 || fabs(pose.position.y) > 50.0)
            return false;

        // 초기값 (0,0) 방지
        if(pose.position.x == 0.0 && pose.position.y == 0.0)
            return false;

        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DockControl>());
    rclcpp::shutdown();
    return 0;
}