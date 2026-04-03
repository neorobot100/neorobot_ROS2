#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <algorithm>

class LookaheadController : public rclcpp::Node
{
public:
    LookaheadController() : Node("lookahead_controller")
    {
        // ✅ path
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg)
            {
                path_ = *msg;
                 // ⭐ 새로운 goal → 상태 리셋
                if( goal_state_ == GoalState::DONE)
                    goal_state_ = GoalState::DRIVE;
            });

        // ✅ lidar
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                scan_ = msg;
            });

        // ✅ output → Arbiter로 보냄
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&LookaheadController::odomCallback, this, std::placeholders::_1)
        );

       imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&LookaheadController::imuCallback, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LookaheadController::loop, this));


       

        RCLCPP_INFO(this->get_logger(), "Lookahead Controller Started");
    }

private:
    nav_msgs::msg::Path path_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    geometry_msgs::msg::Point prev_target_;

    double prev_v_ = 0.0;
    double prev_w_ = 0.0;
    double current_speed_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
   
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    enum class GoalState
    {
        DRIVE,
        STOP_WAIT,
        ALIGN,
        DONE
    };

    GoalState goal_state_ = GoalState::DRIVE;
    rclcpp::Time state_start_time_;

    void loop()
    {
        if (path_.poses.empty()) {
          //   RCLCPP_INFO(this->get_logger(),"N0 path!!!");
            return; // 경로 없으면 아무것도 안 함
        }

        geometry_msgs::msg::TransformStamped tf;

        try
        {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero); //map 기준으로 로봇 위치 가져옴
        }
        catch (...)
        {
            return;
        }

        double robot_x = tf.transform.translation.x;
        double robot_y = tf.transform.translation.y;
        double robot_yaw = tf2::getYaw(tf.transform.rotation);

        // -----------------------------
        // 🚨 충돌 방지
        // -----------------------------
        double min_front = 10.0; //앞쪽 최소 거리 초기화

        if (scan_)
        {
            int center = scan_->ranges.size() / 2; //LiDAR 중앙 = 정면

            for (int i = center - 20; i < center + 20; i++)    //정면 ±20개만 봄 (앞쪽만 체크)
            {
                float r = scan_->ranges[i];
                if (r > 0.05 && r < min_front)   // 유효 거리 중 가장 가까운 것 선택
                    min_front = r;               // 앞쪽 장애물까지 거리 = min_front
            }
        }

        // -----------------------------
        // 1. closest point
        // -----------------------------
        size_t closest = 0;
        double min_dist = 1e9;
        
        //현재 로봇과 가장 가까운 경로 점 찾기
        for (size_t i = 0; i < path_.poses.size(); i++) //경로 전체 탐색
        {
            double dx = path_.poses[i].pose.position.x - robot_x;
            double dy = path_.poses[i].pose.position.y - robot_y;
            double dist = hypot(dx, dy);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest = i;
            }
        }

        // -----------------------------
        // 2. lookahead target
        // -----------------------------
        double base_lookahead  = 0.1; //0.5;  //🥕 당근 거리 ,Lookahead target 선택, 로봇 앞 몇 m를 목표로 볼지, 최소 거리 (정지 상태 기준)
        double speed = current_speed_;  // odom에서 받아오기
        double lookahead = base_lookahead + speed * 0.8; //속도 빠르면 더 멀리 봄

       // geometry_msgs::msg::Point target;  //x, y, z 👉 위치만 있음
        geometry_msgs::msg::PoseStamped target_bl;  // 위치 + 방향 + 좌표계 정보 포함
        bool found = false;

        for (size_t i = closest; i < path_.poses.size(); i++) // 이미 지난 경로는 무시, 뒤로 가지 않게 closest부터 시작
        {
            geometry_msgs::msg::PoseStamped pose_bl;

            // "이 경로 포인트를 base_link로 변환 시도 → 실패하면 그냥 다음 포인트로 넘어간다"
            try {
                //tf_buffer_->transform(path_.poses[i], pose_bl, "base_link"); //map → base_link 변환, 성공하면 pose_bl에 값 들어감 → 정상 계산
                geometry_msgs::msg::PoseStamped pose = path_.poses[i];
                pose.header.stamp = rclcpp::Time(0);  // ⭐ 핵심

                tf_buffer_->transform(pose, pose_bl, "base_link");

            } 
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF FAIL: %s", ex.what());
                continue;
            }
            // 거리 계산
            double dist = hypot(
                pose_bl.pose.position.x,
                pose_bl.pose.position.y
            );

            if (dist > lookahead) //원하는 점 찾으면 종료
            {
                target_bl = pose_bl;
                found = true;
                break;
            }

          
        }

        // !found : 경로 끝에 가까울 때, lookahead보다 남은 경로가 짧을 때
        if (!found)
        {
            try {
                auto pose = path_.poses.back();
                pose.header.stamp = rclcpp::Time(0);

                tf_buffer_->transform(pose, target_bl, "base_link");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Fallback TF FAIL: %s", ex.what());
                return;
            }
        }

        // -----------------------------
        // 🔥 target smoothing  ,필터 (Low-pass)
        //    효과: 경로 바뀔 때 튐 방지 ,부드러운 움직임
        // -----------------------------
        // target.x = 0.7 * prev_target_.x + 0.3 * target.x;
        // target.y = 0.7 * prev_target_.y + 0.3 * target.y;
        // prev_target_ = target;

        // -----------------------------
        // 3. robot frame transform ,로봇 좌표계 변환
        // -----------------------------
        // double dx = target.x - robot_x;
        // double dy = target.y - robot_y;

        double dx = target_bl.pose.position.x;  //로봇과 떨어진 x 좌표
        double dy = target_bl.pose.position.y;
        double L = hypot(dx , dy);

        // | 값   | 의미  |
        // | --- | --- |
        // | x_r | 앞/뒤 |
        // | y_r | 좌/우 |

        // double x_r = cos(robot_yaw) * dx + sin(robot_yaw) * dy;
        // double y_r = -sin(robot_yaw) * dx + cos(robot_yaw) * dy;

        // double L = hypot(x_r, y_r);   //  목표까지 거리

        // -----------------------------
        // 4. pure pursuit
        // -----------------------------
       // double curvature = 2.0 * y_r / (L * L + 1e-6);  // 회전해야 하는 정도 (곡률)
        // -----------------------------
        double curvature = (2.0 * dy) / (L * L);
        // 🎯 각도 에러 기반 회전
        // -----------------------------

            // ================= 4. 속도 프로파일 =================
        double max_linear = 0.15;//0.2;
        double max_angular = 1.2;

        double angle_error  = atan2(dy, dx);
        double angle_abs = fabs(angle_error);
      
        // 🔥 곡률 기반 감속 (핵심)
        double curvature_scale = 1.0 / (1.0 + fabs(curvature) * 2.0);
        // 🔥 각도 기반 감속
        //double angle_scale = exp(-angle_abs * 2.0);
        // 🔥 목표 근처 감속
        double goal_yaw = tf2::getYaw( path_.poses.back().pose.orientation); //목표점 에서 바라볼 방향
        double goal_dx = path_.poses.back().pose.position.x - robot_x;
        double goal_dy = path_.poses.back().pose.position.y - robot_y;
        double goal_dist = hypot(goal_dx, goal_dy);

        //double goal_scale = std::clamp(goal_dist / 1.0, 0.2, 1.0);

        double kp_vc = 1.;
        double kp_ang = 1.5;
        double ki_ang = 1.;
       
        double goal_yaw_error = 0.;
        double goal_yaw_error_I = 0.;

        double linear = 0.;
        double angular = 0. ;
        

       
    //    RCLCPP_INFO(this->get_logger(),"goal_state_%d Goal yaw %f R yaw %f IMU %f",goal_state_,goal_yaw,robot_yaw,imu_value);
RCLCPP_INFO(this->get_logger(),"goal_state_%d Goal yaw %f R yaw %f angle_error %f dx %f dy %f rX %f ry %f ", goal_state_,goal_yaw,robot_yaw,angle_error,dx,dx,robot_x,robot_y);
        switch (goal_state_)
        {
            case GoalState::DRIVE:
            {
                // 일반 주행
                //linear = max_linear * curvature_scale * angle_scale * goal_scale;
                //angular = curvature * linear;
                

                linear = goal_dist * kp_vc;
                //angular = angle_error * kp_ang ;
                angular = std::clamp(angle_error * kp_ang,
                                        -max_angular, max_angular);
                 // 회전 우선 조건
                if (angle_abs > 0.25 ) // 15도   //0.7 40도
                {
                    linear = 0.0;
                }
                // =================  목표 도착 =================
                if (goal_dist < 0.15)
                {
                    goal_state_ = GoalState::STOP_WAIT;
                    state_start_time_ = this->now();
                }
                break;
            }

            case GoalState::STOP_WAIT:
            {
                linear = 0.0;
                angular = 0.0;

                double elapsed = (this->now() - state_start_time_).seconds();

                if (elapsed > 1.0)
                {
                    goal_state_ = GoalState::ALIGN;
                    goal_yaw_error_I = 0.;
                }
                break;
            }

            case GoalState::ALIGN:
            {


                linear = 0.0;
                goal_yaw_error = goal_yaw - robot_yaw;
                // while(goal_yaw_error > M_PI) theta_e -= 2*M_PI;
                // while(goal_yaw_error < -M_PI) theta_e += 2*M_PI;
                goal_yaw_error = atan2(sin(goal_yaw_error), cos(goal_yaw_error));
                goal_yaw_error_I += goal_yaw_error;
                goal_yaw_error_I  = std::clamp(goal_yaw_error_I, -0.1, 0.1); //0.05 2배

                if (fabs(goal_yaw_error) > 0.05)
                {
                    angular = std::clamp(goal_yaw_error * kp_ang + goal_yaw_error_I * ki_ang,
                                        -max_angular, max_angular);
                }
                else
                {
                    angular = 0.0;
                    goal_yaw_error_I = 0.;
                    //state_start_time_ = this->now();

                    goal_state_ = GoalState::DONE;

                    
                }
                break;
            }

            case GoalState::DONE:
            {
                linear = 0.0;
                angular = 0.0;

                // if (goal_dist > 0.25)
                // {
                //     goal_state_ = GoalState::DRIVE;
                //    // state_start_time_ = this->now();
                // }


                // double elapsed = (this->now() - state_start_time_).seconds();
                // if (elapsed > 1.0)
                // {
                //     goal_state_ = GoalState::DRIVE;
                // }
                break;
            }
        }

        
        // ================= publish =================
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(linear, -max_linear, max_linear);
        cmd.angular.z = std::clamp(angular, -max_angular, max_angular);

        //RCLCPP_INFO(this->get_logger(),"Vc %f  Herr %f ",cmd.linear.x, angular);

        cmd_pub_->publish(cmd);


    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_speed_ = fabs(msg->twist.twist.linear.x);
    }
double imu_value = 0;
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 각속도
        double wx = msg->angular_velocity.x;
        double wy = msg->angular_velocity.y;
        double wz = msg->angular_velocity.z;

        // 가속도
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // orientation (쿼터니언)
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
imu_value = qz;
        // RCLCPP_INFO(this->get_logger(),
        //     "IMU wz: %.2f ax: %.2f", wz, ax);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LookaheadController>());
    rclcpp::shutdown();
}