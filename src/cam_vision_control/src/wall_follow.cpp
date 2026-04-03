#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class WallFollower : public rclcpp::Node
{
public:
    WallFollower() : Node("wall_follower")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&WallFollower::imageCallback, this, std::placeholders::_1));

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        kp_ = 0.004;
        kd_ = 0.002;
        prev_error_ = 0.0;
         RCLCPP_INFO(this->get_logger(), "wall follow start");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;  //ROS 이미지 → OpenCV 이미지 변환
        } catch (...) {
            return;
        }

        int h = frame.rows;
        int w = frame.cols;

        // ROI (아래쪽만)
        cv::Mat roi = frame(cv::Range(h * 0.6, h), cv::Range::all()); //화면 아래 40%만 사용

        cv::Mat gray, edges;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges, 50, 150);   //밝기 변화 큰 부분만 추출

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 30, 10);  //엣지 → “직선”으로 변환

        double left_x = 0, right_x = 0;
        int left_count = 0, right_count = 0;

        for (auto &l : lines)
        {
            int x1 = l[0], y1 = l[1];
            int x2 = l[2], y2 = l[3];

            double slope = (double)(y2 - y1) / (x2 - x1 + 1e-5);  // 좌/우 벽 분리

            // 거의 수직선만 사용
            if (fabs(slope) < 0.5) continue;

            int mid_x = (x1 + x2) / 2;

            if (mid_x < w/2)
            {
                left_x += mid_x;
                left_count++;
            }
            else
            {
                right_x += mid_x;
                right_count++;
            }
        }

        if (left_count == 0 || right_count == 0)
            return; // 한쪽 벽 없으면 패스
        
        // 평균 위치 계산    
        left_x /= left_count;
        right_x /= right_count;

        //중앙 계산
        double center = (left_x + right_x) / 2.0;
        double image_center = w / 2.0;

        double error = image_center - center;

        // PID (P + D)
        double d_error = error - prev_error_;
        prev_error_ = error;

        double angular = kp_ * error + kd_ * d_error;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.15;
        cmd.angular.z = angular;

        pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "err: %.2f", error);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    double kp_, kd_;
    double prev_error_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}