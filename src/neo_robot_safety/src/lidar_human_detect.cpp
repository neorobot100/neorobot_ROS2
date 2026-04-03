#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct Cluster
{
    float x;
    float y;
    float width;
    int size;   // 🔥 추가
};

struct Track
{
    int id;
    float x;
    float y;
    float vx;
    float vy;
    rclcpp::Time last_update;
};

class HumanDetector : public rclcpp::Node
{
public:
    HumanDetector() : Node("human_detector")
    {
          //CLCPP_ERROR(this->get_logger(), "노드 시작됨");

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            scanCallback(msg);  // 🔥 반드시 호출
        });
        
        human_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/human_point", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&HumanDetector::odomCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr human_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    float robot_vx_ = 0.0;
    float robot_vy_ = 0.0;

    std::vector<Track> tracks_;
    int next_id_ = 0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //----------------------------------------
    // 1. scan → cluster
    //----------------------------------------
    std::vector<Cluster> makeClusters(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<Cluster> clusters;

        float prev_x = 0, prev_y = 0;
        std::vector<std::pair<float,float>> current_cluster;

        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            float r = msg->ranges[i];
            if(r < msg->range_min || r > msg->range_max) continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            float x = r * cos(angle);
            float y = r * sin(angle);

            if(current_cluster.empty())
            {
                current_cluster.push_back({x,y});
            }
            else
            {
                float dx = x - prev_x;
                float dy = y - prev_y;
                float dist = sqrt(dx*dx + dy*dy);

                if(dist < 0.4) // 같은 클러스터
                {
                    current_cluster.push_back({x,y});
                }
                else
                {
                    clusters.push_back(makeCluster(current_cluster));
                    current_cluster.clear();
                    current_cluster.push_back({x,y});
                }
            }

            prev_x = x;
            prev_y = y;
        }

        if(!current_cluster.empty())
            clusters.push_back(makeCluster(current_cluster));

        return clusters;
    }

    Cluster makeCluster(const std::vector<std::pair<float,float>>& pts)
    {
        Cluster c;

        float sum_x = 0, sum_y = 0;
        for(auto &p : pts)
        {
            sum_x += p.first;
            sum_y += p.second;
        }

        c.x = sum_x / pts.size();
        c.y = sum_y / pts.size();

        // width 계산
        float dx = pts.front().first - pts.back().first;
        float dy = pts.front().second - pts.back().second;
        c.width = sqrt(dx*dx + dy*dy);
        c.size = pts.size();

        return c;
    }

    //----------------------------------------
    // 2. tracking
    //----------------------------------------
    int findTrack(float x, float y)
    {
        int best = -1;
        float min_dist = 999;

        for(int i = 0; i < tracks_.size(); i++)
        {
            float dx = tracks_[i].x - x;
            float dy = tracks_[i].y - y;
            float d = sqrt(dx*dx + dy*dy);

            if(d < 0.5 && d < min_dist)
            {
                min_dist = d;
                best = i;
            }
        }

        return best;
    }

    void updateTracks(const std::vector<Cluster>& clusters)
    {
        auto now_t = now();

        for(const auto& c : clusters)
        {
            int idx = findTrack(c.x, c.y);

            if(idx >= 0)
            {
                auto &t = tracks_[idx];
                float dt = (now_t - t.last_update).seconds();

                if(dt > 0.001)
                {
                    t.vx = (c.x - t.x) / dt;
                    t.vy = (c.y - t.y) / dt;
                }

                t.x = c.x;
                t.y = c.y;
                t.last_update = now_t;
            }
            else
            {
                Track t;
                t.id = next_id_++;
                t.x = c.x;
                t.y = c.y;
                t.vx = 0;
                t.vy = 0;
                t.last_update = now_t;
                tracks_.push_back(t);
            }
        }

        // 오래된 트랙 삭제
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                [&](const Track& t)
                {
                    return (now_t - t.last_update).seconds() > 1.0;
                }),
            tracks_.end()
        );
    }

    //----------------------------------------
    // 3. 사람 판단
    //----------------------------------------
    bool isHuman(const Cluster& c, const Track& t)
    {
        float dist = sqrt(c.x*c.x + c.y*c.y);

        // 🔥 로봇 속도 제거
        float obj_vx = t.vx - robot_vx_;
        float obj_vy = t.vy - robot_vy_;

        float speed = sqrt(obj_vx*obj_vx + obj_vy*obj_vy);

        // RCLCPP_WARN(this->get_logger(),
        //     "CHECK dist=%.2f width=%.2f size=%d speed=%.2f y=%.2f",
        //     dist, c.width, c.size, speed, c.y);

        return (
            dist > 0.5 && dist < 1.5 &&     // 거리
            c.width > 0.25 && c.width < 0.6 &&  // 사람 폭
            fabs(c.y) < 0.4 &&              // 정면
            c.size > 5 &&                   // 노이즈 제거
            speed > 0.1 && speed < 2.0      // 🔥 핵심 (움직임)
        );
    }

    //----------------------------------------
    // 4. callback
    //----------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto clusters = makeClusters(msg);
      
//         RCLCPP_WARN(this->get_logger(), "cluster 개수: %ld", clusters.size());

// for(const auto& c : clusters)
// {
//     float dist = sqrt(c.x*c.x + c.y*c.y);

//     RCLCPP_WARN(this->get_logger(),
//         "CLUSTER dist=%.2f width=%.2f x=%.2f y=%.2f",
//         dist, c.width, c.x, c.y);
// }


        updateTracks(clusters);

        bool human_detected = false;

        for(const auto& c : clusters)
        {
            int idx = findTrack(c.x, c.y);
            if(idx < 0) continue;

            if(isHuman(c, tracks_[idx]))
            {
                //human_detected = true;
                // geometry_msgs::msg::Point p;
                // p.x = c.x;
                // p.y = c.y;
                // p.z = 0.0;

                // human_pub_->publish(p);

                geometry_msgs::msg::PointStamped in, out;

                in.header.frame_id = "laser_frame";   // 🔥 중요
                in.point.x = c.x;
                in.point.y = c.y;
                in.point.z = 0.0;

                try
                {
                    out = tf_buffer_->transform(in, "base_link");
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "TF fail: %s", ex.what());
                    return;
                }

                human_pub_->publish(out.point);
                
                break;
            }
        }

        // std_msgs::msg::Bool out;
        // out.data = human_detected;
        // human_pub_->publish(out);
        
        
        
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_vx_ = msg->twist.twist.linear.x;
        robot_vy_ = msg->twist.twist.linear.y;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanDetector>());
    rclcpp::shutdown();
    return 0;
}


