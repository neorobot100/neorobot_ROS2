#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
struct Cluster
{
    std::vector<std::pair<float, float>> pts; // (r, angle)
};

class LidarHumanDetect : public rclcpp::Node
{
public:
    LidarHumanDetect() : Node("lidar_human_leg_detect")
    {
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            std::bind(&LidarHumanDetect::scanCallback, this, std::placeholders::_1));
    
        target_pub_ = create_publisher<geometry_msgs::msg::Point>("/leg_target", 10);

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/human_marker", 10);

        start_time_ = this->now();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    rclcpp::Time start_time_;

    float prev_x_ = 0.0;
    float prev_y_ = 0.0;
    bool has_prev_ = false;

    // =============================
    std::pair<float, float> getCenter(const Cluster &c)
    {
        float x = 0, y = 0;

        for (auto &p : c.pts)
        {
            float r = p.first;
            float a = p.second;

            x += r * cos(a);
            y += r * sin(a);
        }

        x /= c.pts.size();
        y /= c.pts.size();

        return {x, y};
    }

    // =============================
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::vector<Cluster> clusters;
        Cluster current;

        float prev_r = 0.0;
        bool first = true;

        // ===== 1. 클러스터링 =====
        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float r = scan->ranges[i];

            if (std::isinf(r) || r < 0.15 || r > 3.0)
                continue;

            float angle = scan->angle_min + i * scan->angle_increment;

            if (first)
            {
                current.pts.push_back({r, angle});
                prev_r = r;
                first = false;
                continue;
            }

            if (fabs(r - prev_r) < 0.08)
            {
                current.pts.push_back({r, angle});
            }
            else
            {
                if (!current.pts.empty())
                    clusters.push_back(current);

                current.pts.clear();
                current.pts.push_back({r, angle});
            }

            prev_r = r;
        }

        if (!current.pts.empty())
            clusters.push_back(current);

        // ===== 2. 다리 후보 =====
        std::vector<Cluster> legs;

        for (auto &c : clusters)
        {
            if (c.pts.size() < 3 || c.pts.size() > 30)
                continue;

            float min_r = 999, max_r = 0;

            for (auto &p : c.pts)
            {
                min_r = std::min(min_r, p.first);
                max_r = std::max(max_r, p.first);
            }

            float width = max_r - min_r;

            // 🔥 기존 조건 유지
            if (!(width > 0.05 && width < 0.25))
                continue;

            // 🔥 여기부터 추가 (타원 필터)
            float min_x=999, max_x=-999;
            float min_y=999, max_y=-999;

            for (auto &p : c.pts)
            {
                float r = p.first;
                float a = p.second;

                float x = r * cos(a);
                float y = r * sin(a);

                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }

            float dx = max_x - min_x;
            float dy = max_y - min_y;

            float ratio = dx / (dy + 1e-5);

            // 🔥 타원 형태만 통과
            if (ratio < 0.3 || ratio > 3.0)
                continue;

            // ✅ 최종 통과
            legs.push_back(c);
        }
    //     RCLCPP_INFO(this->get_logger(),
    // "clusters: %ld, legs: %ld",
    // clusters.size(), legs.size());

        // ===== 3. 페어 찾기 =====
        bool found = false;
        float best_score = 999.0;

        float target_x = 0, target_y = 0;
        float confidence = 0.0;

        for (size_t i = 0; i < legs.size(); i++)
        {
            for (size_t j = i + 1; j < legs.size(); j++)
            {
                auto c1 = getCenter(legs[i]);
                auto c2 = getCenter(legs[j]);

                float dx = c1.first - c2.first;
                float dy = c1.second - c2.second;

                float dist = hypot(dx, dy);

                // 🔥 1. 사람 다리 간격 필터
                if (dist < 0.15 || dist > 0.5)
                    continue;

                // 🔥 2. 두 다리 거리 비슷해야 함
                float d1 = hypot(c1.first, c1.second);
                float d2 = hypot(c2.first, c2.second);

                if (fabs(d1 - d2) > 0.2)
                    continue;

                float cx = (c1.first + c2.first) / 2.0;
                float cy = (c1.second + c2.second) / 2.0;

                // 🔥 3. score 계산
                float score = 0.0;
                score += fabs(d1 - d2) * 2.0;
                score += fabs(dist - 0.25);
                score += fabs(atan2(cy, cx)) * 0.5;

                // 🔥 4. 최적 선택
                float move = 0.0;

                if (has_prev_)
                {
                    move = hypot(cx - prev_x_, cy - prev_y_);
                }

                // 🔥 핵심: 움직이는 것만 선택
                double elapsed = (this->now() - start_time_).seconds();

                bool init_phase = (elapsed < 1.0);  // 처음 1초

                if (init_phase || !has_prev_ || move > 0.01)
                {
                    if (score < best_score)
                    {
                        best_score = score;
                        target_x = cx;
                        target_y = cy;
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            confidence = 1.0;

            if (best_score > 0.5) confidence -= 0.3;
            if (best_score > 1.0) confidence -= 0.5;
        }
        
        // ===== 4. publish =====
        geometry_msgs::msg::Point msg;

        // 🔥 튐 방지 (핵심!!)
        if (found && has_prev_)
        {
            float jump = hypot(target_x - prev_x_, target_y - prev_y_);

            if (jump > 0.5)   // 50cm 이상 튀면 무시
            {
                found = false;
            }
        }
        
        if (found && has_prev_)
        {
            target_x = 0.7 * prev_x_ + 0.3 * target_x;
            target_y = 0.7 * prev_y_ + 0.3 * target_y;
        }

        if (found)
        {
            msg.x = target_x;
            msg.y = target_y;
            msg.z = confidence;
            publishMarker(target_x, target_y, confidence);

            prev_x_ = target_x;
            prev_y_ = target_y;
            has_prev_ = true;
        }
        else
        {
            msg.x = 0;
            msg.y = 0;
            msg.z = 0;
            // 🔥 사람 없으면 marker 제거
           deleteMarker();
        }

        target_pub_->publish(msg);
    }

    void publishMarker(float x, float y, float confidence)
    {


        visualization_msgs::msg::Marker sphere;

        sphere.header.frame_id = "base_link";
        sphere.header.stamp = this->now();

        sphere.ns = "human";
        sphere.id = 0;

        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;

        sphere.pose.position.x = x;
        sphere.pose.position.y = y;
        sphere.pose.position.z = 0.2;

        sphere.scale.x = 0.3;
        sphere.scale.y = 0.3;
        sphere.scale.z = 0.3;

        sphere.color.a = 1.0;
        sphere.color.r = 1.0;
        sphere.color.g = 0.0;
        sphere.color.b = 0.0;

        marker_pub_->publish(sphere);

        visualization_msgs::msg::Marker arrow;

arrow.header.frame_id = "base_link";
arrow.header.stamp = this->now();

arrow.ns = "human";
arrow.id = 1;

arrow.type = visualization_msgs::msg::Marker::ARROW;
arrow.action = visualization_msgs::msg::Marker::ADD;

// 🔥 시작점 (로봇)
geometry_msgs::msg::Point p_start;
p_start.x = 0.0;
p_start.y = 0.0;
p_start.z = 0.1;

// 🔥 끝점 (사람)
geometry_msgs::msg::Point p_end;
p_end.x = x;
p_end.y = y;
p_end.z = 0.1;

arrow.points.push_back(p_start);
arrow.points.push_back(p_end);

// 두께
arrow.scale.x = 0.05;  // shaft
arrow.scale.y = 0.1;   // head
arrow.scale.z = 0.1;

arrow.color.a = 1.0;
arrow.color.r = 0.0;
arrow.color.g = 1.0;
arrow.color.b = 0.0;

marker_pub_->publish(arrow);


//         visualization_msgs::msg::Marker marker;

//         marker.header.frame_id = "map";
//         marker.header.stamp = this->now();

//         marker.ns = "human";
//         marker.id = 0;

//         marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         // marker.pose.position.x = x;
//         // marker.pose.position.y = y;
//         // marker.pose.position.z = 0.5;
// marker.pose.position.x = 1.0;
// marker.pose.position.y = 0.0;
// marker.pose.position.z = 0.5;

//         marker.scale.z = 0.1;

//         marker.color.a = 1.0;

//         if (confidence > 0.7)
//         {
//             marker.color.r = 0.0;
//             marker.color.g = 1.0;
//         }
//         else
//         {
//             marker.color.r = 1.0;
//             marker.color.g = 0.0;
//         }

//         //marker.text = "👾";
//         marker.text = "HUMAN";

//         marker_pub_->publish(marker);
    
    }

    void deleteMarker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();

        marker.ns = "human";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETE;

        marker_pub_->publish(marker);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarHumanDetect>());
    rclcpp::shutdown();
    return 0;
}