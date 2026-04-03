#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using nav2_costmap_2d::LETHAL_OBSTACLE;


namespace human_n_obstacle_layer
{

  class HumanLayer  : public nav2_costmap_2d::Layer
  {
  public:

    //----------------------------------------
    // 초기화
    //----------------------------------------
    void onInitialize() override
    {
      
      auto node = node_.lock();
      if (!node) {
        throw std::runtime_error("Node lock failed");   // 🔥 추가
      }
      //RCLCPP_INFO(node->get_logger(), "HumanLayer INIT START 🔥");
      // TF
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      // human
      human_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
        "/human_point", 10,
        std::bind(&HumanLayer::humanCallback, this, std::placeholders::_1));

       /* ---------- ultrasonic ---------- */

      ultra_left_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/ultra_left",10,
      std::bind(&HumanLayer::ultraLeftCallback,this,std::placeholders::_1));

      ultra_right_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/ultra_right",10,
      std::bind(&HumanLayer::ultraRightCallback,this,std::placeholders::_1));

      /* ---------- bumper ---------- */

      bumper_left_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/bumper_left",10,
      std::bind(&HumanLayer::bumperLeftCallback,this,std::placeholders::_1));

      bumper_right_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/bumper_right",10,
      std::bind(&HumanLayer::bumperRightCallback,this,std::placeholders::_1));

      /* ---------- PSD ---------- */

      psd_left_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/psd_left",10,
      std::bind(&HumanLayer::psdLeftCallback,this,std::placeholders::_1));

      psd_right_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/psd_right",10,
      std::bind(&HumanLayer::psdRightCallback,this,std::placeholders::_1));
      //RCLCPP_INFO(node->get_logger(), "HumanLayer INIT START 🔥🔥🔥🔥🔥🔥🔥🔥");
    
    }

    //----------------------------------------
    // 필수 override (🔥 없으면 abstract 에러)
    //----------------------------------------
    void reset() override
    {
      // human_x_ = msg->x;
      // human_y_ = msg->y;
      // has_human_ = true;
    }

    bool isClearable() override
    {
      return true;
    }

    
   
    
    //----------------------------------------
    // 데이터 콜백
    //----------------------------------------
    void humanCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
      human_x_ = msg->x;
      human_y_ = msg->y;
      has_human_ = true;
    }

     void psdLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      psd_left_=msg->range;
     
      if (psd_left_ < 0.006) //
      {   
        psd_left_flg = true; 
      }
      else if (psd_left_ > 0.0065)psd_left_flg = false; 

      if(psd_left_flg && !psd_prev_left_flg)  
      {
        snesor_kind_ = Sensor_Kind::PSD_LEFT;
        createBlockedZone();
      }
      psd_prev_left_flg = psd_left_flg;

    }
    void psdRightCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      psd_right_=msg->range;
      if (psd_right_ < 0.006) //
      {   
        psd_right_flg = true; 
      }
      else if (psd_right_ > 0.0065)psd_right_flg = false; 

      if(psd_right_flg && !psd_prev_right_flg)  
      {
        snesor_kind_ = Sensor_Kind::PSD_RIGHT;
        createBlockedZone();
      }
      psd_prev_right_flg = psd_right_flg;
    }

    void ultraLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      ultra_left_= msg->range;
      if(ultra_left_ == 0.0) ultra_left_ = 1.0;
      if (ultra_left_ < 0.04) //0.35 0.50 0.60
      {   
        ultra_left_flg = true; 
      }
      else if (ultra_left_ > 0.059)ultra_left_flg = false; 

      if(ultra_left_flg && !ultra_prev_left_flg)  
      {
        snesor_kind_ = Sensor_Kind::ULTRA_LEFT;
        createBlockedZone();
      }
      ultra_prev_left_flg = ultra_left_flg;
    }
    void ultraRightCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      ultra_right_= msg->range;
      if(ultra_right_ == 0.0) ultra_right_ = 1.0;
      if (ultra_right_ < 0.04)
      {
          ultra_right_flg = true; 
      }
      else if (ultra_right_ > 0.059)ultra_right_flg = false; 
      if(ultra_right_flg && !ultra_prev_right_flg)  
      {
        snesor_kind_ = Sensor_Kind::ULTRA_RIGHT;
        createBlockedZone();
      }
      ultra_prev_right_flg = ultra_right_flg;
    }
    void bumperLeftCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      
      bumper_left_= msg->data;
      if (bumper_left_ && !bumper_prev_left_) {
        snesor_kind_ = Sensor_Kind::BUMPER_LEFT;
        createBlockedZone();
      }
      bumper_prev_left_ = bumper_left_;
    }
    void bumperRightCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      bumper_right_= msg->data;
      if (bumper_right_ && !bumper_prev_right_) {
        snesor_kind_ = Sensor_Kind::BUMPER_RIGHT;
        createBlockedZone();
      }
      bumper_prev_right_ = bumper_right_;
      
    }

   


    //----------------------------------------
    // blocked zone 생성 🔥 핵심
    //----------------------------------------
   
    void createBlockedZone( )
    {
      auto node = node_.lock();
      
      try
      {
        // base_link = 로봇 기준 좌표, map = 전역 좌표
        // 👉 결과: tf에 현재 로봇 위치 (x, y, yaw) 얻음
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
       //  auto tf = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);

        // std::string global_frame = layered_costmap_->getGlobalFrameID();

        // auto tf = tf_buffer_->lookupTransform(global_frame, "base_link", tf2::TimePointZero);

        double robot_x = tf.transform.translation.x;
        double robot_y = tf.transform.translation.y;

        //yaw(방향) 계산,“앞쪽”이 어디인지 알아야 함
        tf2::Quaternion q(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 🔥 앞 방향
        double forward = 0.2; //로봇 앞쪽 20cm 위치
        double side = 0.07; //로봇 옆쪽 7cm 위치
        // 🔥 좌우 오프셋 (핵심)
        
        if(snesor_kind_ == Sensor_Kind::BUMPER_LEFT) side = 0.07;
        else if(snesor_kind_ == Sensor_Kind::BUMPER_RIGHT) side = -0.07;
        else if(snesor_kind_ == Sensor_Kind::ULTRA_LEFT) side = 0.07;
        else if(snesor_kind_ == Sensor_Kind::ULTRA_RIGHT) side = -0.07;
        else if(snesor_kind_ == Sensor_Kind::PSD_LEFT) side = 0.15;
        else if(snesor_kind_ == Sensor_Kind::PSD_RIGHT) side = -0.15;
        
        // 충돌 위치 계산 (핵심🔥)
        BlockedPoint p;

        p.x = robot_x + cos(yaw) * forward - sin(yaw) * side;
        p.y = robot_y + sin(yaw) * forward + cos(yaw) * side;
        p.yaw = yaw;
        p.side = side; 
        p.sensor_kind  = snesor_kind_;
        // 앞 방향 = (cos(yaw), sin(yaw))
        // 좌 방향 = (-sin(yaw), cos(yaw))
        RCLCPP_WARN(node->get_logger(),
       "CREATE BLOCK x=%.2f y=%.2f", p.x, p.y);

        //RCLCPP_INFO(node->get_logger(),"Rx=%f, Ry=%f, Rw=%f Px=%f, Py=%f, Pw=%f Ps =%f ",robot_x,robot_y,yaw,p.x,p.y,p.yaw,p.side);
        
        p.time = node->now();

        blocked_points_.push_back(p);

        if (blocked_points_.size() > 5) //5 개이상 포인트 개수면 
        blocked_points_.erase(blocked_points_.begin()); //가장 오래된(첫 번째) 블럭을 삭제
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(node->get_logger(), "TF ERROR: %s", ex.what());
        return;
      }
    }

    //----------------------------------------
    // costmap 영역 갱신
    //----------------------------------------
    // void updateBounds(double, double, double,
    //                   double* min_x, double* min_y,
    //                   double* max_x, double* max_y) override
    // {
    //   // 🔥 항상 일정 영역 갱신
    //   double range = 2.0;  // 2m

    //   *min_x = -range;
    //   *min_y = -range;
    //   *max_x =  range;
    //   *max_y =  range;

    //   if (has_human_)
    //   {
    //     *min_x = std::min(*min_x, human_x_);
    //     *min_y = std::min(*min_y, human_y_);
    //     *max_x = std::max(*max_x, human_x_);
    //     *max_y = std::max(*max_y, human_y_);
    //   }
    // }

    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                  double* min_x, double* min_y,
                  double* max_x, double* max_y) override
    {
      auto node = node_.lock();
      if (!node) return;

      // RCLCPP_WARN(node->get_logger(),
      // "[HumanLayer][%s] updateBounds",
      // layered_costmap_->getGlobalFrameID().c_str());

      double range = 3.0;

      // 🔥 반드시 robot 기준으로
      *min_x = robot_x - range;
      *min_y = robot_y - range;
      *max_x = robot_x + range;
      *max_y = robot_y + range;

      // human 위치 포함
      if (has_human_)
      {
        *min_x = std::min(*min_x, human_x_);
        *min_y = std::min(*min_y, human_y_);
        *max_x = std::max(*max_x, human_x_);
        *max_y = std::max(*max_y, human_y_);
      }

      // 🔥 blocked zone도 반영 (중요)
      for (auto &p : blocked_points_)
      {
        *min_x = std::min(*min_x, p.x);
        *min_y = std::min(*min_y, p.y);
        *max_x = std::max(*max_x, p.x);
        *max_y = std::max(*max_y, p.y);
      }
    }

    //----------------------------------------
    // obstacle 생성 costmap 적용
    //----------------------------------------
    void updateCosts(nav2_costmap_2d::Costmap2D& costmap,
                    int, int, int, int) override
    {
  //     if (blocked_points_.empty())
  // return;
      auto node = node_.lock();
      if (!node) return;
      // 🔥 global_costmap에서만 동작
      if (layered_costmap_->getGlobalFrameID() != "map")
        return;

      auto now = node->now();

      unsigned int mx, my;
      
      //----------------------------------------
      // 1️⃣ 사람 (원형 느낌)
      //----------------------------------------
      has_human_ = 0;
      if (has_human_)
      {
          if (costmap.worldToMap(human_x_, human_y_, mx, my))
          {
             

              int invader_A[8][11] =
              {
                {0,0,1,0,0,0,0,0,1,0,0},
                {0,0,0,1,0,0,0,1,0,0,0},
                {0,0,1,1,1,1,1,1,1,0,0},
                {0,1,1,0,1,1,1,0,1,1,0},
                {1,1,1,1,1,1,1,1,1,1,1},
                {1,0,1,1,1,1,1,1,1,0,1},
                {1,0,1,0,0,0,0,0,1,0,1},
                {0,0,0,1,1,0,1,1,0,0,0},
               
              };

               int invader_B[8][11] =
              {
                {0,0,1,0,0,0,0,0,1,0,0},
                {0,0,0,1,0,0,0,1,0,0,0},
                {1,0,1,1,1,1,1,1,1,0,1},
                {1,1,1,0,1,1,1,0,1,1,1},
                {1,1,1,1,1,1,1,1,1,1,1},
                {0,0,1,1,1,1,1,1,1,0,0},
                {0,0,1,0,0,0,0,0,1,0,0},
                {0,1,0,0,0,0,0,0,0,1,0},
               
              };

              static int cnt = 0;
              cnt++;

              static bool toggle = false;
              if(cnt % 5 == 0)
                toggle = !toggle;

              int (*invader)[11] = toggle ? invader_A : invader_B;

              for(int i=0; i<8; i++)
              {
                for(int j=0; j<11; j++)
                {
                  if(invader[i][j] == 1)
                  {
                    costmap.setCost(mx + j - 5, my + i - 4, 200);
                  }
                }
              }
          }
      }

     
double size = 1.5;
      for (auto &p : blocked_points_)
      {
        double yaw = p.yaw;   
        double base_side = p.side;   // 🔥 저장된 좌/우 위치
        double dt = (now - p.time).seconds();

        if((p.sensor_kind == Sensor_Kind::BUMPER_LEFT) || (p.sensor_kind == Sensor_Kind::BUMPER_RIGHT))  size = 0.1;
        else if((p.sensor_kind == Sensor_Kind::ULTRA_LEFT) || (p.sensor_kind == Sensor_Kind::ULTRA_RIGHT))  size = 0.05;
        else if((p.sensor_kind == Sensor_Kind::PSD_LEFT) || (p.sensor_kind == Sensor_Kind::PSD_RIGHT))  size = 0.01;
        if (dt < 20.0)
        {
          for(double i=-size; i<size; i += 0.02)          // forward
          {
            double d = 0.2 + i * 0.1;

            for(double w=-size; w<=size; w += 0.02)      // side
            {
              double side = base_side + w * 0.12;  //0.15 너무크게 회전 0.10  은 조금 작은느낌 // 🔥 중심 + 확장

              double px = p.x + cos(yaw)*d - sin(yaw)*side;
              double py = p.y + sin(yaw)*d + cos(yaw)*side;

              if (costmap.worldToMap(px, py, mx, my))
              {
                costmap.setCost(mx, my, LETHAL_OBSTACLE);
              }
            }
          }
        }

      }

      //----------------------------------------
      // 3️⃣ 범퍼 (강제 차단)
      //----------------------------------------
//      double hold_time = 1.0;  // 1초 유지
 //     bool bumper_active = false;

      // if(bumper_left_)
      // {
      //     double dt = (rclcpp::Clock().now() - bumper_left_last_time_).seconds();

      //     if(dt < hold_time)
      //         bumper_left_active = true;
      //     else
      //         bumper_hit_ = false;  // 자동 해제
      // }


      // if (bumper_left_)
      // {
      //     double obs_x = 0.2; // 로봇 중심에서 20cm 앞
      //     double obs_y = 0.01;            // 로봇 중심에서 좌측으로 1cm

      //     if (costmap.worldToMap(obs_x, obs_y, mx, my))
      //     {
      //       for(int dx=-1; dx<=1; dx++)
      //       {
      //         for(int dy=0; dy<=4; dy++)
      //         {
      //           costmap.setCost(mx+dx, my+dy, LETHAL_OBSTACLE);
      //         }
      //       }
      //     }
      // }

      // if (bumper_right_)
      // {
      //     double obs_x = 0.2;  // 로봇 중심에서 20cm 앞
      //     double obs_y = -0.01;              // 로봇 중심에서 우측으로 1cm

      //     if (costmap.worldToMap(obs_x, obs_y, mx, my))
      //     {
      //       for(int dx=-1; dx<=1; dx++)      // 두께
      //       {
      //         for(int dy=0; dy>=-4; dy--)    //길이
      //         {
      //           costmap.setCost(mx+dx, my+dy, LETHAL_OBSTACLE);
      //         }
      //       }
      //     }
      // }


      //----------------------------------------
      // 오래된 제거
      //----------------------------------------
      blocked_points_.erase(
        std::remove_if(blocked_points_.begin(), blocked_points_.end(),
          [&](const BlockedPoint &p)
          {
            return (now - p.time).seconds() > 60.0;
          }),
        blocked_points_.end());

    }

  private:

    
    enum class Sensor_Kind
    {
        NON = 0,
        BUMPER_LEFT,
        BUMPER_RIGHT,
        ULTRA_LEFT,
        ULTRA_RIGHT,
        PSD_LEFT,
        PSD_RIGHT,
        HUMAN,
        BOTTOM
    };

    Sensor_Kind snesor_kind_;
    //----------------------------------------
    // blocked 구조
    //----------------------------------------
    struct BlockedPoint
    {
      double x;
      double y;
      double yaw;   
      double side;  
      Sensor_Kind sensor_kind;   // 센서 종류 2026,03,30
      rclcpp::Time time;
    };

    std::vector<BlockedPoint> blocked_points_;

    //----------------------------------------
    // 상태 변수
    //----------------------------------------
    double human_x_ = 0.0;
    double human_y_ = 0.0;
    bool has_human_ = false;

   

    float psd_left_ = 1.0;
    float psd_right_ = 1.0;
    bool psd_left_flg = false;
    bool psd_right_flg = false;
    bool psd_prev_left_flg = false;
    bool psd_prev_right_flg = false;

    float ultra_left_ = 1.0;
    float ultra_right_ = 1.0;
    bool ultra_left_flg = false;
    bool ultra_right_flg = false;
    bool ultra_prev_left_flg = false;
    bool ultra_prev_right_flg = false;

    bool bumper_left_ = false;
    bool bumper_right_ = false;
    bool bumper_prev_left_ = false;
    bool bumper_prev_right_ = false;

    
    //----------------------------------------
    // subscriber
    //----------------------------------------

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr human_sub_;
        
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_right_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr psd_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr psd_right_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_right_sub_;



    rclcpp::Time bumper_left_last_time_;
    rclcpp::Time bumper_right_last_time_;
   
    rclcpp::Time ultra_left_last_time_;
    rclcpp::Time ultra_right_last_time_;

    rclcpp::Time psd_left_last_time_;
    rclcpp::Time psd_right_last_time_;

    //----------------------------------------
    // TF
    //----------------------------------------
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



  };


}

PLUGINLIB_EXPORT_CLASS(human_n_obstacle_layer::HumanLayer, nav2_costmap_2d::Layer)