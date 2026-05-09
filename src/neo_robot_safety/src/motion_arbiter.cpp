// ------------------------------------------------------------
// Motion Arbiter
//
// Nav2 ----> /cmd_vel_nav
// Joy  ----> /cmd_vel_joy
//                  ↓
//            Motion Arbiter
//                  ↓
//             /cmd_vel  
//                  ↓
//              Motor FW
//
// Sensors
// LiDAR
// PSD (front 45deg)
// Ultrasonic
// Bumper
// Cliff (5)
//
// Industrial Safety State Machine
// ------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/string.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
/*
Safety State

NORMAL        정상
CAUTION       감속
SOFT_STOP     정지
HARD_STOP     충돌
RECOVERY_BACK 후진
RECOVERY_TURN 회피
*/

enum class SafetyState
{
  NORMAL,
  CAUTION,
  SOFT_STOP,
  HARD_STOP,
  RECOVERY_BACK,
  RECOVERY_TURN,
  E_STOP
};

class MotionArbiter : public rclcpp::Node
{
public:

    MotionArbiter() : Node("motion_arbiter")
    {

        clock_ = this->get_clock();
            
        /* ---------- cmd_vel sources ---------- */
        sub_lookahead_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_lookahead", 10,
        std::bind(&MotionArbiter::lookaheadCallback, this, std::placeholders::_1));


        nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_nav",10,
        std::bind(&MotionArbiter::navCallback,this,std::placeholders::_1));

        joy_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_joy",10,
        std::bind(&MotionArbiter::joyCallback,this,std::placeholders::_1));

        localization_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_localization",10,
        std::bind(&MotionArbiter::localizationCallback,this,std::placeholders::_1));

        /* ---------- lidar ---------- */
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&MotionArbiter::scanCallback, this, std::placeholders::_1));

        /* ---------- bumper ---------- */

        bumper_left_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/bumper_left",10,
        std::bind(&MotionArbiter::bumperLeftCallback,this,std::placeholders::_1));

        bumper_right_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/bumper_right",10,
        std::bind(&MotionArbiter::bumperRightCallback,this,std::placeholders::_1));

        /* ---------- ultrasonic ---------- */

        ultra_left_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/ultra_left",10,
        std::bind(&MotionArbiter::ultraLeftCallback,this,std::placeholders::_1));

        ultra_right_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/ultra_right",10,
        std::bind(&MotionArbiter::ultraRightCallback,this,std::placeholders::_1));

        /* ---------- PSD ---------- */

        psd_left_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/psd_left",10,
        std::bind(&MotionArbiter::psdLeftCallback,this,std::placeholders::_1));

        psd_right_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/psd_right",10,
        std::bind(&MotionArbiter::psdRightCallback,this,std::placeholders::_1));

        /* ---------- cliff sensors ---------- */

        cliff_fl_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cliff_fl",10,
        std::bind(&MotionArbiter::cliffFLCallback,this,std::placeholders::_1));

        cliff_fc_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cliff_fc",10,
        std::bind(&MotionArbiter::cliffFCCallback,this,std::placeholders::_1));

        cliff_fr_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cliff_fr",10,
        std::bind(&MotionArbiter::cliffFRCallback,this,std::placeholders::_1));

        cliff_rl_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cliff_rl",10,
        std::bind(&MotionArbiter::cliffRLCallback,this,std::placeholders::_1));

        cliff_rr_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cliff_rr",10,
        std::bind(&MotionArbiter::cliffRRCallback,this,std::placeholders::_1));

        /* wheel lift */
        wheel_lift_l_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/wheel_lift_l",10,
        std::bind(&MotionArbiter::wheelLiftLCallback,this,std::placeholders::_1));

        wheel_lift_r_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/wheel_lift_r",10,
        std::bind(&MotionArbiter::wheelLiftRCallback,this,std::placeholders::_1));

    
        /* ---------- cmd_vel output ---------- */

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",10);

        /* ---------- 이벤트 메세지 ---------- */
        event_pub_ = create_publisher<std_msgs::msg::String>(
        "/safety_manager/event", 10);

        human_sub_ = create_subscription<geometry_msgs::msg::Point>(
        "/leg_target", 10,
        std::bind(&MotionArbiter::humanCallback, this, std::placeholders::_1));

        /* ---------- control loop ---------- */

        timer_ = create_wall_timer(
        50ms,
        std::bind(&MotionArbiter::controlLoop,this));


        last_nav_time_ = clock_->now();
        last_joy_time_ = clock_->now();
        last_lookahead_time_ = clock_->now();
        last_localization_time_ = clock_->now();

        nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");


        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MotionArbiter::odomCallback, this, std::placeholders::_1));

        charge_onoff_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/charge_onoff", 10,
        std::bind(&MotionArbiter::charge_callback, this, std::placeholders::_1));     



        dock_stat_sub_ = create_subscription<std_msgs::msg::Int8>(
        "/dock_stat", 10,
        std::bind(&MotionArbiter::dock_stat_callback, this, std::placeholders::_1));

        undock_stat_sub_ = create_subscription<std_msgs::msg::Int8>(
        "/undock_stat", 10,
        std::bind(&MotionArbiter::undock_stat_callback, this, std::placeholders::_1));

    

    }

private:

    /* ---------- state ---------- */

    SafetyState state_ = SafetyState::NORMAL;

    geometry_msgs::msg::Point lidar_human_leg_;

    /* ---------- cmd sources ---------- */
    geometry_msgs::msg::Twist lookahead_cmd_;
    geometry_msgs::msg::Twist nav_cmd_;
    geometry_msgs::msg::Twist joy_cmd_;
    geometry_msgs::msg::Twist localization_cmd_;
    /* ---------- cmd_vle publish ---------- */
    geometry_msgs::msg::Twist out;
    /* ---------- sensor values ---------- */

    float scan_min_front_ = 10.0;
    float scan_min_left45_ = 100.;
    float scan_min_right45_ = 100.;

    float psd_left_ = 1.0;
    float psd_right_ = 1.0;

    bool psd_event_left_flg = false;
    bool psd_event_right_flg = false;

    float ultra_left_ = 1.0;
    float ultra_right_ = 1.0;

    bool ultra_event_left_flg = false;
    bool ultra_event_right_flg = false;

    bool bumper_left_ = false;
    bool bumper_right_ = false;

    bool bumper_prev_left_ = false;
    bool bumper_prev_right_ = false;
    bool bumper_prev_ = false;

    bool bumper_event_left_flg = false;
    bool bumper_event_right_flg = false;

    bool cliff_fl_ = false;
    bool cliff_fc_ = false;
    bool cliff_fr_ = false;
    bool cliff_rl_ = false;
    bool cliff_rr_ = false;

    int cliff_count_ = 0;


    bool wheel_lift_l_ = false;
    bool wheel_lift_r_ = false;

    bool charge_onoff = false;



    rclcpp::Time recovery_start_;
    rclcpp::Time soft_stop_start_;
    rclcpp::Clock::SharedPtr clock_;
    /* ---------- ROS ---------- */

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_lookahead_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr localization_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_right_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_right_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr psd_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr psd_right_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cliff_fl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cliff_fc_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cliff_fr_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cliff_rl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cliff_rr_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wheel_lift_l_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wheel_lift_r_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr human_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charge_onoff_sub_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr dock_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr dock_stat_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr undock_stat_sub_;

    bool human_leg_detected_ = false;

    rclcpp::Time last_nav_time_;
    rclcpp::Time last_joy_time_;
    rclcpp::Time last_lookahead_time_;
    rclcpp::Time last_localization_time_;
    
    double cmd_timeout_ = 0.5;

    uint8_t arbiter_docking_status = 0;
    uint8_t arbiter_undocking_status = 0;

    uint8_t arbiter_docking_status_last_ = 0;
    uint8_t arbiter_undocking_status_last_ = 0;


    /* ---------- callbacks ---------- */

    // void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg){nav_cmd_=*msg;}
    // void joyCallback(const geometry_msgs::msg::Twist::SharedPtr msg){joy_cmd_=*msg;}

    void bumperLeftCallback(const std_msgs::msg::Bool::SharedPtr msg){bumper_left_=msg->data;}
    void bumperRightCallback(const std_msgs::msg::Bool::SharedPtr msg){bumper_right_=msg->data;}

    void ultraLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg){ultra_left_=msg->range;}
    void ultraRightCallback(const sensor_msgs::msg::Range::SharedPtr msg){ultra_right_=msg->range;}

    void psdLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg){psd_left_=msg->range;}
    void psdRightCallback(const sensor_msgs::msg::Range::SharedPtr msg){psd_right_=msg->range;}

    void cliffFLCallback(const std_msgs::msg::Bool::SharedPtr msg){cliff_fl_=msg->data;}
    void cliffFCCallback(const std_msgs::msg::Bool::SharedPtr msg){cliff_fc_=msg->data;}
    void cliffFRCallback(const std_msgs::msg::Bool::SharedPtr msg){cliff_fr_=msg->data;}
    void cliffRLCallback(const std_msgs::msg::Bool::SharedPtr msg){cliff_rl_=msg->data;}
    void cliffRRCallback(const std_msgs::msg::Bool::SharedPtr msg){cliff_rr_=msg->data;}

    void wheelLiftLCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        wheel_lift_l_ = msg->data;
    }

    void wheelLiftRCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        wheel_lift_r_ = msg->data;
    }

    void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        nav_cmd_ = *msg;
        last_nav_time_ = clock_->now();
    }

    void joyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        joy_cmd_ = *msg;
        last_joy_time_ = clock_->now();
    }

    void lookaheadCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        lookahead_cmd_ = *msg;
        last_lookahead_time_ = clock_->now();
    }

    void localizationCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Localization에서 오는 cmd_vel은 Nav2보다 우선순위 높게 처리
        localization_cmd_ = *msg;
        last_localization_time_ = clock_->now();
    }   

    void humanCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        lidar_human_leg_ = *msg;
    }
    /* ---------- lidar front distance ---------- */
    #define yaw_offset  0.27
    #define min_distance 0.26 //26cm 이하는 inf

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 🔧 파라미터 (튜닝 필수)
        float angle45 = M_PI / 4.0;     // 45도
        float width = 0.15;           // ±범위
        float threshold = 0.8;        // 개입 거리 (m)
        float k = 1.2;                // gain

        float Front_Angle = 0. + yaw_offset; // 0.32-0.04995 = 0.27 , 18도 - 2.86도 = 15.14도
        scan_min_front_  = getAvg(msg,  Front_Angle , width);
        if(scan_min_front_ < min_distance) scan_min_front_ = min_distance;

        float Left_Angle =  Front_Angle - angle45;
        scan_min_left45_  = getAvg(msg,  Left_Angle , width);
        if(scan_min_left45_ < min_distance) scan_min_left45_ = min_distance;

        float Right_Angle =  Front_Angle + angle45;
        scan_min_right45_  = getAvg(msg,  Right_Angle , width);
        if(scan_min_right45_ < min_distance) scan_min_right45_ = min_distance;

        //RCLCPP_INFO(this->get_logger(),"Left Min = %f, center = %f,Right_Angle = %f ",scan_min_left45_,scan_min_front_,scan_min_right45_);
        

    }

    float getAvg(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                float angle_center, float angle_width)
    {
        int start = (angle_center - angle_width - scan->angle_min) / scan->angle_increment;
        int end   = (angle_center + angle_width - scan->angle_min) / scan->angle_increment;

        float sum = 0.0;
        int count = 0;
        float min_val = 999.0;

        for (int i = start; i <= end; i++)
        {
            if (i < 0 || i >= (int)scan->ranges.size()) continue;

            float v = scan->ranges[i];
            if (std::isfinite(v))
            {
                sum += v;
                count++;
                if (v < min_val) min_val = v;
            }
        }

        // return count > 0 ? sum / count : std::numeric_limits<float>::infinity();
        return count > 0 ? std::min(sum / count, min_val + 0.1f) : scan->range_max;
    }

    double current_x_, current_y_, current_yaw_;
    bool active = false;
    double start_x = 0.0;
    double start_y = 0.0;
    double start_yaw = 0.0;
       
    double wanted_angle = 0;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // quaternion → yaw 변환
        auto &q = msg->pose.pose.orientation;

        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

        current_yaw_ = atan2(siny, cosy);
        //RCLCPP_INFO(this->get_logger()," current_x_ =%f current_y_ = %f current_yaw_ = %f angular = %f",current_x_,current_y_ ,current_yaw_);
    }

    // 센서 우선순위

    // Wheel lift → 로봇 들림
    // Cliff → 낙하
    // Bumper → 충돌
    // PSD → 초근접
    // Ultrasonic → 근접
    // LiDAR → 일반 장애물
    /* ---------- safety logic ---------- */

    void updateState()
    {

        bool bumper_now = bumper_left_ || bumper_right_; 
        bool bumper_event = bumper_now && !bumper_prev_; 

        bool bumper_event_left_ = bumper_left_ && !bumper_prev_left_; 
        bool bumper_event_right_ = bumper_right_ && !bumper_prev_right_; 

        bumper_prev_left_ = bumper_left_;
        bumper_prev_right_ = bumper_right_;
        bumper_prev_ = bumper_now; /* recovery 중에는 기본 safety 로직 무시 */ 

        //RCLCPP_INFO(this->get_logger(),"psd_L %f psd_R %f Ul %f UR %f",psd_left_,psd_right_,ultra_left_,ultra_right_);
       // RCLCPP_INFO(this->get_logger(),"human_detected = %d ",human_detected_);

        if((charge_onoff == true) || (arbiter_docking_status == 1) || (arbiter_undocking_status == 1))
        {
            state_=SafetyState::NORMAL;
          
            return; //충전 중에는 모든 센서 무시 (충전 스테이션에서 발생하는 센서 이벤트 방지   )
        }

        if(state_ == SafetyState::RECOVERY_BACK || state_ == SafetyState::RECOVERY_TURN) 
            return;

        /* ---------- wheel lift (최우선 안전) ---------- */

        if(wheel_lift_l_ || wheel_lift_r_)
        {
             setState(SafetyState::E_STOP,"Wheel lift","Emergency Stop");
            return;
        }

        /* ---------- cliff front ---------- */

        if(cliff_fl_ || cliff_fc_ || cliff_fr_)
        {
            cliff_count_++;

            if(cliff_count_>2)
            {
                setState(SafetyState::RECOVERY_BACK,"CLIFF_F","RECOVERY_BACK");
                recovery_start_ = clock_->now();
            }
            return;
        }
        else
        {
            cliff_count_=0;
        }

        /* ---------- rear cliff ---------- */

        if(cliff_rl_ || cliff_rr_)
        {
            nav_cmd_.linear.x = std::max(nav_cmd_.linear.x,0.0);
        }

        /* bumper */
        if(bumper_event_left_ || bumper_event_right_){
            if(bumper_event_left_ && bumper_event_right_){
                setState(SafetyState::HARD_STOP,"BUMPER !!","RECOVERY_BACK");
                    bumper_event_left_flg = true;
                    bumper_event_right_flg = true;
                    // active = false;
                    return; 
            }
            else if(bumper_event_left_) 
            { 
                    setState(SafetyState::HARD_STOP,"BUMPER LEFT","RECOVERY_BACK");
                    bumper_event_left_flg = true;
                    // active = false;
                    return; 
            }
            else 
            { 
                    setState(SafetyState::HARD_STOP,"BUMPER RIGHT","RECOVERY_BACK");
                    bumper_event_right_flg = true;
                    // active = false;
                    return; 
            }
        }

        /* PSD */

        if(psd_left_<0.006 || psd_right_<0.006)
        {
            return; //PSD 잠시 정지 2026,05,09
            setState(SafetyState::HARD_STOP,"PSD","HARD_STOP");
            if(psd_left_<0.006) psd_event_left_flg = true;
            else psd_event_right_flg = true;
            return;
        }

        /* ultrasonic */
        #define Ultra_Detact_Range      0.035  //0.17 최저, 0.34 ,0.51
        //if(scan_min_front_<0.018 || ultra_left_<0.012 || ultra_right_<0.012)
        if(ultra_left_ == 0.0) ultra_left_ = 1.0; //장애물 없을 시 0
        if(ultra_right_ == 0.0) ultra_right_ = 1.0; //장애물 없을 시 0
        if(ultra_left_< Ultra_Detact_Range || ultra_right_< Ultra_Detact_Range)
        {
            if(state_ != SafetyState::SOFT_STOP) soft_stop_start_ = clock_->now();
            if((ultra_left_< Ultra_Detact_Range) && (ultra_right_< Ultra_Detact_Range))
            {
                    setState(SafetyState::HARD_STOP,"ULTRA ","HARD_STOP");
                    ultra_event_left_flg = true;
                    ultra_event_right_flg = true;
            }
            else if(ultra_left_< Ultra_Detact_Range) 
            {
                    setState(SafetyState::HARD_STOP,"ULTRA LEFT","HARD_STOP");
                    ultra_event_left_flg = true;
            }
            else
            {    setState(SafetyState::HARD_STOP,"ULTRA RIGHT","HARD_STOP");        
                    ultra_event_right_flg = true;
            }
            return;
        }

        /* lidar human zone */

        // if(scan_min_front_<0.35)
        // {
        //     state_ = SafetyState::CAUTION;
        //     return;
        // }

        state_=SafetyState::NORMAL;
    }

    /* ---------- control loop ---------- */

    void controlLoop()
    {

        updateState();

        

        /* joystick priority */

        // if(joy_cmd_.linear.x!=0.0 || joy_cmd_.angular.z!=0.0)
        //     out = joy_cmd_;
        // else
        //     out = nav_cmd_;

        auto isZero = [](const geometry_msgs::msg::Twist &cmd)
        {
            return std::fabs(cmd.linear.x) < 1e-4 &&
                   std::fabs(cmd.angular.z) < 1e-4;
        };

       
       
        bool joy_recent = (clock_->now() - last_joy_time_).seconds() < cmd_timeout_;
        bool joy_active = joy_recent && !isZero(joy_cmd_);

        bool nav_recent = (clock_->now() - last_nav_time_).seconds() < cmd_timeout_;
        bool nav_active = nav_recent && !isZero(nav_cmd_);
       
        bool lookahead_recent = (clock_->now() - last_lookahead_time_).seconds() < cmd_timeout_;
        bool lookahead_active = lookahead_recent && !isZero(lookahead_cmd_);

        bool localization_recent = (clock_->now() - last_localization_time_).seconds() < cmd_timeout_;
        bool localization_active = localization_recent && !isZero(localization_cmd_);
        
        // -----------------------------
        // 🧠 Nav2 Recovery 판단
        // -----------------------------
        bool is_rotate =
            fabs(nav_cmd_.angular.z) > 0.3 &&
            fabs(nav_cmd_.linear.x) < 0.05;

        bool is_backup =
            nav_cmd_.linear.x < -0.05;

        bool is_recovery = is_rotate || is_backup;

        //if(is_recovery) lookahead_active = 0;
        
        //lookahead_active = true;
        nav_active = false;
// RCLCPP_WARN(get_logger(),
//     "joy_active=%d loc_active=%d joy_recent=%d loc_recent=%d",
//     joy_active, localization_active, joy_recent, localization_recent);

        if(joy_active == false && localization_active == false && lookahead_active == false && nav_active == false)
        {
            state_ = SafetyState::NORMAL; //보통 정지 상태에서는 Safety 정지 2026-04-24
        }

        if(state_ != SafetyState::NORMAL) //arbiter 상황 우선 수행
        {
              state_machine();
        }
        else if (joy_active)
        {
            out = joy_cmd_;
        }
        else if (localization_active)
        {
            out = localization_cmd_;
        }
        else if (lookahead_active)
        {
            out = lookahead_cmd_;
        }
        else if (nav_active)
        {
            out = nav_cmd_;
        }
        else
        {
            out.linear.x = 0.0;
            out.angular.z = 0.0;
        }
        
        if(( out.linear.x > 0.0) || (out.angular.z != 0.0))
        {
            float threshold = 0.30;  // 30cm 이하일 때만
            float threshold_Linear = 0.50; 
            if((scan_min_front_ <= threshold_Linear) || (scan_min_left45_ <= threshold_Linear) ||  (scan_min_right45_ <= threshold_Linear) ){
            // out.linear.x  = 0.5 * out.linear.x  ; 
            if( out.linear.x > 0.10)  out.linear.x = 0.10;
            } 

            float correction = 0.;
            float Kp_gain = 10;
            //양쪽이 장애물 접근시
            
            if((scan_min_left45_ <= threshold) && (scan_min_right45_ <= threshold)){
                float error = scan_min_left45_ - scan_min_right45_;
                correction = error * Kp_gain;
                out.angular.z = correction;
            }
            else {

                if(scan_min_left45_ <= threshold) {
                    correction = threshold - scan_min_left45_;
                    out.angular.z =  -15 * correction;
                }

                if(scan_min_right45_ <= threshold) {
                    correction = threshold - scan_min_right45_;
                    //out.angular.z = 0.6 * out.angular.z + 0.4 * correction;
                    out.angular.z = 15 * correction;
                }

            }
        }      
  
        

        cmd_pub_->publish(out);
    }

    void setState(SafetyState new_state,
                const std::string &reason,
                const std::string &action)
    {
        if(state_ != new_state)
        {
            publish_event(reason, action);
            state_ = new_state;
        }
    }

    void publish_event(const std::string &reason, const std::string &action)
    {
        std_msgs::msg::String msg;

        msg.data = reason + " -> " + action;

        event_pub_->publish(msg);

        RCLCPP_WARN(get_logger(),
                    "SAFETY EVENT: %s -> %s",
                    reason.c_str(),
                    action.c_str());
    }

    void cancelNav2()
    {
        if(!nav_client_->wait_for_action_server(std::chrono::milliseconds(200)))
            return;

        nav_client_->async_cancel_all_goals();

        RCLCPP_WARN(get_logger(),"Nav2 Goal Cancelled (Safety)");
    }

    void state_machine()
    {

        /* state machine */
        //state_ = SafetyState::NORMAL;
        switch(state_)
        {

            case SafetyState::NORMAL:
            break;

            case SafetyState::E_STOP:
                out.linear.x = 0;
                out.angular.z = 0;
            break;

            case SafetyState::CAUTION:
                out.linear.x *=0.3;
            break;

            case SafetyState::SOFT_STOP:
                out.linear.x =0;
                /* 고정 장애물 대응 */ 
                if((clock_->now() - soft_stop_start_).seconds() > 2.0) 
                { 
                    state_ = SafetyState::RECOVERY_TURN; 
                    recovery_start_ = clock_->now(); 
                    active = false;
                }

            break;

            case SafetyState::HARD_STOP:
                //cancelNav2();
                out.linear.x = 0;
                out.angular.z = 0;

                publish_event("COLLISION","BACK");

                recovery_start_ = clock_->now();

                state_ = SafetyState::RECOVERY_BACK;
                active = false;


            break;

            case SafetyState::RECOVERY_BACK:

                if((clock_->now()-recovery_start_).seconds()<10.0)
                {
                    //out.linear.x=-0.1;
                    if (doBackward(0.10))  // 10cm 후진
                    {
                        state_=SafetyState::RECOVERY_TURN;
                        recovery_start_=clock_->now();
                    }
                }
                else
                {
                    state_=SafetyState::RECOVERY_TURN;
                    recovery_start_=clock_->now();
                }

            break;

            case SafetyState::RECOVERY_TURN:

                if((clock_->now()-recovery_start_).seconds()<1.0)
                {

                    wanted_angle = 0;
                    if(cliff_fl_)
                        wanted_angle = 0;

                    else if(cliff_fr_)
                        wanted_angle = 0;   
                    else if(bumper_event_left_flg)
                        // out.angular.z = -0.6;
                        wanted_angle = -40;
                    else if(bumper_event_right_flg)
                        // out.angular.z = 0.6;
                        wanted_angle = 40;
                    else if(psd_event_left_flg)
                        wanted_angle = -20;
                    else if(psd_event_right_flg)
                        wanted_angle = 20;
                    else if(ultra_event_left_flg)             
                        wanted_angle = -20;           
                    else if(ultra_event_right_flg)             
                        wanted_angle = 20;           
                    else
                        wanted_angle = 0;

                    active = false;
                    
                }
                else if((clock_->now()-recovery_start_).seconds()<10.0)
                {
                    if(doRotate(wanted_angle * (M_PI/180.)))
                    {
                        state_=SafetyState::NORMAL;
                        bumper_event_left_flg = false;
                        bumper_event_right_flg = false;
                        psd_event_left_flg = false;
                        psd_event_right_flg = false;
                        ultra_event_left_flg = false;
                        ultra_event_right_flg = false;
                        publish_event("RECOVERY","TURN");
                    }

                }
                else
                {
                    state_=SafetyState::NORMAL;
                        bumper_event_left_flg = false;
                        bumper_event_right_flg = false;
                        psd_event_left_flg = false;
                        psd_event_right_flg = false;
                        ultra_event_left_flg = false;
                        ultra_event_right_flg = false;
                    publish_event("RECOVERY","TURN");

                }

            break;

        }
    }





    bool doBackward(double target_dist)
    {
        


        // 🔥 시작 시 위치 저장
        if (!active)
        {
            start_x = current_x_;
            start_y = current_y_;
            active = true;
        }

        // 🔥 이동 거리 계산 (로봇 기준 후진 거리)
        double dx = current_x_ - start_x;
        double dy = current_y_ - start_y;

        double back_dist =
            cos(current_yaw_) * (start_x - current_x_) +
            sin(current_yaw_) * (start_y - current_y_);

        // 🔥 목표 거리 도달 체크
        if (back_dist < target_dist)
        {
            double remain = target_dist - back_dist;

            // 속도 점점 줄이기
            double speed = -std::min(0.12, remain * 2.5);

            if(speed > -0.05) speed = -0.05;
            out.linear.x = speed;
            out.angular.z = 0.0;
            // RCLCPP_INFO(this->get_logger(),"back_dist = %f target_dist = %f ",back_dist,target_dist);
            return false; // 아직 진행중
        }
        else
        {
            // 종료
            out.linear.x = 0.0;
            out.angular.z = 0.0;

            //active = false;
            return true; // 완료
        }

        
    }
double target_yaw = 0.0;
double kp_ang = 1.5;
double ki_ang = 1.;
double max_angular = 1.2;
double yaw_error_I = 0.;
    bool doRotate(double target_angle)
    {

        
        // 🔥 시작 시 목표 yaw 설정
        if (!active)
        {
            start_yaw = current_yaw_;
            target_yaw = start_yaw + target_angle;

            active = true;
        }

        // 🔥 yaw error 계산 (wrap 처리 필수)
        double yaw_error = target_yaw - current_yaw_;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));  //-π ~ π
        yaw_error_I += yaw_error;
        yaw_error_I  = std::clamp(yaw_error_I, -0.1, 0.1); //0.05 2배
        // 🔥 아직 회전 중
        if (fabs(yaw_error) > 0.05)
        {
             double angular = std::clamp(yaw_error * kp_ang + yaw_error_I * ki_ang,
                                        -max_angular, max_angular);

            
            //CLCPP_INFO(this->get_logger(),"target_yaw = %f current_yaw_ = %f start_yaw = %f target_angle =%f ",target_yaw,current_yaw_ ,start_yaw,target_angle);
            out.linear.x = 0.0;
            out.angular.z = angular;

            return false;
        }
        else
        {
            // 종료
            out.linear.x = 0.0;
            out.angular.z = 0.0;

            //active = false;
            return true;
        }
    }

    void charge_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {

        charge_onoff = msg->data;
 
    }

 
    /* ===== 도킹 상태 ===== */
    void dock_stat_callback(const std_msgs::msg::Int8::SharedPtr msg)    {
        arbiter_docking_status = msg->data;
        arbiter_docking_status_last_ = arbiter_docking_status;
       // RCLCPP_INFO(get_logger(), "Dock Status: %d", docking_status);
    }
    
    void undock_stat_callback(const std_msgs::msg::Int8::SharedPtr msg)    {
        arbiter_undocking_status = msg->data;
        
        arbiter_undocking_status_last_ = arbiter_undocking_status;
        //RCLCPP_INFO(get_logger(), "Undock Status: %d", undocking_status);
    }   
};

/* ---------- main ---------- */

int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<MotionArbiter>());

    rclcpp::shutdown();

    return 0;
}