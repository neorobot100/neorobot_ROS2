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


    last_nav_time_ = now();
    last_joy_time_ = now();
    last_lookahead_time_ = now();

    nav_client_ =
    rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
}

private:

/* ---------- state ---------- */

SafetyState state_ = SafetyState::NORMAL;

geometry_msgs::msg::Point lidar_human_leg_;

/* ---------- cmd sources ---------- */
geometry_msgs::msg::Twist lookahead_cmd_;
geometry_msgs::msg::Twist nav_cmd_;
geometry_msgs::msg::Twist joy_cmd_;
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




rclcpp::Time recovery_start_;
rclcpp::Time soft_stop_start_;
/* ---------- ROS ---------- */

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_lookahead_;

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
bool human_leg_detected_ = false;

rclcpp::Time last_nav_time_;
rclcpp::Time last_joy_time_;
rclcpp::Time last_lookahead_time_;

double cmd_timeout_ = 0.5;

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
    last_nav_time_ = now();
}

void joyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    joy_cmd_ = *msg;
    last_joy_time_ = now();
}

void lookaheadCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    lookahead_cmd_ = *msg;
    last_lookahead_time_ = now();
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
                recovery_start_ = now();
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
                    return; 
            }
            else if(bumper_event_left_) 
            { 
                    setState(SafetyState::HARD_STOP,"BUMPER LEFT","RECOVERY_BACK");
                    bumper_event_left_flg = true;
                    return; 
            }
            else 
            { 
                    setState(SafetyState::HARD_STOP,"BUMPER RIGHT","RECOVERY_BACK");
                    bumper_event_right_flg = true;
                    return; 
            }
        }

        /* PSD */

        if(psd_left_<0.006 || psd_right_<0.006)
        {
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
            if(state_ != SafetyState::SOFT_STOP) soft_stop_start_ = now();
            if((ultra_left_< Ultra_Detact_Range) && (ultra_right_< Ultra_Detact_Range))
                    setState(SafetyState::SOFT_STOP,"ULTRA ","SOFT_STOP");
            else if(ultra_left_< Ultra_Detact_Range) 
                    setState(SafetyState::SOFT_STOP,"ULTRA LEFT","SOFT_STOP");
            else    setState(SafetyState::SOFT_STOP,"ULTRA RIGHT","SOFT_STOP");        
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

       
       
        bool joy_recent = (now() - last_joy_time_).seconds() < cmd_timeout_;
        bool joy_active = joy_recent && !isZero(joy_cmd_);

        bool nav_recent = (now() - last_nav_time_).seconds() < cmd_timeout_;
        bool nav_active = nav_recent && !isZero(nav_cmd_);
       
        bool lookahead_recent = (now() - last_lookahead_time_).seconds() < cmd_timeout_;
        bool lookahead_active = lookahead_recent && !isZero(lookahead_cmd_);

        
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
        if (state_ != SafetyState::NORMAL) //arbiter 상황 우선 수행
        {
              state_machine();
        }
        else if (joy_active)
        {
            out = joy_cmd_;
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
        
        if(( out.linear.x != 0.0) || (out.angular.z != 0.0))
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
                if((now() - soft_stop_start_).seconds() > 2.0) 
                { 
                    state_ = SafetyState::RECOVERY_TURN; 
                    recovery_start_ = now(); 
                }

            break;

            case SafetyState::HARD_STOP:
                //cancelNav2();
                out.linear.x = 0;
                out.angular.z = 0;

                publish_event("COLLISION","BACK");

                recovery_start_ = now();

                state_ = SafetyState::RECOVERY_BACK;


            break;

            case SafetyState::RECOVERY_BACK:

                if((now()-recovery_start_).seconds()<1.0)
                {
                    out.linear.x=-0.1;
                }
                else
                {
                    state_=SafetyState::RECOVERY_TURN;
                    recovery_start_=now();
                }

            break;

            case SafetyState::RECOVERY_TURN:

                if((now()-recovery_start_).seconds()<1.0)
                {

                
                if(cliff_fl_)
                    out.angular.z = -0.6;

                else if(cliff_fr_)
                    out.angular.z = 0.6;

                else if(bumper_event_left_flg)
                    out.angular.z = -0.6;
                else if(bumper_event_right_flg)
                    out.angular.z = 0.6;
                else if(psd_event_left_flg)
                    out.angular.z = -0.6;
                else if(psd_event_right_flg)
                    out.angular.z = 0.6;
                else
                    out.angular.z = 0.6;


                }
                else
                {
                    state_=SafetyState::NORMAL;
                    bumper_event_left_flg = false;
                    bumper_event_right_flg = false;
                    publish_event("RECOVERY","TURN");

                }

            break;

        }
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