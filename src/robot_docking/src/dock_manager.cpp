#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
#include "std_srvs/srv/empty.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <action_msgs/msg/goal_status_array.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

#define use_kidnap_detection       0 // true: AMCL의 위치 불확실성(covariance)을 기반으로 키드냅 감지, false: 감지 않함

class DockControl : public rclcpp::Node
{
public:
    DockControl() : Node("dock_control")
    {

        clock_ = this->get_clock();

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&DockControl::joy_callback, this, std::placeholders::_1));

        amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&DockControl::amcl_pose_callback, this, std::placeholders::_1));

        dock_stat_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/dock_stat", 10,
            std::bind(&DockControl::dock_stat_callback, this, std::placeholders::_1));

        undock_stat_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/undock_stat", 10,
            std::bind(&DockControl::undock_stat_callback, this, std::placeholders::_1));

        charge_onoff_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/charge_onoff",
            10,
            std::bind(&DockControl::charge_callback, this, std::placeholders::_1)
        );     


        dock_cmd_pub_ = create_publisher<std_msgs::msg::Int8>("/dock_cmd", 10);

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        timer_ = create_wall_timer(100ms,
            std::bind(&DockControl::update, this));


        init_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // 2초 후 자동 실행
        init_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&DockControl::init_pose_once, this)
        );

        stop_lidar_client_ = create_client<std_srvs::srv::Empty>("/stop_scan");
        start_lidar_client_ = create_client<std_srvs::srv::Empty>("/start_scan");

        global_loc_client_ = create_client<std_srvs::srv::Empty>(
            "/reinitialize_global_localization");


        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_localization", 10);   

        sound_pub_ = create_publisher<std_msgs::msg::String>("/sound", 10);
            
        nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>("/navigate_to_pose/_action/status", 10,
            std::bind(&DockControl::nav_status_callback,this,std::placeholders::_1));
        

        RCLCPP_INFO(get_logger(), "Dock Control Node Started");
    }

private:
    /* ===== 상태 ===== */
    enum State
    {
        IDLE = 0,
        CANCEL_NAV = 1,
        GO_HOME = 2,
        WAIT_ARRIVAL = 3,
        DOCKING = 4,
        UNDOCKING = 5,
        LOCALIZE = 6,
        READY = 7,
        RECOVERY   = 8
    };

    State state_ = IDLE;

    uint8_t state_last_ = 0;

    rclcpp::Clock::SharedPtr clock_;

    /* ===== ROS ===== */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr dock_stat_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr undock_stat_sub_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr dock_cmd_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    bool init_pose_done_ = false;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_lidar_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_lidar_client_;
    
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_loc_client_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charge_onoff_sub_;

    // ⭐ LOCALIZE용
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sound_pub_;
    
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;

    /* ===== 변수 ===== */
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::PoseStamped dock_pose_;

    bool dock_request_ = false;
    bool undock_request_ = false;
 

    bool dock_pose_saved_ = false;
    bool need_save_dock_pose_ = false;

    uint8_t docking_status = 0;
    uint8_t undocking_status = 0;

    uint8_t docking_status_last_ = 0;
    uint8_t undocking_status_last_ = 0;

    
    double cov_x_ = 999.0; //999.0 : 무조건 “불안정 상태”로 시작
    double cov_y_ = 999.0;
    bool amcl_ready_ = false; // AMCL 동작 시작 확인용
    
    

    bool global_loc_called_ = true; // true: 이미 글로벌 로컬라이제이션 실행한 상태 (키드냅 감지에서는 한 번만 실행하면 됨), false: 아직 실행 안한 상태
    rclcpp::Time localize_start_time_;

    /* ===== JOY ===== */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // BACK 버튼 → DOCK
        if(msg->buttons[6] == 1)
        {
            RCLCPP_WARN(get_logger(), "JOY DOCK");
            dock_request_ = true;
            undock_request_ = false;
            state_ = IDLE;
        }

        // START 버튼 → UNDOCK
        if(msg->buttons[7] == 1)
        {
            RCLCPP_WARN(get_logger(), "JOY UNDOCK");
            undock_request_ = true;
            dock_request_ = false;
            state_ = IDLE;
        }

        if((msg->buttons[10] == 1) && (need_save_dock_pose_ == false))//오른쪽 스틱 누르기
        {
            RCLCPP_WARN(get_logger(), "JOY TIRIRING");
            need_save_dock_pose_ = true;

            if( charge_onoff_state == true )
            {
                save_dock_pose(); // 현재 위치를 도킹 위치로 저장 (JOY 명령으로도 저장할 수 있게)
            }
            else RCLCPP_WARN(get_logger(), "Cannot save dock pose because not charging");
        }

    }

    /* ===== 위치 ===== */
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        init_pose_once(); // AMCL이 준비되면 초기 위치 설정 시도 (한번만 실행)
        // pose.pose : 실제 위치
        current_pose_ = msg->pose.pose;

        //pose.covariance = “위치가 얼마나 정확한지” 나타내는 값 ⭐ , 작을수록 → 위치 확실, 클수록 → 위치 불확실
        // cov_x = 0.01 → 매우 정확 ⭐
        // cov_x = 0.1  → 괜찮음
        // cov_x = 1.0  → 거의 모름 ❌
        cov_x_ = msg->pose.covariance[0];  // x variance
        cov_y_ = msg->pose.covariance[7];  // y variance

        amcl_ready_ = true;


        // x, y, yaw의 분산(Variance) 값 추출
        double var_x = msg->pose.covariance[0];  // x 방향 불확실성
        double var_y = msg->pose.covariance[7];  // y 방향 불확실성
        double var_yaw = msg->pose.covariance[35]; // 회전 방향 불확실성

        RCLCPP_INFO(this->get_logger(), "Covariance -> X: %.4f, Y: %.4f, Yaw: %.4f", 
                    var_x, var_y, var_yaw);

        // 키드냅 감지 로직 예시: 오차가 일정 수준(예: 0.2) 이상이면 경고
        // if (var_x > 0.2 || var_y > 0.2) {
        //     RCLCPP_WARN(this->get_logger(), "위치 불확실성 높음! 키드냅 의심됨.");
        // }


    }

    bool is_localized()
    {
        // return (cov_x_ < 0.1 && cov_y_ < 0.1);
        return (cov_x_ < 0.25 && cov_y_ < 0.25); // 0.1에서 0.25로 완화 (도킹 직후는 불확실성 높음)
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
            else if(undocking_status == 2){ //완료
                
                RCLCPP_WARN(get_logger(), "Undocking Completed");
                state_ = LOCALIZE;
                //need_save_dock_pose_ = true;   // ⭐ 플래그만 설정
            
                // global_loc_called_ = false;
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
        if(state_ != state_last_)
        {
            RCLCPP_INFO(get_logger(), "Dock State changed: %d -> %d", state_last_, state_);
            
        }   
        state_last_ = state_;
        
       

        if(charge_onoff_state == true)
        {
            if(state_ == LOCALIZE) state_ = IDLE;
            // RCLCPP_INFO(get_logger(), "Charging... ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐👉🔥🔥🔥🔥");
           
        }
        else if((state_ != LOCALIZE) && (use_kidnap_detection) && (undock_request_ == false)) // 언도킹 직후는 키드냅 감지 안함 (도킹 위치 저장이 아직 안됐기 때문)
        {
            // covariance 기반
            //if(cov_x_ > 1.0 || cov_y_ > 1.0)
            if(cov_x_ > 0.2 || cov_y_ > 0.2)
            {

                RCLCPP_ERROR(get_logger(), "Kidnapped detected!");

                state_ = LOCALIZE;
                global_loc_called_ = false;
            }
        }

        switch(state_)
        {
            case IDLE:
                if(dock_request_) // JOY Pad 도킹 명령이 들어오면
                {
                    state_ = CANCEL_NAV;
                    // 도킹 시작
                    //stop_lidar();
                }
                else if(undock_request_)  // JOY Pad 언도킹 명령이 들어오면
                {
                    state_ = UNDOCKING;
                    start_lidar();
                }
                break;

            case CANCEL_NAV:
                //cancel_nav(); 우선 주석
               
                state_ = GO_HOME;
                break;

            case GO_HOME:
                if(dock_pose_saved_ == false)
                {
                    state_ = DOCKING;  // 도킹 위치가 저장되어 있지 않으면 바로 도킹 시도 (예: 첫 부팅)
                }
                else{ 
                    send_goal();
                    state_ = WAIT_ARRIVAL;
                }
                break;

            case WAIT_ARRIVAL:
                if(arrived())
                    state_ = DOCKING;
                    //  MCU 도킹 시작
                   // stop_lidar();
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

            case LOCALIZE:

                // 1️⃣ global localization 1회
                if(!global_loc_called_)   //false: 첫부팅, 키드넵, 언도킹
                {
                    cancel_nav(); // Nav2 죽이기

                    start_global_localization();

                    global_loc_called_ = true;
                    localize_start_time_ = clock_->now();

                    RCLCPP_WARN(get_logger(), "Global localization triggered");
                }

                // 2️⃣ 회전 (중요 ⭐)
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = 0;//0.03;
                cmd.angular.z = 0.3;
                cmd_vel_pub_->publish(cmd);

                // 3️⃣ 성공 조건
                if(is_localized())
                {
                    RCLCPP_WARN(get_logger(), "Localization SUCCESS");

                    stop_robot();

                    // if(need_save_dock_pose_ && is_pose_valid(current_pose_))
                    // {
                    //     dock_pose_saved_ = false;
                    //     save_dock_pose();
                    //     need_save_dock_pose_ = false;
                    // }

                    state_ = IDLE;
                }

                // // 4️⃣ 타임아웃 재시도 (핵심 ⭐)
                // if((now() - localize_start_time_).seconds() > 8.0)
                // {
                //     RCLCPP_WARN(get_logger(), "Localization retry");

                //     global_loc_called_ = false; // 다시 실행
                // }

                break;



    
        }
            
    }

    /* ===== Nav2 ===== */
    void send_goal()
    {
       

        if(!is_localized())
        {
            RCLCPP_WARN(get_logger(), "Not localized → skip nav");
            return;
        }

        if (!nav_client_->wait_for_action_server(1s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 not available");
            return;
        }

        load_pose_from_file(); // 파일에서 도킹 위치 불러오기
        
        NavigateToPose::Goal goal;
        goal.pose = dock_pose_;

        nav_client_->async_send_goal(goal);

        RCLCPP_INFO(get_logger(), "Going to Dock Pose");

        goal_home_reached_ = false;
        speak("Going to Dock Pose");
       

    }
    void speak(const std::string &text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        sound_pub_->publish(msg);
    }

    void cancel_nav()
    {
        nav_client_->async_cancel_all_goals();
        RCLCPP_WARN(get_logger(), "Nav2 canceled");
    }

    /* ===== 처음위치 도착 판단 ===== */
    bool arrived()
    {
        double dx = current_pose_.position.x - dock_pose_.pose.position.x;
        double dy = current_pose_.position.y - dock_pose_.pose.position.y;

        double dist = sqrt(dx*dx + dy*dy);

        RCLCPP_INFO(
    get_logger(),
    "Dock dist: %.3f",
    dist);
        if(goal_home_reached_) return dist < 0.4; // 40cm
        else return false;

    }

    /* ===== 위치 저장 ===== */
    void save_dock_pose()
    {
        // if(!dock_pose_saved_)
        // {
        //     dock_pose_.header.frame_id = "map";
        //     dock_pose_.pose = current_pose_;
        //     dock_pose_saved_ = true;

        //     RCLCPP_INFO(get_logger(), "Dock pose saved");
        // }

        save_pose_to_file(current_pose_);
     
    }

    /* ===== UART 명령 ===== */
    void send_dock_cmd()
    {
        std_msgs::msg::Int8 msg;
        msg.data = 2; // DOCK
        dock_cmd_pub_->publish(msg);

        speak("Search for Station");
        RCLCPP_WARN(get_logger(), "🏠 DOCK CMD");
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



    void init_pose_once()
    {

        //RCLCPP_INFO(get_logger(), "Dock state_ %d", state_);
        if(init_pose_done_) return;
        if(amcl_ready_ == false) return; // AMCL이 아직 준비 안됨
        
        // ⭐ AMCL이 이 토픽을 들을 준비가 되었는지 확인 ⭐⭐⭐⭐⭐ (구독자가 없으면 계속 기다림) 2026,04,28
        if (init_pose_pub_->get_subscription_count() == 0) {
            RCLCPP_INFO(get_logger(), "Waiting for AMCL to subscribe to /initialpose...");
            return; // 아직 준비 안됨 (다음 타이머 주기에 다시 시도)
        }

        geometry_msgs::msg::PoseWithCovarianceStamped msg;

        msg.header.frame_id = "map";
        msg.header.stamp = clock_->now();

        // if(dock_pose_saved_)  //진입 안함
        // {
        //     // 예전에 저장해둔 도킹 위치가 있으면 그 위치를 AMCL 초기 위치로 사용한다"
        //     msg.pose.pose = dock_pose_.pose;
        //     RCLCPP_WARN(get_logger(), "Init pose from dock_pose ⭐⭐⭐");
           
        // }
        // else
        // {
        //     msg.pose.pose.position.x = 0.0;
        //     msg.pose.pose.position.y = 0.0;
        //     msg.pose.pose.orientation.w = 1.0;

        //     RCLCPP_WARN(get_logger(), "Init pose DEFAULT (0,0) ⭐⭐⭐");
        // }

        
        //2D pose
        // msg.pose.pose.position.x = 0.0;
        // msg.pose.pose.position.y = 0.0;
        // msg.pose.pose.orientation.w = 1.0;
        load_pose_from_file(); // 파일에서  위치 불러오기

        dock_pose_.pose.position.z = 0.0;
        dock_pose_.pose.orientation.x = 0.0;
        dock_pose_.pose.orientation.y = 0.0;
        msg.pose.pose.position.x = dock_pose_.pose.position.x;
        msg.pose.pose.position.y = dock_pose_.pose.position.y;
        msg.pose.pose.orientation.z = dock_pose_.pose.orientation.z;
        msg.pose.pose.orientation.w = dock_pose_.pose.orientation.w;
        
        // msg.pose.pose.position.x = 0.0;
        // msg.pose.pose.position.y = 0.0;
        // msg.pose.pose.orientation.w = 1.0;

        RCLCPP_WARN(get_logger(), "Init pose DEFAULT (%.2f, %.2f) ⭐⭐⭐", msg.pose.pose.position.x, msg.pose.pose.position.y);
// /initialpose 의 “초기 위치를 얼마나 믿을지” 설정
        // msg.pose.covariance[0] = 0.25;  //X 위치 오차
        // msg.pose.covariance[7] = 0.25;  //Y 위치 오차
        // msg.pose.covariance[35] = 0.1;  //Z 회전 오차
        msg.pose.covariance[0] = 5.25;  //X 위치 오차
        msg.pose.covariance[7] = 5.25;  //Y 위치 오차
        msg.pose.covariance[35] = 0.25;  //Z 회전 오차

        init_pose_pub_->publish(msg);

    //    start_global_localization(); //오차 적용 보다는 글로벌 로컬라이제이션을  실행하는 전략으로 변경 2026-05-07

    //     // ⭐ 핵심: 여러번 보내기
    //     for(int i=0; i<5; i++)
    //     {
    //         init_pose_pub_->publish(msg);
    //         rclcpp::sleep_for(200ms);
    //     }

        init_pose_done_ = true;
        init_timer_->cancel();
    }

    void stop_lidar()
    {
        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        stop_lidar_client_->async_send_request(req);
    }

    void start_lidar()
    {
        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        start_lidar_client_->async_send_request(req);
    }

    void start_global_localization()
    {
        // auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        // global_loc_client_->async_send_request(req);
        if(use_kidnap_detection){
            if (!global_loc_client_->wait_for_service(std::chrono::seconds(2))) 
            {
                RCLCPP_WARN(this->get_logger(), "AMCL Global Localization 서비스가 아직 준비되지 않았습니다.");
                return;
            }
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            global_loc_client_->async_send_request(req);

            RCLCPP_INFO(this->get_logger(), "Global Localization 요청을 보냈습니다.");
        }
         else{
            RCLCPP_INFO(this->get_logger(), "Kidnap detection disabled → skip global localization");
        }
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    bool charge_last = false;
    bool charge_Info = false;
    bool charge_onoff_state = false;
    void charge_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {

        charge_Info = msg->data;
        if(charge_Info && !charge_last)
        {
            charge_onoff_state = true;
        }
        else if(!charge_Info && charge_last)
        {
           charge_onoff_state = false;
        }
        
        charge_last = msg->data;
    }


    void save_pose_to_file(const geometry_msgs::msg::Pose& pose)
    {
        std::ofstream file("/home/neorobot/station_pos.txt");

        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open station_pos.txt");
            RCLCPP_ERROR(get_logger(), strerror(errno));
            speak("Tiriring~");
            return;
        }

        file << pose.position.x << " "
            << pose.position.y << " "
            << pose.orientation.z << " "
            << pose.orientation.w;

        file.close();
        RCLCPP_INFO(get_logger(), "Dock pose saved");
        speak("Setting complete");
    }

    void load_pose_from_file()
    {
        std::ifstream file("/home/neorobot/station_pos.txt");

        if (!file.is_open()) {
            RCLCPP_WARN(get_logger(), "station_pos.txt not found");
            speak("Tiriring~");
            return;
        }

        if (!(file >> dock_pose_.pose.position.x
                >> dock_pose_.pose.position.y
                >> dock_pose_.pose.orientation.z
                >> dock_pose_.pose.orientation.w))
        {
            RCLCPP_ERROR(get_logger(), "Failed to read dock pose");
            file.close();
            return;
        }

        dock_pose_.pose.position.z = 0.0;
        dock_pose_.pose.orientation.x = 0.0;
        dock_pose_.pose.orientation.y = 0.0;

        dock_pose_.header.frame_id = "map";
        dock_pose_.header.stamp = now();

        dock_pose_saved_ = true;

        file.close();

        RCLCPP_INFO(
            get_logger(),
            "Dock pose loaded: x=%.2f y=%.2f",
            dock_pose_.pose.position.x,
            dock_pose_.pose.position.y);
    }

    bool goal_home_reached_ = false;

    void nav_status_callback(
        const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        if(msg->status_list.empty())
            return;

        auto latest_status =
            msg->status_list.back().status;

        switch(latest_status)
        {
            case 1:
                RCLCPP_INFO(get_logger(),
                    "Goal accepted");
                break;

            case 2:
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Navigating...");
                break;

            case 4:
                RCLCPP_INFO(get_logger(),
                    "Goal reached!");

                stop_robot();
                goal_home_reached_ = true;


                break;

            case 5:
                RCLCPP_WARN(get_logger(),
                    "Goal canceled");
                break;

            case 6:
                RCLCPP_ERROR(get_logger(),
                    "Goal aborted");
                break;
            default:
                break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DockControl>());
    rclcpp::shutdown();
    return 0;
}