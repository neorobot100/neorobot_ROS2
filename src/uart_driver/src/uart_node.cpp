#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <vector>
#include <cmath>
#include <cstring>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/range.hpp>

#define USE_IMU     0  // 0: Wheel yaw  1: IMU yaw

/* ================= CRC8 ================= */
uint16_t L_ENC = 0,R_ENC=0;
uint32_t crc_ok_count_ = 0;
uint32_t crc_err_count_ = 0;
bool crc_ok_flag_ = false;

bool odom_initialized = false; //odometry 초기화
int32_t left_offset = 0;
int32_t right_offset = 0;


double yaw_offset_ = 0.0;
bool imu_initialized_ = false; //yaw 초기화

/* ================= CRC8 ================= */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

class UARTNode : public rclcpp::Node
{
public:
    UARTNode() : Node("uart_node")
    {
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&UARTNode::cmdvel_cb, this, std::placeholders::_1));

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&UARTNode::joy_cb, this, std::placeholders::_1));

        open_serial("/dev/ttyAMA0", 115200);

        read_timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&UARTNode::read_serial, this));

        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz 10msec
            std::bind(&UARTNode::publish_odom, this));

        timeout_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&UARTNode::timeout_check, this));

        imu_reset_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&UARTNode::imu_reset_retry, this));

        last_cmdvel_time_ = now();

        temp_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&UARTNode::temp_timer_cb, this));

        bumper_left_pub_  = create_publisher<std_msgs::msg::Bool>("/bumper_left", 10);
        bumper_right_pub_ = create_publisher<std_msgs::msg::Bool>("/bumper_right", 10);
        bottom_front_center_pub_ = create_publisher<std_msgs::msg::Bool>("/cliff_fc", 10);
        bottom_front_left_pub_   = create_publisher<std_msgs::msg::Bool>("/cliff_fl", 10);
        bottom_front_right_pub_  = create_publisher<std_msgs::msg::Bool>("/cliff_fr", 10);
        bottom_rear_left_pub_    = create_publisher<std_msgs::msg::Bool>("/cliff_rl", 10);
        bottom_rearight_pub_     = create_publisher<std_msgs::msg::Bool>("/cliff_rr", 10);
        
        wheel_lift_left_pub_    = create_publisher<std_msgs::msg::Bool>("/wheel_lift_l", 10);
        wheel_lift_right_pub_     = create_publisher<std_msgs::msg::Bool>("/wheel_lift_r", 10);


        ultra_left_pub_  = create_publisher<sensor_msgs::msg::Range>("/ultra_left", 10);
        ultra_right_pub_ = create_publisher<sensor_msgs::msg::Range>("/ultra_right", 10);    

        psd_left_pub_  = create_publisher<sensor_msgs::msg::Range>("psd_left", 10);
        psd_right_pub_ = create_publisher<sensor_msgs::msg::Range>("psd_right", 10);

        RCLCPP_INFO(get_logger(), "UART Integrated Node Started");
    }

private:
    /* ================= Members ================= */
    int fd_{-1};
    std::vector<uint8_t> rx_buf_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::TimerBase::SharedPtr read_timer_, odom_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_, imu_reset_timer_;

    rclcpp::TimerBase::SharedPtr temp_timer_; //라즈베리 온도 체크
    rclcpp::Time last_cmdvel_time_;           //속도 명령 시간 체크 


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_right_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_front_center_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_front_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_front_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_rear_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_rearight_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheel_lift_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheel_lift_right_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultra_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultra_right_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr psd_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr psd_right_pub_;

    bool imu_ack_{false};
    int imu_retry_{0};

    bool estop_{true};
    

    //double x_{0}, y_{0}, yaw_{0};
    //int16_t last_left_{0}, last_right_{0};
    bool odom_init_{false};

    const double wheel_r_ = 0.034;
    //const double wheel_base_ = 0.242;
    //const double ticks_rev_ = 16.0 * 55.11;

    bool estop_active_{false};   // 기본은 정지 상태
    bool stopped_by_timeout_{false}; //일정시간 cmd_vel 없을때 타임아웃용도

    //void timeout_check(); //???
    //void send_zero_velocity();

    
        /* ---------- Odometry ---------- */
     void update_odom(int16_t left, int16_t right);
     void publish_odom();
     void publish_sensors(bool bl, bool br,
                     bool fc, bool fl, bool fr, bool rl, bool rr, bool w_lift_l, bool w_lift_r,
                     double ul, double ur,
                     double psdl, double psfr);
    rclcpp::Time last_scan_time_;

    rclcpp::Time last_time_;

    rclcpp::Time last_update_time_;

    double last_yaw_ = 0.0;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool first_odom_ = true;

    int16_t last_left_  = 0;
    int16_t last_right_ = 0;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    uint8_t Battery_Percent = 0;
    uint8_t Debug_temp8 = 0;
    double Debug_temp_Double = 0;


    /* 로봇 파라미터 (HW 기준) */
    const double wheel_radius_ = 0.034;        // m  (6.8cm / 2)
    //const double wheel_base_   = 0.252;        // m  (바퀴 간 거리)
    const double wheel_base_   = 0.242;        // m  (바퀴 간 거리)
    const double ticks_per_rev_ = 16.0 * 55.11; // ≈ 881.76     Gear :55.11
    /*
    바퀴 1회전당 엔코더 tick
    ticks_per_wheel_rev = Enc_resolution × Gear
                        = 16 × 55.11
                        = 881.76 ticks
    */

    /* ================= Serial ================= */
    void open_serial(const char *dev, int baud)
    {
        fd_ = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
             RCLCPP_ERROR(this->get_logger(),
                         "Failed to open %s : %s",
                         dev, strerror(errno));
            return;
        }

        struct termios tio{};
        tcgetattr(fd_, &tio);
        cfmakeraw(&tio);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;
        tcsetattr(fd_, TCSANOW, &tio);

        RCLCPP_INFO(this->get_logger(),
            "Serial opened (nonblock raw): %s", dev);

        rclcpp::sleep_for(std::chrono::milliseconds(100));
        send_imu_reset();
    }

    /* ================= IMU RESET ================= */
    void send_imu_reset()
    {
        uint8_t tx[6];
        uint8_t idx = 0;

        tx[idx++] = 0xAA;
        tx[idx++] = 0x55;
        tx[idx++] = 2;
        tx[idx++] = 0x30;  // IMU RESET CMD
        tx[idx++] = 0x01;

        tx[idx] = crc8(tx, idx);
        idx++;

        write(fd_, tx, idx);
        RCLCPP_INFO(get_logger(), "IMU RESET TX");
    }

    void imu_reset_retry()
    {
        if (imu_ack_) {
            imu_reset_timer_->cancel();
            RCLCPP_INFO(get_logger(), "IMU RESET OK");
            return;
        }

        if (imu_retry_ >= 5) {
            imu_reset_timer_->cancel();
            RCLCPP_ERROR(get_logger(), "IMU RESET FAIL");
            return;
        }

        if( imu_retry_ < 10){
            send_imu_reset();
            imu_retry_++;
        }
        else  imu_reset_timer_->cancel();
    }

    /* ================= RX ================= */
     /* ---------- UART Read ---------- */
    void read_serial()
    {
        fd_set rfds;
        struct timeval tv;

        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        tv.tv_sec  = 0;
        tv.tv_usec = 0;

        int ret = select(fd_ + 1, &rfds, NULL, NULL, &tv);
        if (ret <= 0)
            return;
        if (fd_ < 0)
            return;

        if (FD_ISSET(fd_, &rfds)) {
            uint8_t buf[256];
            int n = read(fd_, buf, sizeof(buf));
            if (n > 0) {
                rx_buf_.insert(rx_buf_.end(), buf, buf + n);
                parse_rx();
            }
        }
    }
    void parse_rx()
    {
        while (rx_buf_.size() >= 4) {
            if (rx_buf_[0] != 0xAA || rx_buf_[1] != 0x55) {
                rx_buf_.erase(rx_buf_.begin());
                continue;
            }

            uint8_t len = rx_buf_[2];
            size_t frame_len = 3 + len;

            if (rx_buf_.size() < frame_len)
                return;

            uint8_t crc_rx = rx_buf_[frame_len - 1];
            uint8_t crc_cal = crc8(rx_buf_.data(), frame_len - 1);

            if (crc_rx == crc_cal) {
                crc_ok_count_++;
                crc_ok_flag_ = true;
                uint8_t cmd = rx_buf_[3];

                if (cmd == 0x31) {  // IMU RESET ACK
                    imu_ack_ = true;
                }

                if (cmd == 0x11) {  // Odom PACKET
                   
                    handle_Odom(&rx_buf_[0]);
                }
            }
            else {
                crc_err_count_++;
            }


            rx_buf_.erase(rx_buf_.begin(),
                          rx_buf_.begin() + frame_len);
        }
    }

    /* ================= Sensor Handling ================= */
    void handle_Odom(const uint8_t *pkt)
    {
        int idx = 4;
        uint8_t FW_index = pkt[idx++];
        int16_t left  = (pkt[idx++] << 8) | pkt[idx++];
        int16_t right = (pkt[idx++] << 8) | pkt[idx++];

        update_odom(left, right);   // ★ 추가

         /* ---------- 센서 bitfield ---------- */
       
        idx += 11; // imu data

        uint8_t sensor_bits = pkt[idx++];
        uint8_t sensor_bits2 = pkt[idx++];
        
        //Debug_temp8 = sensor_bits;
        bool bumper_left  = sensor_bits & (1 << 0);
        bool bumper_right = sensor_bits & (1 << 1);

        bool floor_fc = sensor_bits & (1 << 2);
        bool floor_fl = sensor_bits & (1 << 3);
        bool floor_fr = sensor_bits & (1 << 4);
        bool floor_rl = sensor_bits & (1 << 5);
        bool floor_rr = sensor_bits & (1 << 6);

        bool wheel_lift_l  = sensor_bits2 & (1 << 0);
        bool wheel_lift_r  = sensor_bits2 & (1 << 1);
        /* ---------- 배터리 ---------- */
        Battery_Percent = pkt[idx++];
        uint16_t Battery_Volt_raw  = (pkt[idx++] << 8) | pkt[idx++];
        /* ---------- 초음파 ---------- */
        uint16_t ultra_left_raw  = (pkt[idx++] << 8) | pkt[idx++];
        uint16_t ultra_right_raw = (pkt[idx++] << 8) | pkt[idx++];

        double ultra_left_m  = ultra_left_raw  * 0.001;
        double ultra_right_m = ultra_right_raw * 0.001;
//Debug_temp_Double = ultra_left_m;
         /* ---------- PSD ---------- */
        uint16_t PSD_left_raw  = (pkt[idx++] << 8) | pkt[idx++];
        uint16_t PSD_right_raw = (pkt[idx++] << 8) | pkt[idx++];

        double PSD_left_m  = PSD_left_raw    * 0.00001;
        double PSD_right_m = PSD_right_raw   * 0.00001;

        floor_fc = floor_fl = floor_fr  = floor_rl = floor_rr = wheel_lift_l = wheel_lift_r = 0; //바닥은 나중에 처리

        publish_sensors(bumper_left, bumper_right,
                        floor_fc, floor_fl, floor_fr, floor_rl, floor_rr, wheel_lift_l, wheel_lift_r,
                        ultra_left_m, ultra_right_m,
                        PSD_left_m,PSD_right_m);

        publish_imu(pkt);

        
    }

    void publish_imu(const uint8_t *pkt)
    {
        int idx = 4;

        idx += 1;               // fw index
        idx += 4;               // skip encoders
        idx += 1;               // imu index

        int16_t angle = (pkt[idx] << 8) | pkt[idx+1]; idx += 2;
        int16_t rate  = (pkt[idx] << 8) | pkt[idx+1]; idx += 2;

        int16_t xacc  = (pkt[idx] << 8) | pkt[idx+1]; idx += 2;
        int16_t yacc  = (pkt[idx] << 8) | pkt[idx+1]; idx += 2;
        int16_t zacc  = (pkt[idx] << 8) | pkt[idx+1]; idx += 2;

        
        sensor_msgs::msg::Imu imu;
        // imu.header.stamp = this->now();
        imu.header.stamp = this->get_clock()->now();

        imu.header.frame_id = "imu_link";

        /* yaw만 사용 (rad 변환 필요 시 여기서) */
        //double yaw = ((angle * -0.001)* M_PI) / 180.0;   // 예: raw → rad (튜닝 포인트)
        double raw_yaw  = (angle / -100.0) * M_PI / 180.0;
        Debug_temp_Double = raw_yaw;

        while (raw_yaw  > M_PI)  raw_yaw  -= 2*M_PI;
        while (raw_yaw  < -M_PI) raw_yaw  += 2*M_PI;

         /* ---------- IMU 초기화 ---------- */
        if (!imu_initialized_)
        {
            yaw_offset_ = raw_yaw;
            imu_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU yaw initialized");
        }
        double yaw = raw_yaw - yaw_offset_;

        while (yaw > M_PI)  yaw -= 2*M_PI;
        while (yaw < -M_PI) yaw += 2*M_PI;
        
        if(USE_IMU == 1) yaw_ = yaw;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);

        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();

                
        /* ⭐ 여기 추가 ⭐ */
        imu.orientation_covariance[0] = 1e6;   // roll 신뢰 안 함
        imu.orientation_covariance[4] = 1e6;   // pitch 신뢰 안 함
        imu.orientation_covariance[8] = 0.05;  // yaw 신뢰도 (튜닝 포인트)

        imu.angular_velocity.z = rate * 0.001;
        imu.linear_acceleration.x = xacc * 0.001;
        imu.linear_acceleration.y = yacc * 0.001;
        imu.linear_acceleration.z = zacc * 0.001;

        imu_pub_->publish(imu);
       // RCLCPP_INFO(this->get_logger(), "publish_imu() called");
    }

    

    /* ================= TX ================= */
    void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        //if (!imu_ack_) return;

        if (fd_ < 0) return;

        last_cmdvel_time_ = this->now();

        stopped_by_timeout_ = false;

        float v = msg->linear.x;
        float w = msg->angular.z;
        
        // E-STOP 적용
        uint8_t e_stop = estop_active_ ? 1 : 0;
        if (estop_active_)
        {
            v = 0.0f;
            w = 0.0f;
        }

        int16_t v_i = (int16_t)(v * 1000.0f);
        int16_t w_i = (int16_t)(w * 1000.0f);

        uint8_t tx[15];
        uint8_t idx = 0;

        tx[idx++] = 0xAA;
        tx[idx++] = 0x55;
        tx[idx++] = 7;        // LEN
        tx[idx++] = 0x10;     // CMD_VEL

        tx[idx++] = (v_i >> 8) & 0xFF;
        tx[idx++] = v_i & 0xFF;

        tx[idx++] = (w_i >> 8) & 0xFF;
        tx[idx++] = w_i & 0xFF;

        
        tx[idx++] = e_stop & 0xFF;  // E-STOP 상태

        tx[idx] = crc8(tx, idx);
        idx++;

        write(fd_, tx, idx);

        RCLCPP_DEBUG(this->get_logger(),
        //RCLCPP_INFO(this->get_logger(),
            "TX cmd_vel v=%d w=%d", v_i, w_i);
    }

    void temp_timer_cb()
    {
        if (fd_ < 0) return;

        // 마지막 cmd_vel 이후 1초 이상 경과 시 온도 전송
        if ((this->now() - last_cmdvel_time_).seconds() < 1.0)
            return;

        float temp = get_cpu_temp();
       // temp = 70.; //임시로 로봇 펜 계속돌게
        int16_t t_i = (int16_t)(temp * 100.0f);

        uint8_t tx[7];
        uint8_t idx = 0;

        tx[idx++] = 0xAA;
        tx[idx++] = 0x55;
        tx[idx++] = 4;
        tx[idx++] = 0x20;

        tx[idx++] = (t_i >> 8) & 0xFF;
        tx[idx++] = t_i & 0xFF;

        tx[idx] = crc8(tx, idx);
        idx++;

        write(fd_, tx, idx);

        RCLCPP_DEBUG(this->get_logger(),
            "TX TEMP %.2f C", temp);
    }

    float get_cpu_temp()
    {
        FILE *fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
        if (!fp) return 0.0f;

        int temp;
        fscanf(fp, "%d", &temp);
        fclose(fp);

        return temp / 1000.0f;
    }

   // Joy 콜백 (RT 버튼 → E-STOP)
    void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // F710 기준: buttons[5] 또는 axes[5]
        // 버튼형 기준 (teleop 설정과 동일)
       if (msg->axes.size() > 5)
        {


            if (msg->axes.size() > 5)
                {
                    float rt = msg->axes[5];

                    // 절반 이상 눌렸을 때만 해제
                    if (rt > 0.5f)
                        estop_active_ = false;
                    else
                        estop_active_ = true;
                }
        }
    }

    void timeout_check()
    {
        if (fd_ < 0) return;

        if ((this->now() - last_cmdvel_time_).seconds() > 0.5)
        {
            if (!stopped_by_timeout_)
            {
                send_zero_velocity();
                stopped_by_timeout_ = true;
            }
        }
    }

    
    void send_zero_velocity()
    {
        int16_t v_i = 0;
        int16_t w_i = 0;

        uint8_t tx[15];
        uint8_t idx = 0;

        tx[idx++] = 0xAA;
        tx[idx++] = 0x55;
        tx[idx++] = 7;
        tx[idx++] = 0x10;  // CMD_VEL

        tx[idx++] = (v_i >> 8) & 0xFF;
        tx[idx++] = v_i & 0xFF;

        tx[idx++] = (w_i >> 8) & 0xFF;
        tx[idx++] = w_i & 0xFF;

        tx[idx++] = 1;   // E-STOP 유지

        tx[idx] = crc8(tx, idx);
        idx++;

        write(fd_, tx, idx);

        RCLCPP_WARN(this->get_logger(), "CMD_VEL TIMEOUT → STOP");
    }

};


void UARTNode::update_odom(int16_t left, int16_t right)
{
     if (!imu_ack_) return; //  IMU 정상일시만 오돔 갱신됨

    last_update_time_ = this->now();

        /* ---------- 초기화 ---------- */
    if (!odom_initialized)
    {
        last_left_  = left;
        last_right_ = right;

        x_ = 0.0;
        y_ = 0.0;

        odom_initialized = true;
        RCLCPP_INFO(this->get_logger(), "ODOM initialized");
        return;
    }
    // 엔코더 변화량
    int16_t dL = left  - last_left_;
    int16_t dR = right - last_right_;
    

    last_left_  = left;
    last_right_ = right;

    // tick → meter
    double dl = -(double)dL / ticks_per_rev_
            * (2.0 * M_PI * wheel_radius_);
    double dr = (double)dR / ticks_per_rev_
            * (2.0 * M_PI * wheel_radius_);

    // 회전량 계산 (바퀴 기반)
    double dyaw = (dr - dl) / wheel_base_;
    if(USE_IMU == 0) yaw_ += dyaw;

    // 전진 거리
    double ds = 0.5 * (dl + dr);

    // IMU yaw 사용 (권장)

    //단순 오일러 적분
    // x_ += ds * std::cos(yaw_);
    // y_ += ds * std::sin(yaw_);

    //midpoint integration 방식 ,회전하면서 이동할 때 오차가 줄어듬
    x_ += ds * std::cos(yaw_ + dyaw * 0.5);
    y_ += ds * std::sin(yaw_ + dyaw * 0.5);

    //  publish_odom();
}
/* ================= ODOM ================= */
void UARTNode::publish_odom()
{
        auto now = this->get_clock()->now();

    nav_msgs::msg::Odometry odom;
               
    if (first_odom_) {
        last_time_ = now;
        last_x_ = x_;
        last_y_ = y_;
        last_yaw_ = yaw_;
        first_odom_ = false;
    }
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0001) dt = 0.0001;


            // 이동량 계산
    double dx = x_ - last_x_;
    double dy = y_ - last_y_;
    double dyaw = yaw_ - last_yaw_;

    while (dyaw > M_PI)  dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    // 선속도
    double linear = std::sqrt(dx*dx + dy*dy) / dt;

    // 각속도
    double angular = dyaw / dt;

    last_x_ = x_;
    last_y_ = y_;
    last_yaw_ = yaw_;
    last_time_ = now;

    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    // odom.child_frame_id = "base_link";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0,0,yaw_);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear;
    odom.twist.twist.angular.z = angular;

    odom_pub_->publish(odom);


    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    // tf.child_frame_id = "base_link";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);

      //CRC 성공 시만 로그 출력
    // if (crc_ok_flag_) {
    //     crc_ok_flag_ = false;

    //     uint32_t total = crc_ok_count_ + crc_err_count_;
    //     double err_rate = 0.0;

    //     if (total > 0)
    //         err_rate = (double)crc_err_count_ * 100.0 / total;

    //     RCLCPP_INFO(this->get_logger(),
    //             "Liner x=%.3f Angular=%.3f Bat=%3d%% DIS | CRC OK:%u ERR:%u (%.2f%%) Sensor %8b",
                
    //              Debug_temp_Double,angular, 
    //             //linear, angular,
    //             Battery_Percent,
    //             crc_ok_count_, crc_err_count_,
    //             err_rate,Debug_temp8);
    // }
}

void UARTNode::publish_sensors(bool bl, bool br,
                               bool fc, bool fl, bool fr, bool rl, bool rr, bool w_lift_l, bool w_lift_r,
                               double ul, double ur,  
                               double psdl, double psdr)
{
    auto now = this->get_clock()->now();

    std_msgs::msg::Bool msg;

    msg.data = bl;
    bumper_left_pub_->publish(msg);

    msg.data = br;
    bumper_right_pub_->publish(msg);

    /* 바닥 센서들 */
    
    msg.data = fc;
    bottom_front_center_pub_->publish(msg);

    msg.data = fl;
    bottom_front_left_pub_->publish(msg);

    msg.data = fr;
    bottom_front_right_pub_->publish(msg);

    msg.data = rl;
    bottom_rear_left_pub_->publish(msg);

    msg.data = rr;
    bottom_rearight_pub_->publish(msg);

    msg.data = w_lift_l;
    wheel_lift_left_pub_->publish(msg);

    msg.data = w_lift_r;
    wheel_lift_right_pub_->publish(msg);

    /* 초음파 */
    sensor_msgs::msg::Range rmsg;
    rmsg.header.stamp = now;
    rmsg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    rmsg.field_of_view = 0.5;
    rmsg.min_range = 0.02;
    rmsg.max_range = 4.0;

    rmsg.header.frame_id = "ultra_left_link";
    rmsg.range = ul;
    ultra_left_pub_->publish(rmsg);

    rmsg.header.frame_id = "ultra_right_link";
    rmsg.range = ur;
    ultra_right_pub_->publish(rmsg);

    /* PSD 센서 */
    sensor_msgs::msg::Range psdmsg;

    psdmsg.header.stamp = now;
    psdmsg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    psdmsg.field_of_view = 0.1;
    psdmsg.min_range = 0.05;
    psdmsg.max_range = 0.8;

    psdmsg.header.frame_id = "psd_left_link";
    psdmsg.range = psdl;
    psd_left_pub_->publish(psdmsg);

    psdmsg.header.frame_id = "psd_front_right_link";
    psdmsg.range = psdr;
    psd_right_pub_->publish(psdmsg);
}

/* ================= main ================= */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTNode>());
    rclcpp::shutdown();
    return 0;
}


// AX900UA 라즈베리5 설치 
// cd ~
// git clone https://github.com/biglinux/RTL8851bu.git
// cd RTL8851bu
// make ARCH=arm64 -j4
// sudo make install
// sudo modprobe 8851bu