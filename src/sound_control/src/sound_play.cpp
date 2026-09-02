#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <cstdlib>
#include <string>
#include <memory>
#include <std_msgs/msg/bool.hpp>
class SoundNode : public rclcpp::Node
{
public:
    SoundNode() : Node("sound_node")
    {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sound",
            10,
            std::bind(&SoundNode::sound_callback, this, std::placeholders::_1)
        );

        charge_onoff_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/charge_onoff",
            10,
            std::bind(&SoundNode::charge_callback, this, std::placeholders::_1)
        );  

        
        home_ = std::getenv("HOME");
        device_ = "plughw:0,0";  // 환경에 맞게 수정
    }

private:
    std::string home_;
    std::string device_;
    std::string current_process_;


    void play(const std::string &filename)
    {
        std::string path = home_ + "/sounds/" + filename;

         // 🔊 볼륨 설정 (여기!)
        system("amixer -c 0 sset PCM 60%");

        // 이전 소리 종료
        system("pkill aplay");

        std::string cmd = "aplay -D " + device_ + " " + path + " &";
        system(cmd.c_str());
    }

    void sound_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;

        if (command == "start")
        {
            play("start.wav");
        }
        else if (command == "stop")
        {
            play("stop.wav");
        }
        else if (command == "Going to Dock Pose") // 처음 위치로 이동 합니다.
        {
            play("Move_to_the_initial_position.wav");
        }
        else if (command == "Search for Station") // 충전 스테이션 탐색을 시작 합니다.
        {
            play("Starting_the_search_for_charging_stations.wav");
        }
        else if (command == "Setting complete") //설정 완료
        {
            play("Setting_complete.wav");
        }
        else if (command == "Tiriring~") //설정 실패
        {
            play("Tiriring_Bell.wav");
        }
        
    }
    bool charge_last = false;
    bool charge_Info = false;
    void charge_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {

        charge_Info = msg->data;
        if(charge_Info && !charge_last)
        {
            play("charge.wav");
        }
        else if(!charge_Info && charge_last)
        {
           // play("charge_stop.wav");
        }
        
        charge_last = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charge_onoff_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoundNode>());
    rclcpp::shutdown();
    return 0;
}