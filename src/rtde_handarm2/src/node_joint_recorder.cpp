#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>
#include <string>
#include <vector>

class JointRecorderNode : public rclcpp::Node
{
public:
    JointRecorderNode()
    : Node("joint_recorder_node")
    {
        // 파일 열기 (append 모드)
        file_path_ = "/home/eunseop/nrs_lab2/datasets/joint_recording.txt";
        file_.open(file_path_, std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
        }

        // Subscriber 생성
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_commands", 10,
            std::bind(&JointRecorderNode::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "JointRecorderNode started. Recording to %s", file_path_.c_str());
    }

    ~JointRecorderNode()
    {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!file_.is_open()) return;

        // position 값 확인
        if (msg->position.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Received JointState with less than 6 positions");
            return;
        }

        // 6개의 position 값을 txt 파일에 저장 (한 줄에 q1~q6)
        for (size_t i = 0; i < 6; i++) {
            file_ << msg->position[i];
            if (i < 5) file_ << " ";  // 값 사이에 공백 추가
        }
        file_ << "\n"; // 줄바꿈
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::ofstream file_;
    std::string file_path_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
