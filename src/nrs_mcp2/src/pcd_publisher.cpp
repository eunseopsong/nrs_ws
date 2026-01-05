#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

// PCL 관련 헤더
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcdPublisher : public rclcpp::Node
{
public:
  PcdPublisher() : Node("pcd_publisher")
  {
    // 1. 파라미터 선언 및 가져오기
    this->declare_parameter<std::string>("pcd_path", "");
    this->declare_parameter<std::string>("topic", "/pcd_cloud");
    this->declare_parameter<std::string>("frame_id", "map");

    std::string pcd_path = this->get_parameter("pcd_path").as_string();
    std::string topic = this->get_parameter("topic").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();

    if (pcd_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "pcd_path param is empty. set pcd_path:=/path/to/file.pcd");
      // 생성자에서 return 1을 할 수 없으므로, 로직 중단 또는 예외 처리를 해야 합니다.
      // 여기서는 퍼블리셔를 생성하지 않고 종료합니다.
      return;
    }

    // 2. PCD 파일 로드
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile(pcd_path, cloud_blob) < 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read PCD: " << pcd_path);
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded PCD: " << pcd_path
                    << "  points: " << cloud_blob.width * cloud_blob.height
                    << "  fields: " << cloud_blob.fields.size());

    // 3. 메시지 변환
    sensor_msgs::msg::PointCloud2 msg;
    pcl_conversions::moveFromPCL(cloud_blob, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = this->now();

    // 4. QoS 설정 (ROS 1의 latch=true와 동일한 효과: Transient Local)
    // 늦게 접속한 Subscriber(RViz 등)에게도 마지막 메시지를 보장합니다.
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();
    qos_profile.reliable();

    // 5. 퍼블리셔 생성 및 데이터 전송
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos_profile);
    
    // 퍼블리셔 생성 직후 publish하면 됨 (Transient Local 덕분에 늦게 켜진 RViz도 받음)
    pub_->publish(msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Published to " << topic << " with frame_id=" << frame_id);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}