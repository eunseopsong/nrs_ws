#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp> // .hpp로 변경됨
#include <Eigen/Dense>

// 서비스 헤더 (snake_case로 생성됨)
#include "nrs_mcp2/srv/get_tcp_pose.hpp"

using namespace std::chrono_literals;

class TcpPoseServer : public rclcpp::Node {
public:
  TcpPoseServer()
  : Node("get_tcp_pose_server")
  {
    // ==========================================
    // 1. 파라미터 선언 및 가져오기
    // ==========================================
    this->declare_parameter<std::string>("base_frame", "base");
    this->declare_parameter<std::string>("tool_frame", "tool0");
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<double>("publish_rate", 30.0);
    this->declare_parameter<double>("lookup_timeout", 0.2);

    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("tool_frame", tool_frame_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("lookup_timeout", lookup_timeout_);

    // ==========================================
    // 2. TF Buffer & Listener 초기화
    // ==========================================
    // ROS 2에서 Buffer는 Clock을 필요로 합니다.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ==========================================
    // 3. Publisher & Service 생성
    // ==========================================
    pub_tcp_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tcp_pose", 10);
    pub_fMc_      = this->create_publisher<std_msgs::msg::Float64MultiArray>("fMc_matrix", 10);

    // 서비스 생성
    srv_ = this->create_service<nrs_mcp2::srv::GetTcpPose>(
      "get_tcp_pose",
      std::bind(&TcpPoseServer::handleGetTcpPose, this, std::placeholders::_1, std::placeholders::_2)
    );

    // ==========================================
    // 4. 타이머 설정 (기존 while loop 대체)
    // ==========================================
    // publish_rate(Hz)를 주기로 변환
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      period, std::bind(&TcpPoseServer::publishOnce, this));

    RCLCPP_INFO(this->get_logger(), "get_tcp_pose ready. base=%s, tool=%s, cam=%s",
             base_frame_.c_str(), tool_frame_.c_str(), camera_frame_.c_str());
  }

private:
  // 서비스 콜백 함수 (ROS 2 타입 사용)
  void handleGetTcpPose(const std::shared_ptr<nrs_mcp2::srv::GetTcpPose::Request> /*request*/,
                        std::shared_ptr<nrs_mcp2::srv::GetTcpPose::Response> res)
  {
    geometry_msgs::msg::TransformStamped tf;
    if (lookup(base_frame_, tool_frame_, tf)) {
      res->pose = toPoseStamped(tf);
    } else {
      res->pose.header.stamp = this->now();
      res->pose.header.frame_id = base_frame_;
      RCLCPP_WARN(this->get_logger(), "Service failed to lookup transform");
    }
  }

  // 주기적으로 호출되는 함수
  void publishOnce() {
    // 1) TCP pose publish
    geometry_msgs::msg::TransformStamped tf_bt;
    if (lookup(base_frame_, tool_frame_, tf_bt)) {
      pub_tcp_pose_->publish(toPoseStamped(tf_bt));
    }

    // 2) fMc matrix publish (base -> camera_link)
    geometry_msgs::msg::TransformStamped tf_bc;
    if (lookup(base_frame_, camera_frame_, tf_bc)) {
      // tf2_eigen 변환
      Eigen::Isometry3d Tbc = tf2::transformToEigen(tf_bc);
      std_msgs::msg::Float64MultiArray msg;
      fillMatrixMsgRowMajor(Tbc.matrix(), msg);
      pub_fMc_->publish(msg);
    }
  }

  bool lookup(const std::string& src,
              const std::string& tgt,
              geometry_msgs::msg::TransformStamped& out)
  {
    try {
      // tf2::TimePointZero는 ros::Time(0)과 같습니다 (최신 TF 조회)
      // 타임아웃은 std::chrono::duration으로 변환하여 전달
      out = tf_buffer_->lookupTransform(src, tgt, tf2::TimePointZero,
                                        tf2::durationFromSec(lookup_timeout_));
      return true;
    } catch (const tf2::TransformException& ex) {
      // Throttle 로깅 (ROS 2 스타일)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "TF lookup failed %s -> %s: %s", src.c_str(), tgt.c_str(), ex.what());
      return false;
    }
  }

  static geometry_msgs::msg::PoseStamped toPoseStamped(const geometry_msgs::msg::TransformStamped& t) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = t.header;
    ps.pose.position.x = t.transform.translation.x;
    ps.pose.position.y = t.transform.translation.y;
    ps.pose.position.z = t.transform.translation.z;
    ps.pose.orientation = t.transform.rotation;
    return ps;
  }

  static void fillMatrixMsgRowMajor(const Eigen::Matrix4d& M, std_msgs::msg::Float64MultiArray& out) {
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows";
    out.layout.dim[0].size  = 4;
    out.layout.dim[0].stride= 16;
    out.layout.dim[1].label = "cols";
    out.layout.dim[1].size  = 4;
    out.layout.dim[1].stride= 4;
    out.data.resize(16);
    // row-major flatten
    int k = 0;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        out.data[k++] = M(r, c);
      }
    }
  }

  // 멤버 변수
  std::string base_frame_, tool_frame_, camera_frame_;
  double publish_rate_, lookup_timeout_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tcp_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_fMc_;
  rclcpp::Service<nrs_mcp2::srv::GetTcpPose>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}