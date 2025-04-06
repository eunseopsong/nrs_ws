#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

class ForwardKinematicsNode : public rclcpp::Node
{
public:
  ForwardKinematicsNode()
  : Node("forward_kinematics_node")
  {
    // FK 계산 함수 호출
    computeForwardKinematics();
  }

private:
  // DH 파라미터 구조체
  struct DHParam {
    double a;           // 링크 길이
    double alpha;       // 링크 꼬임각 (라디안)
    double d;           // 링크 오프셋
    double theta_offset; // 관절 오프셋 (라디안)
  };

  // DH 파라미터를 기반으로 4x4 동차 변환 행렬 계산 함수
  Eigen::Matrix4d dhTransformation(double a, double alpha, double d, double theta)
  {
    Eigen::Matrix4d T;
    T << cos(theta),            -sin(theta)*cos(alpha),   sin(theta)*sin(alpha),    a*cos(theta),
         sin(theta),             cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),    a*sin(theta),
         0,                      sin(alpha),              cos(alpha),               d,
         0,                      0,                       0,                        1;
    return T;
  }

  void computeForwardKinematics()
  {
    // 예시: 2-DOF 평면 로봇 (여러분의 로봇에 맞게 수정)
    std::vector<DHParam> dh_params = {
      {1.0, 0.0, 0.0, 0.0}, // 관절 1: a, alpha, d, theta_offset
      {1.0, 0.0, 0.0, 0.0}  // 관절 2
    };

    // 각 관절의 현재 관절각 (라디안 단위)
    // 예: 두 관절 모두 45도 (π/4)
    std::vector<double> joint_angles = { M_PI / 4, M_PI / 4 };

    if(joint_angles.size() != dh_params.size()){
      RCLCPP_ERROR(get_logger(), "관절 개수와 DH 파라미터 개수가 일치하지 않습니다.");
      return;
    }

    // 초기 변환 행렬 (단위 행렬: 기저 좌표계)
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // 각 관절별 변환 행렬을 곱하여 FK 계산
    for (size_t i = 0; i < dh_params.size(); ++i) {
      double theta = joint_angles[i] + dh_params[i].theta_offset;
      Eigen::Matrix4d A = dhTransformation(dh_params[i].a, dh_params[i].alpha, dh_params[i].d, theta);
      T = T * A;
    }

    // 변환 행렬 T에서 위치와 회전(쿼터니언) 추출
    Eigen::Vector3d position = T.block<3,1>(0,3);
    Eigen::Matrix3d rotation = T.block<3,3>(0,0);
    Eigen::Quaterniond quat(rotation);

    // 결과 출력
    RCLCPP_INFO(get_logger(), "Forward Kinematics 변환 행렬:");
    RCLCPP_INFO(get_logger(), "\n[%.3f, %.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f, %.3f]",
                T(0,0), T(0,1), T(0,2), T(0,3),
                T(1,0), T(1,1), T(1,2), T(1,3),
                T(2,0), T(2,1), T(2,2), T(2,3),
                T(3,0), T(3,1), T(3,2), T(3,3));
    RCLCPP_INFO(get_logger(), "엔드 이펙터 위치: [%.3f, %.3f, %.3f]", position.x(), position.y(), position.z());
    RCLCPP_INFO(get_logger(), "엔드 이펙터 자세 (쿼터니언): [%.3f, %.3f, %.3f, %.3f]",
                quat.x(), quat.y(), quat.z(), quat.w());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardKinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
