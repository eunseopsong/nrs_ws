#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  // ROS 2 초기화
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_isaac_integration");
  auto logger = node->get_logger();

  // MoveIt2의 MoveGroupInterface 생성 (여기서는 "panda_arm" 그룹 사용)
  // 실제 로봇 또는 시뮬레이터 설정에 맞게 planning group 이름을 변경하세요.
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "panda_arm");

  // 목표 포즈 설정 (람다 표현식 사용)
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // 목표 포즈로의 경로 계획 (람다를 사용하여 성공 여부와 계획을 함께 반환)
  auto const [success, plan] = [&move_group_interface]{
    MoveGroupInterface::Plan msg;
    bool ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // 계획 성공 시 실행, 실패 시 오류 로그 출력
  if(success) {
    RCLCPP_INFO(logger, "Planning succeeded. Executing plan.");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // ROS 2 종료
  rclcpp::shutdown();
  return 0;
}
