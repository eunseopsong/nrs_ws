#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <map>
#include <cmath>
#include <thread> 

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

// [변경] 패키지 이름 반영: nrs_mcp2
#include "nrs_mcp2/srv/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPoseServer
{
public:
  // MoveGroupInterface는 Node의 SharedPtr가 필요하므로 생성자에서 받습니다.
  GoToPoseServer(const rclcpp::Node::SharedPtr& node)
      : node_(node)
  {
    // ==========================================
    // 1. 파라미터 선언 및 로드
    // ==========================================
    node_->declare_parameter<std::string>("group_name", "manipulator");
    node_->declare_parameter<std::string>("eef_link", "tool0");
    node_->declare_parameter<std::string>("reference_frame", "base_link");
    
    node_->declare_parameter<double>("goal_position_tolerance", 0.005);
    node_->declare_parameter<double>("goal_orientation_tolerance", 0.05);
    node_->declare_parameter<double>("cartesian_min_fraction", 0.80);

    node_->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
    node_->declare_parameter<int>("num_planning_attempts", 20);
    node_->declare_parameter<double>("planning_time", 5.0);

    node_->declare_parameter<bool>("position_only_ik", false);
    node_->declare_parameter<bool>("use_workspace", false);
    node_->declare_parameter<double>("workspace.min_x", -0.8);
    node_->declare_parameter<double>("workspace.min_y", -0.8);
    node_->declare_parameter<double>("workspace.min_z", 0.0);
    node_->declare_parameter<double>("workspace.max_x", 0.8);
    node_->declare_parameter<double>("workspace.max_y", 0.8);
    node_->declare_parameter<double>("workspace.max_z", 1.4);

    node_->declare_parameter<bool>("go_initial_on_start", true);
    node_->declare_parameter<double>("initial_settle_sec", 0.5);

    node_->get_parameter("group_name", group_name_);
    node_->get_parameter("eef_link", eef_link_);
    node_->get_parameter("reference_frame", ref_frame_);
    node_->get_parameter("goal_position_tolerance", pos_tol_);
    node_->get_parameter("goal_orientation_tolerance", ori_tol_);
    node_->get_parameter("cartesian_min_fraction", min_fraction_);
    node_->get_parameter("planner_id", planner_id_);
    node_->get_parameter("num_planning_attempts", attempts_);
    node_->get_parameter("planning_time", plan_time_);
    node_->get_parameter("position_only_ik", use_pos_only_);
    node_->get_parameter("use_workspace", use_ws_);
    
    // ==========================================
    // 2. TF & MoveGroup 초기화
    // ==========================================
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // MoveGroupInterface 생성 (SharedPtr 사용)
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name_);
    
    move_group_->setEndEffectorLink(eef_link_);
    move_group_->setPoseReferenceFrame(ref_frame_);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    move_group_->setGoalPositionTolerance(pos_tol_);
    move_group_->setGoalOrientationTolerance(ori_tol_);
    move_group_->setPlannerId(planner_id_);
    move_group_->setNumPlanningAttempts(attempts_);
    move_group_->setPlanningTime(plan_time_);

    if (use_ws_) {
      double min_x, min_y, min_z, max_x, max_y, max_z;
      node_->get_parameter("workspace.min_x", min_x);
      node_->get_parameter("workspace.min_y", min_y);
      node_->get_parameter("workspace.min_z", min_z);
      node_->get_parameter("workspace.max_x", max_x);
      node_->get_parameter("workspace.max_y", max_y);
      node_->get_parameter("workspace.max_z", max_z);
      move_group_->setWorkspace(min_x, min_y, min_z, max_x, max_y, max_z);
    }

    if (use_pos_only_) {
      RCLCPP_INFO(node_->get_logger(), "Position-only IK expected.");
    }

    // ==========================================
    // 3. 서비스 생성 (패키지 이름 nrs_mcp2 반영)
    // ==========================================
    service_ = node_->create_service<nrs_mcp2::srv::GoToPose>(
      "/go_to_pose",
      std::bind(&GoToPoseServer::handleService, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO_STREAM(node_->get_logger(), "[nrs_mcp2/go_to_pose] ready. group=" << group_name_);

    // ==========================================
    // 4. 초기 자세 이동 (비동기 처리)
    // ==========================================
    bool go_initial;
    node_->get_parameter("go_initial_on_start", go_initial);
    if (go_initial) {
      // 1초 뒤에 초기화 함수 실행 (타이머 사용)
      initial_pose_timer_ = node_->create_wall_timer(
        1s, std::bind(&GoToPoseServer::moveToInitialPose, this));
    }
  }

private:
  void moveToInitialPose()
  {
    // 타이머는 한 번만 실행하고 취소
    initial_pose_timer_->cancel();

    RCLCPP_INFO(node_->get_logger(), "[go_to_pose_server] Moving to initial joint pose...");

    std::map<std::string, double> initial_pose = {
        {"shoulder_pan_joint", -30.0 * M_PI / 180.0},
        {"shoulder_lift_joint", -55.54 * M_PI / 180.0},
        {"elbow_joint", -107.51 * M_PI / 180.0},
        {"wrist_1_joint", -107.45 * M_PI / 180.0},
        {"wrist_2_joint", 92.18 * M_PI / 180.0},
        {"wrist_3_joint", 20.74 * M_PI / 180.0},
    };

    // 관절 이름이 로봇 모델과 일치해야 함
    bool success = move_group_->setJointValueTarget(initial_pose);
    if (!success) {
       RCLCPP_WARN(node_->get_logger(), "Failed to set joint target (Joint names might be mismatch)");
       return;
    }

    // move() 호출
    auto exec_ret = move_group_->move();
    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Initial move failed (code=%d).", exec_ret.val);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Initial move done.");
    }

    double settle_sec;
    node_->get_parameter("initial_settle_sec", settle_sec);
    if (settle_sec > 0.0)
    {
      // [수정 완료] rclcpp::sleep_for에 올바른 타입 전달
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(settle_sec)));
    }
    move_group_->setStartStateToCurrentState();
  }

  // [변경] 서비스 타입 nrs_mcp2 사용
  void handleService(const std::shared_ptr<nrs_mcp2::srv::GoToPose::Request> req,
                     std::shared_ptr<nrs_mcp2::srv::GoToPose::Response> res)
  {
    try
    {
      RCLCPP_INFO(node_->get_logger(), "==== [GoToPose Request] ====");
      RCLCPP_INFO(node_->get_logger(), "Target Frame: %s", req->target_frame.c_str());

      move_group_->setMaxVelocityScalingFactor(std::clamp(static_cast<double>(req->vel_scale), 0.0, 1.0));
      move_group_->setMaxAccelerationScalingFactor(std::clamp(static_cast<double>(req->acc_scale), 0.0, 1.0));
      move_group_->setPlanningTime(std::max(plan_time_, static_cast<double>(req->allowed_planning_time)));
      move_group_->setStartStateToCurrentState();
      move_group_->clearPathConstraints();

      const std::string planning_frame = move_group_->getPoseReferenceFrame();
      geometry_msgs::msg::PoseStamped target_ps = transformPose(req->pose, req->target_frame, planning_frame);

      if (target_ps.header.frame_id.empty())
      {
        res->success = false;
        res->message = "TF transform failed";
        return;
      }

      if (req->cartesian)
      {
        RCLCPP_INFO(node_->get_logger(), "Planning Cartesian path...");
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(move_group_->getCurrentPose(eef_link_).pose);
        waypoints.push_back(target_ps.pose);

        moveit_msgs::msg::RobotTrajectory traj_msg;
        // MoveIt2 computeCartesianPath returns double (fraction)
        double fraction = move_group_->computeCartesianPath(
            waypoints,
            req->eef_step > 0.0 ? req->eef_step : 0.005,
            req->jump_threshold,
            traj_msg,
            true);

        RCLCPP_INFO(node_->get_logger(), "Cartesian planning done. fraction=%.3f", fraction);

        if (req->plan_only)
        {
          bool ok = (!traj_msg.joint_trajectory.points.empty() && fraction >= min_fraction_);
          res->success = ok;
          res->message = ok ? "planned cartesian" : "planning failed";
          return;
        }
        if (traj_msg.joint_trajectory.points.empty() || fraction < min_fraction_)
        {
          res->success = false;
          res->message = "cartesian planning failed";
          return;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Executing Cartesian path...");
        bool ok = (move_group_->execute(traj_msg) == moveit::core::MoveItErrorCode::SUCCESS);
        res->success = ok;
        res->message = ok ? "executed cartesian" : "execution failed";
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Planning joint-space trajectory...");
        move_group_->setPoseTarget(target_ps, eef_link_);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (req->plan_only)
        {
          move_group_->clearPoseTargets();
          res->success = plan_success;
          res->message = plan_success ? "planned joint trajectory" : "planning failed";
          return;
        }
        
        if (!plan_success)
        {
          move_group_->clearPoseTargets();
          res->success = false;
          res->message = "planning failed";
          return;
        }

        RCLCPP_INFO(node_->get_logger(), "Executing trajectory...");
        bool ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_->stop();
        move_group_->clearPoseTargets();

        res->success = ok;
        res->message = ok ? "executed trajectory" : "execution failed";
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "go_to_pose error: " << e.what());
      res->success = false;
      res->message = std::string("error: ") + e.what();
    }
  }

  geometry_msgs::msg::PoseStamped transformPose(const geometry_msgs::msg::Pose &pose_in,
                                                const std::string &from_frame,
                                                const std::string &to_frame)
  {
    geometry_msgs::msg::PoseStamped in, out;
    // [수정 완료] rclcpp::Time(0) 사용
    in.header.stamp = rclcpp::Time(0); 
    in.header.frame_id = from_frame;
    in.pose = pose_in;

    if (from_frame == to_frame)
      return in;

    try
    {
      geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero, 1s);
      tf2::doTransform(in, out, tf);
      return out;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "TF transform failed " << from_frame << " -> " << to_frame << ": " << e.what());
      out.header.frame_id.clear();
      return out;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  // [변경] 서비스 타입 nrs_mcp2 사용
  rclcpp::Service<nrs_mcp2::srv::GoToPose>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr initial_pose_timer_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string group_name_, eef_link_, ref_frame_;
  double pos_tol_, ori_tol_, min_fraction_;
  std::string planner_id_;
  int attempts_;
  double plan_time_;
  bool use_pos_only_;
  bool use_ws_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // MoveIt2는 NodeOptions의 automatic_declarations를 활용하는 경우가 많습니다.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("moveit_go_to_pose_server", node_options);
  
  // 서버 인스턴스 생성
  auto server = std::make_shared<GoToPoseServer>(node);

  // MoveIt2는 MultiThreadedExecutor 필수 (Planning과 Action Server 통신 병렬 처리)
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}