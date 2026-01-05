#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <functional>

struct LaunchSpec {
  std::string pkg;
  std::string file;
};

class CentralSupervisor : public rclcpp::Node {
public:
  CentralSupervisor() : Node("central_supervisor"), running_(true) {
    // 관리할 론치 정의 (패키지명, 론치파일명)
    // ROS 2에서는 .launch.py 또는 .launch.xml 확장자를 명시하는 것이 좋습니다.
    specs_["scan"]    = {"nrs_mcp", "multiview_scan.launch.py"}; 
    specs_["teach"]   = {"nrs_solution", "nrs_fsm.launch.py"};
    specs_["path"]    = {"nrs_path", "path_planning.launch.py"};
    specs_["control"] = {"nrs_mcp", "robot_control.launch.py"};

    // 서비스 서버 생성 유틸리티 함수 사용
    create_services();

    // 자식 프로세스 리퍼 스레드 시작
    reaper_ = std::thread(&CentralSupervisor::reaperLoop, this);

    RCLCPP_INFO(this->get_logger(), "[central_supervisor] ready.");
  }

  virtual ~CentralSupervisor() {
    running_ = false;
    if (reaper_.joinable()) reaper_.join();

    // 남은 론치들 안전 종료
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& kv : pids_) {
      safeTerminate(kv.second);
    }
    pids_.clear();
  }

private:
  // ===== 서비스 콜백 (ROS 2 스타일) =====
  // ROS 2 서비스 콜백은 void 반환형이며 request, response를 shared_ptr로 받습니다.
  void handleScanStart(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>) {
    start("scan");
  }
  void handleScanEnd(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
    stop("scan");
  }

  void handleTeachStart(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>) {
    start("teach");
  }
  void handleTeachEnd(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>) {
    stop("teach");
  }

  void handlePathPlanningStart(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                               std::shared_ptr<std_srvs::srv::Empty::Response>) {
    start("path");
  }
  void handlePathPlanningEnd(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>) {
    stop("path");
  }

  void handleRobotControlStart(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                               std::shared_ptr<std_srvs::srv::Empty::Response>) {
    start("control");
  }
  void handleRobotControlEnd(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>) {
    stop("control");
  }

  // ===== 실행 로직 =====
  bool start(const std::string& key) {
    auto it = specs_.find(key);
    if (it == specs_.end()) {
      RCLCPP_ERROR(this->get_logger(), "unknown key: %s", key.c_str());
      return false;
    }

    std::lock_guard<std::mutex> lk(mtx_);

    // 이미 실행 중이면 OK 처리하고 패스
    if (pids_.count(key) && processAlive(pids_[key])) {
      RCLCPP_WARN(this->get_logger(), "[%s] already running with pid %d", key.c_str(), pids_[key]);
      return true;
    }

    const auto& spec = it->second;
    pid_t pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(this->get_logger(), "[%s] fork failed: %s", key.c_str(), strerror(errno));
      return false;
    }

    if (pid == 0) {
      // child process
      setsid(); // 세션 분리

      // ROS 1: execlp("roslaunch", "roslaunch", ...)
      // ROS 2: execlp("ros2", "ros2", "launch", ...)
      // 주의: ROS 2 launch 파일이 .py 인지 .xml 인지 확인하여 specs_에 정확히 기입해야 함.
      execlp("ros2", "ros2", "launch", spec.pkg.c_str(), spec.file.c_str(), (char*)nullptr);
      
      // 실행 실패 시
      _exit(EXIT_FAILURE);
    }

    // parent process
    pids_[key] = pid;
    RCLCPP_INFO(this->get_logger(), "[%s] launched: ros2 launch %s %s (pid=%d)", 
                key.c_str(), spec.pkg.c_str(), spec.file.c_str(), pid);
    return true;
  }

  bool stop(const std::string& key) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!pids_.count(key)) {
      RCLCPP_WARN(this->get_logger(), "[%s] no running process", key.c_str());
      return true;
    }
    pid_t pid = pids_[key];
    bool ok = safeTerminate(pid);
    if (ok) {
      pids_.erase(key);
      RCLCPP_INFO(this->get_logger(), "[%s] terminated", key.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "[%s] terminate failed for pid %d", key.c_str(), pid);
    }
    return ok;
  }

  // ===== 유틸 (기존 로직 유지) =====
  bool processAlive(pid_t pid) {
    if (pid <= 0) return false;
    if (kill(pid, 0) == 0) return true;
    return false;
  }

  bool safeTerminate(pid_t pid) {
    if (pid <= 0) return true;

    // 1) SIGINT (ROS 2 노드도 SIGINT에 종료되도록 설계됨)
    kill(pid, SIGINT);
    if (waitForExit(pid, 3000)) return true;

    // 2) SIGTERM
    kill(pid, SIGTERM);
    if (waitForExit(pid, 2000)) return true;

    // 3) SIGKILL
    kill(pid, SIGKILL);
    if (waitForExit(pid, 1000)) return true;

    return false;
  }

  bool waitForExit(pid_t pid, int timeout_ms) {
    const int step = 50;
    int waited = 0;
    int status = 0;
    while (waited < timeout_ms) {
      pid_t r = waitpid(pid, &status, WNOHANG);
      if (r == pid) return true;
      if (r == -1 && errno == ECHILD) return true; 
      std::this_thread::sleep_for(std::chrono::milliseconds(step));
      waited += step;
    }
    return false;
  }

  void reaperLoop() {
    while (running_) {
      int status = 0;
      pid_t r = waitpid(-1, &status, WNOHANG);
      if (r > 0) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (auto it = pids_.begin(); it != pids_.end(); ) {
          if (it->second == r) {
            RCLCPP_INFO(this->get_logger(), "[reaper] child pid %d exited for key %s", r, it->first.c_str());
            it = pids_.erase(it);
          } else {
            ++it;
          }
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }

  void create_services() {
    using namespace std::placeholders;
    // std::bind를 사용하여 멤버 함수와 this 포인터 바인딩
    srv_scan_start_ = this->create_service<std_srvs::srv::Empty>(
        "/scan_package_start", std::bind(&CentralSupervisor::handleScanStart, this, _1, _2));
    srv_scan_end_ = this->create_service<std_srvs::srv::Empty>(
        "/scan_package_end", std::bind(&CentralSupervisor::handleScanEnd, this, _1, _2));
    
    srv_teach_start_ = this->create_service<std_srvs::srv::Empty>(
        "/teach_package_start", std::bind(&CentralSupervisor::handleTeachStart, this, _1, _2));
    srv_teach_end_ = this->create_service<std_srvs::srv::Empty>(
        "/teach_package_end", std::bind(&CentralSupervisor::handleTeachEnd, this, _1, _2));
    
    srv_path_start_ = this->create_service<std_srvs::srv::Empty>(
        "/path_planning_package_start", std::bind(&CentralSupervisor::handlePathPlanningStart, this, _1, _2));
    srv_path_end_ = this->create_service<std_srvs::srv::Empty>(
        "/path_planning_package_end", std::bind(&CentralSupervisor::handlePathPlanningEnd, this, _1, _2));
    
    srv_control_start_ = this->create_service<std_srvs::srv::Empty>(
        "/robot_control_package_start", std::bind(&CentralSupervisor::handleRobotControlStart, this, _1, _2));
    srv_control_end_ = this->create_service<std_srvs::srv::Empty>(
        "/robot_control_package_end", std::bind(&CentralSupervisor::handleRobotControlEnd, this, _1, _2));
  }

private:
  // ROS 2 Service Server 타입 정의
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_scan_start_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_scan_end_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_teach_start_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_teach_end_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_path_start_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_path_end_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_control_start_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_control_end_;

  std::map<std::string, LaunchSpec> specs_;
  std::map<std::string, pid_t> pids_;
  std::mutex mtx_;

  std::thread reaper_;
  std::atomic<bool> running_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // 노드 생성 및 실행
  auto node = std::make_shared<CentralSupervisor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}