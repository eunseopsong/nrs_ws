#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <sstream>

#include "Y2FT_AQ/FT_EtherGet.hpp"
#include "Y2FT_AQ/FT_eCANGet.hpp"
#include "Y2Filters/Gen_filter.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "Y2Matrix/YMatrix.hpp"

// =============================
// Sensor / Robot 설정
// =============================

/* Sensor Communication Type */
#define USE_ECAN 1 // 1: eCAN, 0: Ethernet 사용

/* Gravity Compensation Mode */
#define GRAVITY_COMPENSATION_MODE 1 // 0: Disabled, 1: Enabled

/* Sensor IP/PORT */
static const char* FT1_IP   = "192.168.0.44";
static const int   FT1_PORT = 4001;  // 4001: eCAN, 8890: Ethernet

/* Sampling frequency */
static const int   SAMPLING_FREQ_HZ = 2000;

/* Sensor 초기화를 위한 샘플링 개수 */
static const unsigned int SENSOR_INIT_COUNT_NUM = 1000;

/* MOV 필터 크기 */
static const int MOV_SIZE = 10/5;

/* Robot Namespace */
static const char* ROBOT_NAME = "/ur10skku";

/* FT local frame -> TCP local frame (FT 기준 벡터를 TCP 기준으로) */
YMatrix ROT_TCP2FT =
{
    { 1,  0,  0},
    { 0, -1,  0},
    { 0,  0, -1}
};

/* === Gravity Compensation Parameters (from G_Comp_Matrix.py) ===
 *
 *  - TOOL_MASS: 툴 질량 [kg]
 *  - TOOL_COG : FT(센서) 기준 툴 CoM 위치 [m], 3x1 column vector
 *
 *  G_Comp_Matrix.py 결과를 보고 이 값을 채워 넣으면 됨.
 */
static double TOOL_MASS = 1.6;  // 예: 1.632653 등

YMatrix TOOL_COG =
{
    { 0.0 },     // r_x
    { 0.0 },     // r_y
    { -0.149303 } // r_z (예시)
};


// =============================
// 유틸 함수
// =============================

// 3x1 벡터 간의 외적: c = a x b (결과를 result에 저장, 불필요한 임시 객체 생성 제거)
inline void cross3(const YMatrix& a, const YMatrix& b, YMatrix& result)
{
    result[0][0] = a[1][0]*b[2][0] - a[2][0]*b[1][0];
    result[1][0] = a[2][0]*b[0][0] - a[0][0]*b[2][0];
    result[2][0] = a[0][0]*b[1][0] - a[1][0]*b[0][0];
}


// ============================================
// FTGetMain Node 정의
// ============================================
class FTGetMain : public rclcpp::Node
{
public:
    FTGetMain(const std::string& ft_ip,
              int ft_port,
              const std::string& robot_name,
              int mov_size = 1);
    ~FTGetMain();

    /// 타이머 콜백: 센서 읽고, 필터 + 중력 보상 + 좌표 변환 + publish
    void transferData();

private:
    // ----- Parameters -----
    std::vector<Y_MovFilter> movFilter;         // MOV filter for each axis
    std::vector<double>      current_TCP_pose;  // [x,y,z,wx,wy,wz]
    YMatrix                  ROT_Base2TCP;      // Base → TCP 회전 행렬

    // 마지막 G-comp 결과 저장용 (Base frame)         
    YMatrix                  last_GcompForce;   // 3x1
    YMatrix                  last_GcompMoment;  // 3x1

    // Reusable matrices to avoid per-loop allocations
    YMatrix                  SframeForce_;
    YMatrix                  SframeMoment_;
    YMatrix                  SGravityForce_;
    YMatrix                  SGravityMoment_;
    YMatrix                  ScompForce_;
    YMatrix                  ScompMoment_;
    YMatrix                  TCPForce_;
    YMatrix                  TCPMoment_;
    YMatrix                  RframeForce_;
    YMatrix                  RframeMoment_;
    YMatrix                  GcompForce_;
    YMatrix                  GcompMoment_;
    YMatrix                  cross_temp_;  // Temporary for cross3 function

    // Gravity compensation temporaries
    YMatrix                  g_base_;        // base frame gravity
    YMatrix                  g_tcp_;         // tcp frame gravity
    YMatrix                  ROT_TCP2FT_T_;  // TCP->FT
    YMatrix                  g_ft_;          // FT frame gravity

    // Δg 모델용: 초기 g_ft(0) 저장
    YMatrix                  g_ft_init_;     // FT frame initial gravity
    bool                     g_ft_init_set_;

    std::string              log_file_path_;
    std::ofstream            log_ofs_;
    std::mutex               log_ofs_mutex_;
    // Async logger
    std::deque<std::string>  log_queue_;
    std::mutex               log_queue_mutex_;
    std::condition_variable  log_queue_cv_;
    std::thread              log_thread_;
    std::atomic<bool>        log_thread_running_{false};
    size_t                   log_queue_max_ = 10000; // bounded queue
    // Diagnostics counters
    std::atomic<uint64_t>    transfer_dbg_count_{0};
    std::atomic<uint64_t>    transfer_dbg_accum_us_{0};
    std::atomic<uint64_t>    ftget_dbg_accum_us_{0};

    // ----- FT Raw getter -----
    #if USE_ECAN
    FT_eCANGet FT1SensorGet;
    #else
    FT_EtherGet FT1SensorGet;
    #endif

    // ----- Timer -----
    rclcpp::TimerBase::SharedPtr timer_;

    // ----- Publisher / Subscriber -----
    geometry_msgs::msg::WrenchStamped ft1_msg;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ft1_pub;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr currentP_sub;

    // ----- Service ----
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr print_param_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr save_log_srv_;

    // ----- 내부 함수 -----
    FTData filtering(const FTData& ftdata);
    void currentPCB(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    void printParamsCB(const std_srvs::srv::SetBool::Request::SharedPtr req,
                       std_srvs::srv::SetBool::Response::SharedPtr res);
    void saveLogCB(const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res);
};


// ============================================
// 생성자
// ============================================
FTGetMain::FTGetMain(const std::string& ft_ip,
                     int ft_port,
                     const std::string& robot_name,
                     int mov_size)
    : Node("FTGetMain")
    , movFilter()
    , current_TCP_pose(6, 0.0)
    , ROT_Base2TCP(3, 3)
    , last_GcompForce(3, 1)
    , last_GcompMoment(3, 1)
    , FT1SensorGet(ft_ip, ft_port)
    , SframeForce_(3, 1)
    , SframeMoment_(3, 1)
    , SGravityForce_(3, 1)
    , SGravityMoment_(3, 1)
    , ScompForce_(3, 1)
    , ScompMoment_(3, 1)
    , TCPForce_(3, 1)
    , TCPMoment_(3, 1)
    , RframeForce_(3, 1)
    , RframeMoment_(3, 1)
    , GcompForce_(3, 1)
    , GcompMoment_(3, 1)
    , cross_temp_(3, 1)
    , g_base_(3, 1)
    , g_tcp_(3, 1)
    , ROT_TCP2FT_T_(3, 3)
    , g_ft_(3, 1)
    , g_ft_init_(3, 1)
    , g_ft_init_set_(false)
{
    // ----- Publisher -----
    std::string ftdata_topic = robot_name + std::string("/ftdata");
    ft1_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(ftdata_topic, 10);

    // ----- Subscriber (현재 TCP 포즈) -----
    std::string currentP_topic = robot_name + std::string("/currentP");

    printf("\033[33m[FTGetMain] robot_topic_name: %s \033[0m\n", currentP_topic.c_str());

    currentP_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        currentP_topic,
        10,
        std::bind(&FTGetMain::currentPCB, this, std::placeholders::_1)
    );

    // ----- Service: print parameters -----
    print_param_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "print_ft_params",
        std::bind(&FTGetMain::printParamsCB, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    // ----- log 파일 경로 설정 -----
    try {
        auto share_dir = ament_index_cpp::get_package_share_directory("Y2FT_AQ");
        std::filesystem::path log_dir =
            std::filesystem::path(share_dir).parent_path() / "log";
        std::filesystem::create_directories(log_dir);
        log_file_path_ = (log_dir / "ft_gcomp_log.txt").string();
        RCLCPP_INFO(this->get_logger(), "Log file: %s", log_file_path_.c_str());
        // Open log file once
        try {
            log_ofs_.open(log_file_path_, std::ios::app);
            if (!log_ofs_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open log file for appending: %s", log_file_path_.c_str());
            } else {
                log_ofs_ << std::fixed << std::setprecision(6);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception opening log file: %s", e.what());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to resolve log dir: %s", e.what());
    }

    // Start async logger thread if file was opened
    if (log_ofs_) {
        log_thread_running_.store(true);
        log_thread_ = std::thread([this]() {
            std::vector<std::string> batch;
            batch.reserve(256);
            while (log_thread_running_.load() || !log_queue_.empty()) {
                {
                    std::unique_lock<std::mutex> ql(log_queue_mutex_);
                    log_queue_cv_.wait_for(ql, std::chrono::milliseconds(200), [this]() {
                        return !log_queue_.empty() || !log_thread_running_.load();
                    });

                    while (!log_queue_.empty() && batch.size() < 512) {
                        batch.push_back(std::move(log_queue_.front()));
                        log_queue_.pop_front();
                    }
                }

                if (!batch.empty()) {
                    std::lock_guard<std::mutex> lf(log_ofs_mutex_);
                    for (auto &line : batch) {
                        log_ofs_ << line;
                    }
                    log_ofs_.flush();
                    batch.clear();
                }
            }
            std::lock_guard<std::mutex> lf(log_ofs_mutex_);
            if (log_ofs_.is_open()) log_ofs_.flush();
        });
    }

    // ----- Service: save log -----
    save_log_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "save_ft_log",
        std::bind(&FTGetMain::saveLogCB, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    // frame id
    ft1_msg.header.frame_id = "ft_frame";

    // MOV 필터 초기화 (Fx, Fy, Fz, Mx, My, Mz)
    movFilter.reserve(6);
    for (int i = 0; i < 6; ++i) {
        movFilter.emplace_back(mov_size);
    }

    // Base → TCP 회전행렬 초기값: Identity
    ROT_Base2TCP = YMatrix::identity(3);

    // 타이머 시작
    auto period = std::chrono::duration<double>(1.0 / static_cast<double>(SAMPLING_FREQ_HZ));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FTGetMain::transferData, this)
    );
}


// ============================================
// currentP Subscriber 콜백
// ============================================
void FTGetMain::currentPCB(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    // [x,y,z,wx,wy,wz]
    for (size_t i = 0; i < 6 && i < msg->data.size(); ++i) {
        current_TCP_pose[i] = msg->data[i];
    }

    // Spatial Angle → 회전행렬 (Base -> TCP)
    SpatialAngle spatial_angle(
        current_TCP_pose[3],
        current_TCP_pose[4],
        current_TCP_pose[5]
    );

    ROT_Base2TCP.insert(0, 0, YMatrix::fromSpatialAngle(spatial_angle));
}


// ============================================
// print_ft_params Service 콜백
// ============================================
void FTGetMain::printParamsCB(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
{
    (void)req;  // unused

    std::ostringstream oss;
    oss << "\n";
    oss << "===== FTGetMain Parameters =====\n";
    oss << "USE_ECAN                 : " << USE_ECAN << " (1: eCAN, 0: Ethernet)\n";
    oss << "GRAVITY_COMPENSATION_MODE: " << GRAVITY_COMPENSATION_MODE << " (0: Disabled, 1: Enabled)\n";
    oss << "FT1_IP                   : " << FT1_IP << "\n";
    oss << "FT1_PORT                 : " << FT1_PORT << "\n";
    oss << "SAMPLING_FREQ_HZ         : " << SAMPLING_FREQ_HZ << " [Hz]\n";
    oss << "SENSOR_INIT_COUNT_NUM    : " << SENSOR_INIT_COUNT_NUM << "\n";
    oss << "MOV_SIZE                 : " << MOV_SIZE << "\n";
    oss << "ROBOT_NAME               : " << ROBOT_NAME << "\n";

    // ROT_TCP2FT 출력 (3x3)
    oss << "ROT_TCP2FT (3x3):\n";
    for (int i = 0; i < 3; ++i)
    {
        oss << "  [ ";
        for (int j = 0; j < 3; ++j)
        {
            oss << ROT_TCP2FT[i][j];
            if (j < 2) oss << ", ";
        }
        oss << " ]\n";
    }

    // TOOL_MASS, TOOL_COG 출력
    oss << "TOOL_MASS [kg]: " << TOOL_MASS << "\n";
    oss << "TOOL_COG (FT frame) [m]: [ "
        << TOOL_COG[0][0] << ", "
        << TOOL_COG[1][0] << ", "
        << TOOL_COG[2][0] << " ]\n";

    std::string msg = oss.str();
    RCLCPP_INFO_STREAM(this->get_logger(), msg);

    res->success = true;
    res->message = msg;
}


// ============================================
// save_ft_log Service 콜백
// ============================================
void FTGetMain::saveLogCB(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
{
    if (!req->data) {
        res->success = false;
        res->message = "data=false: nothing written";
        return;
    }

    if (!log_ofs_) {
        res->success = false;
        res->message = "log file not open";
        RCLCPP_ERROR(this->get_logger(), "Log file is not open: %s", log_file_path_.c_str());
        return;
    }

    std::ostringstream oss;
    oss << this->now().seconds() << " ";
    oss << last_GcompForce[0][0]  << " " << last_GcompForce[1][0]  << " " << last_GcompForce[2][0]  << " ";
    oss << last_GcompMoment[0][0] << " " << last_GcompMoment[1][0] << " " << last_GcompMoment[2][0] << " ";
    oss << current_TCP_pose[3]    << " " << current_TCP_pose[4]    << " " << current_TCP_pose[5]    << "\n";
    std::string line = oss.str();

    {
        std::lock_guard<std::mutex> ql(log_queue_mutex_);
        if (log_queue_.size() >= log_queue_max_) {
            res->success = false;
            res->message = "log queue full: dropped";
            RCLCPP_WARN(this->get_logger(), "Log queue full, dropping log entry");
            return;
        }
        log_queue_.push_back(std::move(line));
    }
    log_queue_cv_.notify_one();

    res->success = true;
    res->message = "Queued log entry";
}


// ============================================
// FT 데이터 MOV 필터링
// ============================================
FTData FTGetMain::filtering(const FTData& ftdata)
{
    FTData filtered_data;

    filtered_data.Fx = movFilter[0].MovFilter(ftdata.Fx);
    filtered_data.Fy = movFilter[1].MovFilter(ftdata.Fy);
    filtered_data.Fz = movFilter[2].MovFilter(ftdata.Fz);

    filtered_data.Mx = movFilter[3].MovFilter(ftdata.Mx);
    filtered_data.My = movFilter[4].MovFilter(ftdata.My);
    filtered_data.Mz = movFilter[5].MovFilter(ftdata.Mz);

    return filtered_data;
}


// ============================================
// 타이머 콜백: 메인 처리 루프
// ============================================
void FTGetMain::transferData()
{
    using std::chrono::steady_clock;
    auto t_start = steady_clock::now();

    FTData filtered_out;

    // Reset vectors
    for (int i = 0; i < 3; ++i) {
        SframeForce_[i][0]   = 0.0;
        SframeMoment_[i][0]  = 0.0;
        SGravityForce_[i][0] = 0.0;
        SGravityMoment_[i][0]= 0.0;
        ScompForce_[i][0]    = 0.0;
        ScompMoment_[i][0]   = 0.0;
        TCPForce_[i][0]      = 0.0;
        TCPMoment_[i][0]     = 0.0;
        RframeForce_[i][0]   = 0.0;
        RframeMoment_[i][0]  = 0.0;
        GcompForce_[i][0]    = 0.0;
        GcompMoment_[i][0]   = 0.0;
    }

    // 센서 초기 offset 계산 (내부에서 한 번만 동작한다고 가정)
    FT1SensorGet.FT_init(SENSOR_INIT_COUNT_NUM);

    // Raw 데이터 읽고 필터링
    auto t_ft0 = steady_clock::now();
    filtered_out = filtering(FT1SensorGet.FTGet());
    auto t_ft1 = steady_clock::now();
    uint64_t ft_dur_us = std::chrono::duration_cast<std::chrono::microseconds>(t_ft1 - t_ft0).count();
    ftget_dbg_accum_us_.fetch_add(ft_dur_us, std::memory_order_relaxed);

    // 센서 frame 힘/모멘트
    SframeForce_[0][0]  = filtered_out.Fx;
    SframeForce_[1][0]  = filtered_out.Fy;
    SframeForce_[2][0]  = filtered_out.Fz;

    SframeMoment_[0][0] = filtered_out.Mx;
    SframeMoment_[1][0] = filtered_out.My;
    SframeMoment_[2][0] = filtered_out.Mz;

    // ================================
    // 1) 중력 벡터 g_FT 계산 (Base -> TCP -> FT)
    //    + Δg 모델 적용
    // ================================
#if GRAVITY_COMPENSATION_MODE
    if (TOOL_MASS > 0.0) {

        // Base frame 중력: -z 방향이 중력 방향이라고 가정 (python과 동일)
        for (int i = 0; i < 3; ++i) g_base_[i][0] = 0.0;
        g_base_[2][0] = -9.81;

        // // Base -> TCP 회전
        // for (int i = 0; i < 3; ++i) {
        //     g_tcp_[i][0] = 0.0;
        //     for (int k = 0; k < 3; ++k) {
        //         g_tcp_[i][0] += ROT_Base2TCP[i][k] * g_base_[k][0];
        //     }
        // }

        // g_tcp = ROT_Base2TCP^T * g_base
        for (int i = 0; i < 3; ++i) {
            g_tcp_[i][0] = 0.0;
            for (int k = 0; k < 3; ++k) {
                g_tcp_[i][0] += ROT_Base2TCP[k][i] * g_base_[k][0];  // TRANSPOSE HERE
            }
        }

        // // TCP -> FT = (ROT_TCP2FT)^T
        // for (int i = 0; i < 3; ++i) {
        //     for (int j = 0; j < 3; ++j) {
        //         ROT_TCP2FT_T_[i][j] = ROT_TCP2FT[j][i];
        //     }
        // }

        // // Base 중력을 FT frame 으로
        // for (int i = 0; i < 3; ++i) {
        //     g_ft_[i][0] = 0.0;
        //     for (int k = 0; k < 3; ++k) {
        //         g_ft_[i][0] += ROT_TCP2FT_T_[i][k] * g_tcp_[k][0];
        //     }
        // }

        // 최초 한 번 g_ft_init_ 저장 (센서 init 자세의 g_s(0)에 해당)
        if (!g_ft_init_set_) {
            for (int i = 0; i < 3; ++i) {
                g_ft_init_[i][0] = g_tcp_[i][0];
            }
            g_ft_init_set_ = true;
        }

        // Δg_ft = g_ft(k) - g_ft(0)
        for (int i = 0; i < 3; ++i) {
            double dg = g_tcp_[i][0] - g_ft_init_[i][0];
            SGravityForce_[i][0] = dg * TOOL_MASS;  // F_g = m * Δg_s
        }

        // τ_g = r × F_g
        cross3(TOOL_COG, SGravityForce_, SGravityMoment_);
    }
#endif  // GRAVITY_COMPENSATION_MODE

    // ================================
    // 2) 센서 프레임에서 중력 보상
    // ================================
// #if GRAVITY_COMPENSATION_MODE
//     for (int i = 0; i < 3; ++i) {
//         ScompForce_[i][0]  = SframeForce_[i][0]  - SGravityForce_[i][0];
//         ScompMoment_[i][0] = SframeMoment_[i][0] - SGravityMoment_[i][0];
//     }
// #else
//     for (int i = 0; i < 3; ++i) {
//         ScompForce_[i][0]  = SframeForce_[i][0];
//         ScompMoment_[i][0] = SframeMoment_[i][0];
//     }
// #endif

    // ================================
    // 2) Sensor -> TCP -> Base 변환
    // ================================
    // FT(local) → TCP(local): TCPForce = ROT_TCP2FT * ScompForce
    for (int i = 0; i < 3; ++i) {
        TCPForce_[i][0] = 0.0;
        for (int k = 0; k < 3; ++k) {
            TCPForce_[i][0] += ROT_TCP2FT[i][k] * SframeForce_[k][0];
        }
    }
    // TCPMoment = ROT_TCP2FT * ScompMoment
    for (int i = 0; i < 3; ++i) {
        TCPMoment_[i][0] = 0.0;
        for (int k = 0; k < 3; ++k) {
            TCPMoment_[i][0] += ROT_TCP2FT[i][k] * SframeMoment_[k][0];
        }
    }

    // ================================
    // 3) TCP 프레임에서 중력 보상
    // ================================
#if GRAVITY_COMPENSATION_MODE
    for (int i = 0; i < 3; ++i) {
        TCPForce_[i][0]  = TCPForce_[i][0]  - SGravityForce_[i][0];
        TCPMoment_[i][0] = TCPMoment_[i][0] - SGravityMoment_[i][0];
    }
#else
    for (int i = 0; i < 3; ++i) {
        TCPForce_[i][0]  = TCPForce_[i][0];
        TCPMoment_[i][0] = TCPMoment_[i][0];
    }
#endif


    // TCP(local) → Base frame: RframeForce = ROT_Base2TCP * TCPForce
    for (int i = 0; i < 3; ++i) {
        RframeForce_[i][0] = 0.0;
        for (int k = 0; k < 3; ++k) {
            RframeForce_[i][0] += ROT_Base2TCP[i][k] * TCPForce_[k][0];
        }
    }
    // RframeMoment = ROT_Base2TCP * TCPMoment
    for (int i = 0; i < 3; ++i) {
        RframeMoment_[i][0] = 0.0;
        for (int k = 0; k < 3; ++k) {
            RframeMoment_[i][0] += ROT_Base2TCP[i][k] * TCPMoment_[k][0];
        }
    }

    // 최종 publish용
    for (int i = 0; i < 3; ++i) {
        GcompForce_[i][0]  = RframeForce_[i][0];
        GcompMoment_[i][0] = RframeMoment_[i][0];
        last_GcompForce[i][0]  = TCPForce_[i][0];
        last_GcompMoment[i][0] = TCPMoment_[i][0];
    }

    ft1_msg.header.stamp = this->now();
    ft1_msg.wrench.force.x  = GcompForce_[0][0];
    ft1_msg.wrench.force.y  = GcompForce_[1][0];
    ft1_msg.wrench.force.z  = GcompForce_[2][0];
    ft1_msg.wrench.torque.x = GcompMoment_[0][0];
    ft1_msg.wrench.torque.y = GcompMoment_[1][0];
    ft1_msg.wrench.torque.z = GcompMoment_[2][0];

    ft1_pub->publish(ft1_msg);

    // (디버그 타이밍은 필요하면 다시 켜기)
}


// ============================================
// main
// ============================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTGetMain>(
        std::string(FT1_IP),
        FT1_PORT,
        std::string(ROBOT_NAME),
        MOV_SIZE
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}


FTGetMain::~FTGetMain()
{
    // Stop logger thread
    log_thread_running_.store(false);
    log_queue_cv_.notify_all();
    if (log_thread_.joinable()) {
        log_thread_.join();
    }

    // Close log file if open
    std::lock_guard<std::mutex> lk(log_ofs_mutex_);
    if (log_ofs_.is_open()) {
        log_ofs_.flush();
        log_ofs_.close();
    }
}
