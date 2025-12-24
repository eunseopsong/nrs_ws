#include "Y2FT_AQ/FT_eCANGet.hpp"
#include <fcntl.h>
#include <errno.h>

// =====================================================
// Constructor
// =====================================================
FT_eCANGet::FT_eCANGet(const std::string& IP, const int PORT)
    : IP_(IP)
    , PORT_(PORT)
    , init_force(3, 0.0)
    , init_moment(3, 0.0)
    , have_last_stamp_(false)
    , sample_count_(0)
    , sum_dt_us_(0)
{
    std::printf("[FT_eCANGet] Initializing with IP: %s, PORT: %d\n",
                IP_.c_str(), PORT_);

    std::printf("[FT_eCANGet] CAN_ID: %d\n", Sensor_ID);

    // 5초 대기 시작 시간 기록
    start_time_        = std::chrono::steady_clock::now();
    last_wait_msg_time_ = start_time_;

    // TCP/IP 소켓 생성
    clnt_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (clnt_sock == -1) {
        std::perror("[FT_eCANGet] socket() ERR");
        return;
    }

    std::memset(&st_serv_addr, 0, sizeof(st_serv_addr));
    st_serv_addr.sin_family      = AF_INET;
    st_serv_addr.sin_addr.s_addr = inet_addr(IP_.c_str());
    st_serv_addr.sin_port        = htons(PORT_);

    int connret = connect(
        clnt_sock,
        reinterpret_cast<struct sockaddr*>(&st_serv_addr),
        sizeof(st_serv_addr)
    );
    if (connret == -1) {
        std::perror("[FT_eCANGet] connect() ERR");
        close(clnt_sock);
        clnt_sock = -1;
        return;
    }

    // Set socket to non-blocking so read() won't block the timer thread
    int flags = fcntl(clnt_sock, F_GETFL, 0);
    if (flags != -1) {
        if (fcntl(clnt_sock, F_SETFL, flags | O_NONBLOCK) == -1) {
            std::perror("[FT_eCANGet] fcntl() O_NONBLOCK ERR");
        }
    }

    int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);
    if (iResult == -1) {
        std::perror("[FT_eCANGet] send() ERR");
    } else {
        std::cout << "[FT_eCANGet] Bytes Sent: " << iResult << std::endl;
    }
}

// =====================================================
// Destructor
// =====================================================
FT_eCANGet::~FT_eCANGet()
{
    if (clnt_sock >= 0) {
        close(clnt_sock);
    }
    std::cout << "[FT_eCANGet] Socket closed.\n";
}

// =====================================================
// Initialization (5초 대기 + Force/Moment 별도 평균)
// =====================================================
bool FT_eCANGet::FT_init(const unsigned int init_count_num)
{
    auto now = std::chrono::steady_clock::now();
    double elapsed_sec =
        std::chrono::duration<double>(now - start_time_).count();

    // 5초 대기 로직
    if (!wait_5sec_passed_) {
        double dt_msg =
            std::chrono::duration<double>(now - last_wait_msg_time_).count();

        if (dt_msg > 1.0) {  // 1초마다 메시지
            printf("\033[32m[FT_eCANGet] Waiting 5 sec before initialization... (%.2f sec) \033[0m\n",
                   elapsed_sec);

            last_wait_msg_time_ = now;
        }

        if (elapsed_sec < 5.0) {
            return false;    // 아직 초기화 시작 X
        }

        wait_5sec_passed_ = true;
        printf("\033[33m][FT_eCANGet] 5 sec passed. Starting initialization.\033[0m\n");

        // 5초 후 초기화 시작할 때 offset/카운터 리셋
        init_flag = false;
        init_force.assign(3, 0.0);
        init_moment.assign(3, 0.0);
        init_count  = 0;
    }

    // 이미 init 끝난 상태면 바로 true 리턴
    if (init_flag) {
        return true;
    }

    // 여기부터 실제 offset 수집
    FTData ftdata = FTGet();

    if (init_count < init_count_num) {
        init_force[0] += ftdata.Fx / static_cast<double>(init_count_num);
        init_force[1] += ftdata.Fy / static_cast<double>(init_count_num);
        init_force[2] += ftdata.Fz / static_cast<double>(init_count_num);

        init_moment[0] += ftdata.Mx / static_cast<double>(init_count_num);
        init_moment[1] += ftdata.My / static_cast<double>(init_count_num);
        init_moment[2] += ftdata.Mz / static_cast<double>(init_count_num);

        init_count++;

        return false;
    }

    // 여기까지 오면 둘 다 충분히 모인 것
    if (!init_flag) {
        printf("\033[31m][FT_eCANGet] 5 sec passed. Initialization complete.\033[0m\n");
        std::cout << "  Force offsets : "
                  << init_force[0] << ", "
                  << init_force[1] << ", "
                  << init_force[2] << "\n"
                  << "  Moment offsets: "
                  << init_moment[0] << ", "
                  << init_moment[1] << ", "
                  << init_moment[2] << std::endl;
    }

    init_flag = true;
    return true;
}

// =====================================================
// Data Acquisition
// =====================================================
FTData FT_eCANGet::FTGet()
{
    // FTData FTGet_ftdata;  // 기본 0으로 초기화 

    if (clnt_sock < 0) {
        return FTGet_ftdata;
    }

    readstrlen = read(
        clnt_sock,
        reinterpret_cast<char*>(recvmsg),
        sizeof(recvmsg)
    );
    if (readstrlen == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // non-blocking socket: no data available right now
            return FTGet_ftdata;
        } else {
            std::perror("[FT_eCANGet] read() ERR");
            return FTGet_ftdata;
        }
    } else if (readstrlen == 0) {
        // peer closed connection
        std::cerr << "[FT_eCANGet] peer closed connection\n";
        if (clnt_sock >= 0) {
            close(clnt_sock);
            clnt_sock = -1;
        }
        return FTGet_ftdata;
    }

    unsigned char frame_id = recvmsg[4];
    // ------------------------------------------------
    // Force frame
    // ------------------------------------------------
    if (frame_id == Sensor_ID)
    {
        // 샘플링 주기 / 주파수 측정 (remove std::cout from loop for performance)
        auto now = std::chrono::steady_clock::now();
        unsigned int expression_count = 3000;
        if (have_last_stamp_) {
            auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
                             now - last_stamp_).count();
            sum_dt_us_   += dt_us;
            sample_count_++;

            if (sample_count_ == expression_count) {
                // Print timing info less frequently to avoid I/O overhead
                double mean_dt   = sum_dt_us_ / (double)expression_count;
                double mean_freq = 1e6 / mean_dt;
                std::printf("[FT_eCANGet] mean dt = %.2f us, mean freq ≒ %.3f Hz (sample_count: %u)\n", 
                           mean_dt, mean_freq, expression_count);
                sample_count_ = 0;
                sum_dt_us_    = 0;
            }
        } else {
            have_last_stamp_ = true;
        }
        last_stamp_ = now;

        // Force 값 (init_flag가 true면 offset 적용)
        FTGet_ftdata.Fx = unpackFloat(recvmsg, 0, 0) - (init_flag ? init_force[0] : 0.0);
        FTGet_ftdata.Fy = unpackFloat(recvmsg, 1, 0) - (init_flag ? init_force[1] : 0.0);
        FTGet_ftdata.Fz = unpackFloat(recvmsg, 2, 0) - (init_flag ? init_force[2] : 0.0);

    }
    // ------------------------------------------------
    // Moment frame
    // ------------------------------------------------
    else if (frame_id == Sensor_ID + 1)
    {
        FTGet_ftdata.Mx = unpackFloat(recvmsg, 0, 1) - (init_flag ? init_moment[0] : 0.0);
        FTGet_ftdata.My = unpackFloat(recvmsg, 1, 1) - (init_flag ? init_moment[1] : 0.0);
        FTGet_ftdata.Mz = unpackFloat(recvmsg, 2, 1) - (init_flag ? init_moment[2] : 0.0);

    }
    // ------------------------------------------------
    // Linear acceleration frame
    // ------------------------------------------------
    else if (frame_id == Sensor_ID + 2)
    {
        FTGet_ftdata.LAx = unpackFloat(recvmsg, 0, 2);
        FTGet_ftdata.LAy = unpackFloat(recvmsg, 1, 2);
        FTGet_ftdata.LAz = unpackFloat(recvmsg, 2, 2);
    }
    // ------------------------------------------------
    // Angular acceleration frame
    // ------------------------------------------------
    else if (frame_id == Sensor_ID + 3)
    {
        FTGet_ftdata.AAx = unpackFloat(recvmsg, 0, 3);
        FTGet_ftdata.AAy = unpackFloat(recvmsg, 1, 3);
        FTGet_ftdata.AAz = unpackFloat(recvmsg, 2, 3);
    }

    return FTGet_ftdata;
}

// =====================================================
// unpackFloat
// =====================================================
float FT_eCANGet::unpackFloat(unsigned char* recvmsg, int i, int data_type)
{
    float result = 0.0f;

    int16_t raw = static_cast<int16_t>(
        (static_cast<int>(recvmsg[6 + 2*i]) << 8) |
        static_cast<int>(recvmsg[7 + 2*i])
    );

    if (data_type == 0) {          // Force
        result = raw / 100.0f - 300.0f;
    }
    else if (data_type == 1) {     // Moment
        result = raw / 500.0f - 50.0f;
    }
    else if (data_type == 2) {     // Linear acc (m/s^2)
        double v = raw * 2.0 - 65535.0;
        v = (v / 16384.0) * 9.81;
        result = static_cast<float>(v);
    }
    else if (data_type == 3) {     // Angular acc (rad/s^2) - DEPRECATED
        // State variables removed to prevent floating-point accumulation
        result = 0.0f;
    }

    return result;
}
