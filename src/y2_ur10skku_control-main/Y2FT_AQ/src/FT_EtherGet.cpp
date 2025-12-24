#include "Y2FT_AQ/FT_EtherGet.hpp"


FT_EtherGet::FT_EtherGet(const std::string& IP, const int PORT):
IP_(IP), PORT_(PORT), init_force(3,0.0), init_moment(3,0.0)
{
    // 1️⃣ 소켓 생성
    s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        std::exit(1);
    }

    // 2️⃣ 주소 설정
    struct sockaddr_in sensorAddr;
    sensorAddr.sin_family = AF_INET;
    sensorAddr.sin_port = htons(PORT_);
    inet_pton(AF_INET, IP_.c_str(), &sensorAddr.sin_addr);

    // 3️⃣ 타임아웃 설정 (2초)
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 4️⃣ 데이터 전송
    std::string sendData = "000302";
    std::vector<unsigned char> sendDataBytes(sendData.size() / 2);
    for (size_t i = 0; i < sendData.size(); i += 2) {
        sendDataBytes[i / 2] = std::stoi(sendData.substr(i, 2), nullptr, 16);
    }

    int sentBytes = sendto(s, sendDataBytes.data(), sendDataBytes.size(), 0,
                           (struct sockaddr *)&sensorAddr, sizeof(sensorAddr));
    if (sentBytes < 0) {
        perror("[ERROR] Send failed");
        close(s);
        std::exit(1);
    }

    std::cout << "[INFO] Sent " << sentBytes << " bytes to sensor.\n";
}

FT_EtherGet::~FT_EtherGet()
{
    close(s);
    std::cout << "[INFO] Socket closed.\n";
}

/* Data initialization */
bool FT_EtherGet::FT_init(const unsigned int init_count_num)
{
    FTData ftdata;
    if(init_count == 0)
    {
        init_flag = false;
        init_count++;
    }
    else if(init_count<=init_count_num)
    {
        ftdata = FTGet();
        init_force[0] += ftdata.Fx/static_cast<double>(init_count_num);
        init_force[1] += ftdata.Fy/static_cast<double>(init_count_num);
        init_force[2] += ftdata.Fz/static_cast<double>(init_count_num);

        init_moment[0] += ftdata.Mx/static_cast<double>(init_count_num);
        init_moment[1] += ftdata.My/static_cast<double>(init_count_num);
        init_moment[2] += ftdata.Mz/static_cast<double>(init_count_num);
        init_count++;
    }
    else
    {
        init_flag = true;
    }
    
    return init_flag;

}

/* Data Aquisition */
FTData FT_EtherGet::FTGet()
{
    FTData ftdata;
    char *recvData = recvMsg();
    if (!recvData) {
        std::cerr << "[ERROR] No data received, exiting loop.\n";
        std::exit(1);
    }

    /* Get the raw data */
    ftdata.Fx = unpackFloat(recvData) - static_cast<double>(init_flag)*init_force[0];
    ftdata.Fy = unpackFloat(recvData+4) - static_cast<double>(init_flag)*init_force[1];
    ftdata.Fz = unpackFloat(recvData+8) - static_cast<double>(init_flag)*init_force[2];
    ftdata.Mx = unpackFloat(recvData+12) - static_cast<double>(init_flag)*init_moment[0];
    ftdata.My = unpackFloat(recvData+16) - static_cast<double>(init_flag)*init_moment[1];
    ftdata.Mz = unpackFloat(recvData+20) - static_cast<double>(init_flag)*init_moment[2];

    return ftdata;
}



/*** Base functions ***/

// Function to unpack a float from a byte array
float FT_EtherGet::unpackFloat(const char *bytes) {
    uint32_t asInt = 0;
    std::memcpy(&asInt, bytes, sizeof(asInt));
    asInt = ntohl(asInt); // Convert from network byte order to host byte order
    float result;
    std::memcpy(&result, &asInt, sizeof(result));
    return result;
}

// Function to receive message with retry logic
char* FT_EtherGet::recvMsg() {
    static char recvData[RECV_SIZE];
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);

    int attempt = 0;
    while (attempt < MAX_RETRY) {
        ssize_t bytesReceived = recvfrom(s, recvData, sizeof(recvData), 0, 
                                         (struct sockaddr *)&from, &fromLen);

        if (bytesReceived == RECV_SIZE) {
            // std::cout << "[INFO] Received " << bytesReceived << " bytes from sensor.\n";
            return recvData;
        } else if (bytesReceived < 0) {
            perror("[ERROR] Failed to receive data");
        } else {
            std::cout << "[WARNING] Received " << bytesReceived 
                      << " bytes, expected " << RECV_SIZE << ". Retrying...\n";
        }

        attempt++;
        usleep(500000); // 0.5초 대기
    }

    std::cerr << "[ERROR] Failed to receive proper data after " 
              << MAX_RETRY << " attempts.\n";
    return nullptr;
}