#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include "nrs_forcecon/nrs_3step_faac.h"  // SimpleKalmanFilter 클래스 헤더 포함

// 텍스트 파일을 로드하여 특정 열 값들을 추출하고, 결과를 새로운 파일에 저장하는 함수
void loadDataAndRunKalmanFilter(const std::string& inputFilename, const std::string& outputFilename, const int colSize, const int colIndex, SimpleKalmanFilter& kalmanFilter) {
    std::ifstream inputFile(inputFilename);
    std::ofstream outputFile(outputFilename);  // 결과를 저장할 새로운 텍스트 파일
    
    // 파일이 정상적으로 열렸는지 확인
    if (!inputFile.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << inputFilename << std::endl;
        return;
    }

    if (!outputFile.is_open()) {
        std::cerr << "출력 파일을 열 수 없습니다: " << outputFilename << std::endl;
        return;
    }

    // 파일의 각 줄에 대해 처리
    std::string line;
    while (std::getline(inputFile, line)) {
        std::stringstream ss(line);
        std::vector<double> colVector(colSize);  // 벡터로 변환할 수 있는 배열

        // 텍스트 파일에서 값을 읽어들입니다. 예시로 첫 번째 열을 읽음
        for (int i = 0; i < colSize; ++i) {
            ss >> colVector[i];
        }
        
        // 칼만 필터를 업데이트 (측정값 전달)
        kalmanFilter.predict();
        kalmanFilter.update(colVector[colIndex]);
        
        // 필터링된 상태 추정값 (위치, 속도, 가속도)
        double position = kalmanFilter.getPosition();
        double velocity = kalmanFilter.getVelocity();
        double acceleration = kalmanFilter.getAcceleration();

        // 결과를 새로운 파일에 저장
        outputFile << colVector[colIndex] << "\t" << position << "\t" << velocity << "\t" << acceleration << std::endl;
    }

    inputFile.close();
    outputFile.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kf_acc_tester");
    ros::NodeHandle nh;
    // 칼만 필터 생성 (dt=0.1, process_noise=0.01, measurement_noise=1.0)
    std::vector<double> processNoise = {0.1, 0.1, 0.1};  // 프로세스 노이즈
    std::vector<double> measurementNoise = {1.0, 1.0, 1.0};  // 측정 노이즈
    SimpleKalmanFilter kalman(0.002, processNoise, measurementNoise);

    // File path setting
    std::string packagePath = "/home/nrsur10/catkin_ws/src/nrs_forcecon";  // 패키지 경로 가져오기
    std::string inputFilePath = packagePath + "/data/raw_data.txt";  // 입력 파일 경로
    std::string kfoutFilePath = packagePath + "/data/KFfiltered_results.txt";  // 출력 파일 경로

    // 파일 로드 및 칼만 필터 실행
    loadDataAndRunKalmanFilter(inputFilePath, kfoutFilePath, 14, 2, kalman); // inputFilename, outputFilename, kalmanFilter;

    std::cout << "결과가 'KFfiltered_results.txt'에 저장되었습니다." << std::endl;

    return 0;
}
