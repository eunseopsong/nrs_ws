#include "NRSUR10eControl.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

NRSUR10eControl::NRSUR10eControl(double rtde_frequency, int rt_control_priority, int rt_receive_priority)
    : rtde_control(nullptr), rtde_receive(nullptr) { 
    this->dt = 1.0 / rtde_frequency;
    this->velocity = 0.5;
    this->acceleration = 10.0;
    this->lookahead_time = 0.1;
    this->gain = 600;
    this->running = true;
    this->stop_flag = false;
}

NRSUR10eControl::~NRSUR10eControl() {
    delete rtde_control;
    delete rtde_receive;
}

void NRSUR10eControl::initialize(const std::string& robot_ip) {
    // ROS 초기화 및 메시지 구독/발행
    ros::NodeHandle nh;
    joint_angle_pub = nh.advertise<std_msgs::Float64MultiArray>("UR10_Jangle", 20);
    pose_pub = nh.advertise<std_msgs::Float64MultiArray>("UR10_pose", 20);
    ft_sub = nh.subscribe("ftsensor", 10, &NRSUR10eControl::ftDataCallback, this);
    vr_sub = nh.subscribe("vrpose", 10, &NRSUR10eControl::vrDataCallback, this);
    cmd_mode_sub = nh.subscribe("mode_cmd", 10, &NRSUR10eControl::cmdModeCallback, this);
    joint_cmd_sub = nh.subscribe("joint_cmd", 10, &NRSUR10eControl::jointCmdCallback, this);
    pb_iter_sub = nh.subscribe("pb_iter", 10, &NRSUR10eControl::pbIterCallback, this);

    // YAML 설정 파일 로드
    loadYamlConfigurations("config.yaml");

    // RTDEControlInterface 및 RTDEReceiveInterface 초기화
    rtde_control = new ur_rtde::RTDEControlInterface(robot_ip, dt, ur_rtde::RTDEControlInterface::FLAG_VERBOSE);
    rtde_receive = new ur_rtde::RTDEReceiveInterface(robot_ip, dt);
}

void NRSUR10eControl::loadYamlConfigurations(const std::string& config_file) {
    try {
        // YAML 파일 로드
        YAML::Node config = YAML::LoadFile(config_file);
        
        // VR 설정 파일
        NRS_VR_setting = config["VR_setting"];
        
        // Force control 설정 파일
        NRS_Fcon_setting = config["Force_control"];
        
        // Recording 설정 파일
        NRS_recording = config["Recording"];
        
        // 로봇 IP 로드 (YAML 파일에서 UR10IP 항목을 읽어옴)
        if (config["UR10IP"]) {
            robot_ip = config["UR10IP"].as<std::string>();  // 로봇 IP를 설정
            std::cout << "Robot IP: " << robot_ip << std::endl; // IP 확인 출력
        } else {
            std::cerr << "Robot IP not found in YAML configuration file!" << std::endl;
        }

        // YAML 파일 로드 확인
        std::cout << "YAML configurations loaded successfully." << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load YAML configuration file: " << e.what() << std::endl;
    }
}


void NRSUR10eControl::runControlLoop() {
    double t_start = ros::Time::now().toSec();  // 시작 시간
    while (running) {
        // 실시간 제어 루프
        getActualQ();
        handleForceControl();
        controlRobot();
        
        // RTDE control 주기 맞추기
        rtde_control->waitPeriod(t_start);

        // ROS 메시지 처리 - 별도의 쓰레드나 타이밍으로 처리
        if (ros::ok()) {
            ros::spinOnce(); // 메시지 콜백 처리
        }

        // 주기 시간 계산
        t_start = ros::Time::now().toSec();
    }
}

void NRSUR10eControl::shutdown() {
    rtde_control->servoStop();
    rtde_control->stopScript();
    std::cout << "Control loop terminated." << std::endl;
}

void NRSUR10eControl::getActualQ() {
    joint_q = rtde_receive->getActualQ();
}

void NRSUR10eControl::controlRobot() {
    if (mode_cmd == 1) {
        // Joint control mode logic
    } else if (mode_cmd == 2) {
        // Posture control logic
    } else if (mode_cmd == 3) {
        // Path control logic
    }
}

void NRSUR10eControl::ftDataCallback(const rtde_handarm::ftsensorMsg::ConstPtr& msg) {
    // Force sensor 데이터를 처리하는 로직
}

void NRSUR10eControl::vrDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // VR 데이터를 처리하는 로직
}

void NRSUR10eControl::cmdModeCallback(const std_msgs::UInt16::ConstPtr& msg) {
    // 제어 모드를 설정하는 로직
}

void NRSUR10eControl::jointCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 조인트 명령을 처리하는 로직
}

void NRSUR10eControl::pbIterCallback(const std_msgs::UInt16::ConstPtr& msg) {
    // 재생(iteration) 명령을 처리하는 로직
}
