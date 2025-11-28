#ifndef FUNC_UR10E_MAIN_H_
#define FUNC_UR10E_MAIN_H_

// func_ur10e_main.h
//
// JointControl.cpp / func_ur10e_main.cpp 에서 공용으로 쓰는
// 유틸 함수들의 선언을 모아둔 헤더.
//
//  - set_status
//  - trim_path
//  - yaml_get_path
//  - ensure_parent_dir
//  - readDoublesFromStdin
//
// JointControl.h 쪽에서는 단순히 이 헤더만 include 하면 됨.

#include <string>
#include <cstdio>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

// -----------------------------------------------------------------------------
// 상태 문자열 저장 유틸
//  - dst: char 배열 (예: message_status)
//  - s  : C-string (nullptr 허용, 그 경우 빈 문자열로 처리)
//  - 템플릿이므로 반드시 헤더에서 정의해야 함
// -----------------------------------------------------------------------------
template <size_t N>
inline void set_status(char (&dst)[N], const char* s) {
  std::snprintf(dst, N, "%s", s ? s : "");
}

// -----------------------------------------------------------------------------
// 문자열 앞뒤 공백/개행 제거
//  - 구현은 func_ur10e_main.cpp 에서 제공
// -----------------------------------------------------------------------------
std::string trim_path(std::string s);

// -----------------------------------------------------------------------------
// YAML에서 파일 경로 안전 취득
//
//  - root   : YAML 루트 (예: NRS_recording 노드)
//  - key    : "hand_g_recording" 같은 키
//  - logger : node_->get_logger()
//  - 반환값 : 정리된 경로 문자열 (오류인 경우 "")
//
//  구현은 func_ur10e_main.cpp 에서 제공
// -----------------------------------------------------------------------------
std::string yaml_get_path(const YAML::Node& root,
                          const char* key,
                          const rclcpp::Logger& logger);

// -----------------------------------------------------------------------------
// 파일 경로의 부모 디렉토리 생성 보장
//
//  - filepath: "/home/.../log/exp1.txt" 같은 경로
//  - logger  : node_->get_logger()
//
//  구현은 func_ur10e_main.cpp 에서 제공
// -----------------------------------------------------------------------------
void ensure_parent_dir(const std::string& filepath,
                       const rclcpp::Logger& logger);

// -----------------------------------------------------------------------------
// 터미널에서 double n개 입력 받기
//
//  - prompt: "Enter 6 joint angles [deg]: " 같은 프롬프트 문자열
//  - n     : 필요한 double 개수
//  - out   : 길이 n 이상의 배열 포인터
//
//  성공 시 true, 실패 시 false
//  (cin 에러 클리어 + 버퍼 플러시까지 포함)
//
//  구현은 func_ur10e_main.cpp 에서 제공
// -----------------------------------------------------------------------------
bool readDoublesFromStdin(const char* prompt, int n, double* out);

#endif  // FUNC_UR10E_MAIN_H_
