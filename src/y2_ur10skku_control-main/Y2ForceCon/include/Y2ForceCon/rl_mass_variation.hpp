#pragma once
#include <torch/script.h>      // torch::jit::Module
#include <string>
#include <algorithm>           // std::clamp
#include <cmath>               // std::exp
#include <c10/core/Device.h>

class RL_Mass_Variation {
public:
    RL_Mass_Variation(const std::string& model_path,
                      int threads,
                      c10::Device device,
                      float dt);

    // 1 step 실행: 입력 (xc, x, Fd, Env_Fext) → 출력: 첫 번째 액션 스칼라
    float run(float xc, float x, float Fd, float Env_Fext);

    // --- 가속도 LPF 설정 ---
    // w∈[0,1], 작을수록 더 부드럽게(느리게). 0 → 고정, 1 → 필터 없음
    void set_acc_lpf_weight(float w);

    // 컷오프 주파수(Hz) 지정 → w 자동 계산
    //   tau = 1/(2πfc),  w = 1 - exp(-dt/tau)
    void set_acc_lpf_cutoff(float fc_hz);

    // 상태 초기화 (비씨딩)
    void reset();

    // 과거 이름과 호환 (별칭)
    void reset_state() { reset(); }

    // (선택) 현재 값으로 씨딩해서 재시작 (한 스텝 딜레이 최소화)
    void reset_state(float xc_seed, float x_seed, float x_dot_seed = 0.0f);

private:
    // 디바이스/샘플링/모듈
    c10::Device device_;
    float dt_{0.0f};
    torch::jit::Module module_;

    // 수치미분 이력
    bool  has1_{false};
    bool  has2_{false};
    float xc_prev1_{0.0f};
    float xc_prev2_{0.0f};
    float x_prev1_{0.0f};
    float x_dot_prev1_{0.0f};

    // 가속도 LPF 상태
    float w_acc_{1.0f};   // 기본 가중치 (작을수록 더 많이 평활화, 더 느림)
    bool  acc_init_{false};
    float x_ddot_f_{0.0f};
    float xc_ddot_f_{0.0f};
};
