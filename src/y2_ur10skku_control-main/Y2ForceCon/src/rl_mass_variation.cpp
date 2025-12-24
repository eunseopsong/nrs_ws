#include "Y2ForceCon/rl_mass_variation.hpp"
#include <torch/torch.h>       // TensorOptions
#include <stdexcept>
#include <sstream>

namespace {
constexpr float kPI = 3.14159265358979323846f;
}

RL_Mass_Variation::RL_Mass_Variation(const std::string& model_path,
                                     int threads,
                                     c10::Device device,
                                     float dt)
: device_(device), dt_(dt)
{
    if (dt_ <= 0.0f) {
        throw std::invalid_argument("RL_Mass_Variation: dt must be > 0");
    }
    torch::set_num_threads(std::max(1, threads));
    module_ = torch::jit::load(model_path, device_);
    module_.eval();
}

void RL_Mass_Variation::set_acc_lpf_weight(float w) {
    w_acc_ = std::clamp(w, 0.0f, 1.0f);
}

void RL_Mass_Variation::set_acc_lpf_cutoff(float fc_hz) {
    const float fc  = std::max(fc_hz, 1e-4f);
    const float tau = 1.0f / (2.0f * kPI * fc);
    float w = 1.0f - std::exp(-dt_ / tau);
    w_acc_ = std::clamp(w, 0.0f, 1.0f);
}

void RL_Mass_Variation::reset() {
    has1_ = false;
    has2_ = false;
    xc_prev1_ = xc_prev2_ = 0.0f;
    x_prev1_  = 0.0f;
    x_dot_prev1_ = 0.0f;

    acc_init_ = false;
    x_ddot_f_  = 0.0f;
    xc_ddot_f_ = 0.0f;
}

void RL_Mass_Variation::reset_state(float xc_seed, float x_seed, float x_dot_seed) {
    // 다음 run()에서 "2번째 샘플" 분기부터 자연스럽게 진행되도록 씨딩
    has1_ = true;     // 첫 샘플은 보유
    has2_ = false;    // 두 번째 샘플을 기다리는 상태
    xc_prev1_ = xc_seed;
    xc_prev2_ = xc_seed;   // 안전 차원에서 동일값
    x_prev1_  = x_seed;
    x_dot_prev1_ = x_dot_seed;

    // LPF는 다음 스텝에서 raw로 씨딩되도록 초기화
    acc_init_  = false;
    x_ddot_f_  = 0.0f;
    xc_ddot_f_ = 0.0f;
}

float RL_Mass_Variation::run(float xc, float x, float Fd, float Env_Fext) {
    torch::NoGradGuard _ng;

    // ----------- numerical derivatives -----------
    float x_dot = 0.0f, x_ddot = 0.0f, xc_ddot = 0.0f;

    if (!has1_) {
        // 1st sample: 아직 미분 불가 → 0 유지, 상태 저장
        xc_prev1_ = xc;
        x_prev1_  = x;
        x_dot_prev1_ = 0.0f;
        has1_ = true;
    } else if (!has2_) {
        // 2nd sample: 1차 미분 초기화
        float x_dot_new = (x - x_prev1_) / dt_;
        x_dot_prev1_ = x_dot_new;

        xc_prev2_ = xc_prev1_;
        xc_prev1_ = xc;
        x_prev1_  = x;
        has2_ = true;
    } else {
        // 이후: 정상 계산
        x_dot   = (x - x_prev1_) / dt_;
        x_ddot  = (x_dot - x_dot_prev1_) / dt_;
        xc_ddot = (xc - 2.0f*xc_prev1_ + xc_prev2_) / (dt_ * dt_);

        // ---- 1차 IIR LPF(EMA) 가속도 필터 ----
        const float w = w_acc_;
        if (!acc_init_) {
            // 첫 필터 스텝은 raw로 씨딩
            x_ddot_f_  = x_ddot;
            xc_ddot_f_ = xc_ddot;
            acc_init_  = true;
        } else {
            x_ddot_f_  = w * x_ddot  + (1.0f - w) * x_ddot_f_;
            xc_ddot_f_ = w * xc_ddot + (1.0f - w) * xc_ddot_f_;
        }
        // 필터 출력 사용
        x_ddot  = x_ddot_f_;
        xc_ddot = xc_ddot_f_;

        // 상태 업데이트
        x_dot_prev1_ = x_dot;
        x_prev1_  = x;
        xc_prev2_ = xc_prev1_;
        xc_prev1_ = xc;
    }

    const float f_err = Fd - Env_Fext;

    // ----------- NN 입력: [xc_ddot, x_dot, x_ddot, f_err] -----------
    auto s = torch::tensor({xc_ddot, x_dot, x_ddot, f_err},
                           torch::TensorOptions().dtype(torch::kFloat32))
                 .unsqueeze(0)       // [1,4]
                 .to(device_);

    // ----------- forward (act() 우선, 없으면 forward()) -----------
    torch::IValue iv;
    try {
        iv = module_.get_method("act")({s});
    } catch (const c10::Error&) {
        iv = module_.forward({s});
    }

    torch::Tensor a;
    if (iv.isTensor()) {
        a = iv.toTensor();
    } else if (iv.isTuple()) {
        for (const auto& e : iv.toTuple()->elements()) {
            if (e.isTensor()) { a = e.toTensor(); break; }
        }
        if (!a.defined()) throw std::runtime_error("act()/forward() returned a tuple without a Tensor.");
    } else {
        throw std::runtime_error("Unsupported return type from act()/forward().");
    }

    a = a.to(torch::kCPU).contiguous().reshape({-1}); // [A]
    TORCH_CHECK(a.numel() >= 1, "act()/forward() must return at least 1 scalar, got shape ", a.sizes());

    // 첫 번째 액션 반환
    return a[0].item<float>();
}
