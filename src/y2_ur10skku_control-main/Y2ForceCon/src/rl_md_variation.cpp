#include "Y2ForceCon/rl_md_variation.hpp"
#include <torch/torch.h>
#include <algorithm>
#include <stdexcept>

RL_MD_Variation::RL_MD_Variation(const std::string& model_path,
                                 int threads,
                                 c10::Device device,
                                 float dt)
: device_(device), dt_(dt)
{
    torch::set_num_threads(std::max(1, threads));
    module_ = torch::jit::load(model_path, device_);
    module_.eval();
}

std::vector<double> RL_MD_Variation::run(float xc, float x, float Fd, float Env_Fext) {
    torch::NoGradGuard _ng;

    // ----------- numerical derivatives -----------
    float x_dot = 0.0f, x_ddot = 0.0f, xc_dot = 0.0f, xc_ddot = 0.0f;

    if (!has1_) {
        // 1st sample: 파생량 계산 불가 -> 상태만 적재
        xc_prev1_ = xc;
        x_prev1_  = x;
        x_dot_prev1_ = 0.0f;
        has1_ = true;
    } else if (!has2_) {
        // 2nd sample: x_dot 초기화
        float x_dot_new = (x - x_prev1_) / dt_;
        x_dot_prev1_ = x_dot_new;

        xc_prev2_ = xc_prev1_;
        xc_prev1_ = xc;
        x_prev1_  = x;
        has2_ = true;
    } else {
        // 이후 정상 계산
        x_dot  = (x - x_prev1_) / dt_;
        x_ddot = (x_dot - x_dot_prev1_) / dt_;

        xc_dot = (xc - xc_prev1_) / dt_;
        xc_ddot = (xc - 2.0f*xc_prev1_ + xc_prev2_) / (dt_ * dt_);

        // 상태 업데이트
        x_dot_prev1_ = x_dot;
        x_prev1_  = x;
        xc_prev2_ = xc_prev1_;
        xc_prev1_ = xc;
    }

    float f_err = Fd - Env_Fext;


    // ----------- build network input: [xc_ddot, x_dot, x_ddot, f_err] -----------
    auto s = torch::tensor({xc_ddot, x_dot, x_ddot, f_err},
                           torch::TensorOptions().dtype(torch::kFloat32))
                 .unsqueeze(0)       // [1,5]
                 .to(device_);

    // ----------- forward (Tensor / Tuple & [1,A]/[A] 대응) -----------
    torch::IValue iv = module_.get_method("act")({s});
    torch::Tensor a;
    if (iv.isTensor()) {
        a = iv.toTensor();
    } else if (iv.isTuple()) {
        const auto& elems = iv.toTuple()->elements();
        for (const auto& e : elems) {
            if (e.isTensor()) { a = e.toTensor(); break; }
        }
        if (!a.defined()) {
            throw std::runtime_error("act() returned a tuple without a Tensor.");
        }
    } else {
        throw std::runtime_error("Unsupported return type from act().");
    }

    // [1,2] 또는 [2] -> [2] 로 평탄화
    a = a.to(torch::kCPU).contiguous().reshape({-1});
    TORCH_CHECK(a.numel() >= 2, "act() must return at least 2 scalars, got shape ", a.sizes());

    // 두 액션: [0]=mass, [1]=damping-scale(혹은 ratio)
    std::vector<double> output(2);
    output[0] = static_cast<double>(a[0].item<float>());
    output[1] = static_cast<double>(a[1].item<float>());
    return output;
}
