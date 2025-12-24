#pragma once
#include <torch/script.h>
#include <vector>
#include <string>

class RL_MD_Variation {
public:
    RL_MD_Variation(const std::string& model_path,
                    int threads,
                    c10::Device device,
                    float dt = 0.01f);

    // 원시 입력: (xc, x, Fd, Env_Fext, md, dd)
    // 내부에서 [xc_ddot, x_dot, x_ddot, (Fd-Env_Fext), rendering_eps] 계산 후 모델에 입력
    std::vector<double> run(float xc, float x, float Fd, float Env_Fext);

    void set_sampling_time(float dt) { dt_ = dt; }
    void reset_state() {
        has1_ = has2_ = false;
        xc_prev1_ = xc_prev2_ = 0.0f;
        x_prev1_  = 0.0f;
        x_dot_prev1_ = 0.0f;
    }

private:
    c10::Device device_;
    torch::jit::script::Module module_;

    // ---- numerical derivative state ----
    float dt_ = 0.01f;
    bool  has1_ = false, has2_ = false;
    float xc_prev1_ = 0.0f, xc_prev2_ = 0.0f; // for xc_ddot
    float x_prev1_  = 0.0f;                   // for x_dot
    float x_dot_prev1_ = 0.0f;                // for x_ddot
};
