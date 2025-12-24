#include "Y2ForceCon/rl_md_variation.hpp"
#include <torch/torch.h>   // torch::kCPU
#include <iostream>

int main() {
    try {
        // dt=0.01f를 명시(생략해도 기본값이면 OK)
        RL_MD_Variation policy(
            "/home/jay/ur10skku_ws/src/y2_ur10skku_control/Y2ForceCon/src/checkpoints/MDVar/naf_policy_script.pt",
            /*threads=*/1,
            torch::kCPU,
            /*dt=*/0.01f
        );

        // 유한차분 상태 초기화
        policy.reset_state();

        // 이제 run 입력은 '원시 상태' (xc, x, Fd, Env_Fext)
        float xc       = 0.0f;
        float x        = 0.1f;
        float Fd       = 10.0f;
        float Env_Fext = 8.0f;

        // (선택) 파생항 워밍업: 초기 1~2회는 파생항이 안정화되도록 더미 호출
        (void)policy.run(xc, x, Fd, Env_Fext);
        xc += 0.001f;  // 임의 변화
        x  += 0.0005f;
        (void)policy.run(xc, x, Fd, Env_Fext);

        // 실제 사용 스텝
        xc += 0.001f;
        x  += 0.0005f;

        auto policy_out = policy.run(xc, x, Fd, Env_Fext); // [mass, damping-scale]
        std::cout << "mass = " << policy_out[0]
                  << ", mass-damper ratio coefficient = " << policy_out[1] << "\n";

        // 필요 시 여기서 D, K 매핑
        float md_ratio = 1000.f;
        float D = static_cast<float>(policy_out[0] * policy_out[1] * md_ratio);
        float K = (Fd > 0.01f) ? 0.f : 5000.f;  // 모드 전환 로직은 상황에 맞게 조정
        std::cout << "desired_mdk = [" << policy_out[0] << ", " << D << ", " << K << "]\n";

    } catch (const c10::Error& e) {
        std::cerr << "Torch error: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Std error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
