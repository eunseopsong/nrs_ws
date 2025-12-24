#include "Y2ForceCon/rl_mass_variation.hpp"
#include <torch/torch.h>   // torch::kCPU 사용 시 권장
#include <iostream>

int main() {
    try {
        // dt=0.01f (100 Hz). 만약 ctor에 dt 기본값이 있다면 4번째 인자는 생략 가능.
        RL_Mass_Variation policy(
            "/home/jay/ur10skku_ws/src/y2_ur10skku_control/Y2ForceCon/src/checkpoints/MassVar/naf_policy_script.pt",
            /*threads=*/1,
            torch::kCPU,
            /*dt=*/0.01f
        );

        // 유한차분 상태 초기화
        policy.reset_state();

        // 예시 입력 (이제는 '원시' 상태: xc, x, Fd, Env_Fext)
        float xc        = 0.0f;
        float x         = 0.1f;
        float Fd        = 10.0f;
        float Env_Fext  = 8.0f;

        // (선택) 파생항 워밍업: 초기 2스텝 정도는 파생항이 안정화되도록 더 호출
        (void)policy.run(xc, x, Fd, Env_Fext);
        xc += 0.001f;  // 임의로 약간의 변화
        x  += 0.0005f;
        (void)policy.run(xc, x, Fd, Env_Fext);

        // 실제 사용 스텝
        xc += 0.001f;
        x  += 0.0005f;
        Env_Fext = 8.0f;  // 예: 외력 업데이트
        float mass = policy.run(xc, x, Fd, Env_Fext);
        std::cout << "mass = " << mass << "\n";

        // 필요 시 여기서 D, K 매핑
        float md_ratio = 1000.f;
        float D = mass * md_ratio;
        float K = (Fd > 0.01f) ? 0.f : 5000.f;  // 모드 전환 로직은 상황에 맞게 조정
        std::cout << "desired_mdk = [" << mass << ", " << D << ", " << K << "]\n";

    } catch (const c10::Error& e) {
        std::cerr << "Torch error: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Std error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
