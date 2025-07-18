import os
import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import StratifiedKFold, train_test_split
from sklearn.preprocessing import KBinsDiscretizer


def rowwise_cross(a, b):
    return np.cross(a, b)

# %% Data Loading
def load_and_preprocess_tic_data(data_num=2, base_folder="data/TIC/training"):
    """
    TIC 데이터를 불러오고 전처리를 수행합니다.
    Returns:
        raw_vel:       선형/각속도 벡터 (Nx6)
        TWC_comp_FT:   보정된 Force/Torque (Nx6)
        ori_FT:        원본 측정된 Force/Torque (Nx6)
    """
    # 현재 스크립트 위치 기준으로 절대경로 계산
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    # 파일 경로 구성
    file_vel = os.path.join(data_path, 'TIC_input_vel.txt')
    file_TWC_comp = os.path.join(data_path, 'TWC_compft.txt')
    file_ori_ft = os.path.join(data_path, 'TWC_target_ori_ft.txt')

    try:
        # 앞 28줄 제거 (MATLAB의 raw_xxx(1:28,:) = []와 동일)
        raw_vel = np.loadtxt(file_vel, dtype=float, ndmin=2)[28:, :]
        TWC_comp_FT = np.loadtxt(file_TWC_comp, dtype=float, ndmin=2)[28:, :]
        ori_FT = np.loadtxt(file_ori_ft, dtype=float, ndmin=2)[28:, :]
    except Exception as e:
        raise RuntimeError(f"❌ 파일 읽기 실패: {e}")

    print("✅ TIC 데이터 로드 및 전처리 완료")
    print(f" - raw_vel shape: {raw_vel.shape}")
    print(f" - TWC_comp_FT shape: {TWC_comp_FT.shape}")
    print(f" - raw_FT shape: {ori_FT.shape}")

    return raw_vel, TWC_comp_FT, ori_FT

# %% Data Preprocessing
# % Extract RPY directly and force/moment data
def split_tic_components(raw_vel, TWC_comp_FT):
    """
    raw_vel과 TWC_comp_FT 데이터를 분리하여 구성 요소를 반환합니다.

    Returns:
        lin_vel_data:     (Nx3) 선형 속도
        ang_vel_data:     (Nx3) 각속도
        TWC_force_data:   (Nx3) 보상된 힘
        TWC_moment_data:  (Nx3) 보상된 모멘트
    """
    lin_vel_data = raw_vel[:, 0:3]
    ang_vel_data = raw_vel[:, 3:6]
    TWC_force_data = TWC_comp_FT[:, 0:3]
    TWC_moment_data = TWC_comp_FT[:, 3:6]

    print("✅ TIC 구성 요소 분리 완료")
    print(f" - lin_vel_data shape: {lin_vel_data.shape}")
    print(f" - ang_vel_data shape: {ang_vel_data.shape}")
    print(f" - TWC_force_data shape: {TWC_force_data.shape}")
    print(f" - TWC_moment_data shape: {TWC_moment_data.shape}")

    return lin_vel_data, ang_vel_data, TWC_force_data, TWC_moment_data


def run_kalman_and_convert(lin_vel_data, ang_vel_data, dt=0.01):
    """
    Kalman 필터로 선형/각속도 미분 → 가속도 추정 후,
    힘센서 기준으로 변환된 가속도를 계산합니다.
    Returns:
        lin_acc_data, ang_acc_data, converted_acc
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    current_basename = os.path.splitext(os.path.basename(__file__))[0]
    lin_acc_tune_file = os.path.join(current_dir, f"{current_basename}_lin_em_gain.json")
    ang_acc_tune_file = os.path.join(current_dir, f"{current_basename}_ang_em_gain.json")

    # Kalman 필터 적용
    ang_acc_data = kalman_em_diff_estimate(ang_vel_data, dt, ang_acc_tune_file)
    lin_acc_data = kalman_em_diff_estimate(lin_vel_data, dt, lin_acc_tune_file)

    # 기준 변환
    r = np.array([0, 0, -0.185])
    r_mat = np.tile(r, (lin_acc_data.shape[0], 1))

    omega_cross_r = rowwise_cross(ang_vel_data, r_mat)
    omega_cross_omega_cross_r = rowwise_cross(ang_vel_data, omega_cross_r)
    alpha_cross_r = rowwise_cross(ang_acc_data, r_mat)

    converted_acc = lin_acc_data - alpha_cross_r - omega_cross_omega_cross_r

    print("✅ Kalman 추정 및 기준 변환 완료")
    print(f" - lin_acc_data shape: {lin_acc_data.shape}")
    print(f" - ang_acc_data shape: {ang_acc_data.shape}")
    print(f" - converted_acc shape: {converted_acc.shape}")

    return lin_acc_data, ang_acc_data, converted_acc

def kalman_em_diff_estimate(vel_data, dt, filename, max_iter=20):
    """
    EM 기반 Kalman 필터를 이용해 1차 미분값(가속도 등)을 추정한다.
    Q, R을 지정된 경로에서 불러오거나 EM 알고리즘으로 학습하여 저장함.
    최종적으로 필터링된 acc_out (Nx3)을 반환한다.
    """
    import os
    import json
    import numpy as np

    A = np.eye(3)
    H = np.eye(3)
    n = vel_data.shape[0]
    acc_out = np.zeros((n, 3))

    json_path = filename.replace('.mat', '.json')

    if os.path.exists(json_path):
        with open(json_path, 'r') as f:
            json_data = json.load(f)
            Q = np.array(json_data["Q"])
            R = np.array(json_data["R"])
        print(f"[Kalman EM] Loaded Q, R from {json_path}")
    else:
        print(f"[Kalman EM] No Q/R found. Start tuning...")
        Q = np.eye(3) * 1e-4
        R = np.eye(3) * 1e-3
        x = np.zeros((3, 1))
        P = np.eye(3)

        for _ in range(max_iter):
            xs = np.zeros((3, n))
            Ps = np.zeros((3, 3, n))
            Pcs = np.zeros((3, 3, n - 1))
            xf = np.zeros((3, n))
            Pf = np.zeros((3, 3, n))

            x_pred = x.copy()
            P_pred = P.copy()

            # Forward Kalman Filtering
            for k in range(1, n):
                z = (vel_data[k] - vel_data[k - 1]).reshape(3, 1) / dt
                K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
                x = x_pred + K @ (z - H @ x_pred)
                P = (np.eye(3) - K @ H) @ P_pred
                xf[:, k] = x.flatten()
                Pf[:, :, k] = P
                x_pred = A @ x
                P_pred = A @ P @ A.T + Q

            # Backward Smoothing
            xs[:, -1] = xf[:, -1]
            Ps[:, :, -1] = Pf[:, :, -1]
            for k in range(n - 2, -1, -1):
                Ck = Pf[:, :, k] @ A.T @ np.linalg.inv(A @ Pf[:, :, k] @ A.T + Q)
                xs[:, k] = xf[:, k] + Ck @ (xs[:, k + 1] - A @ xf[:, k])
                Ps[:, :, k] = Pf[:, :, k] + Ck @ (Ps[:, :, k + 1] - A @ Pf[:, :, k] @ A.T - Q) @ Ck.T
                Pcs[:, :, k] = Ck @ Ps[:, :, k + 1]

            # M-step: update Q, R
            Q_sum = np.zeros((3, 3))
            R_sum = np.zeros((3, 3))
            for k in range(1, n):
                dz = (vel_data[k] - vel_data[k - 1]).reshape(3, 1) / dt - H @ xs[:, k].reshape(3, 1)
                dx = xs[:, k].reshape(3, 1) - A @ xs[:, k - 1].reshape(3, 1)
                Q_sum += dx @ dx.T + Ps[:, :, k] + A @ Ps[:, :, k - 1] @ A.T \
                         - A @ Pcs[:, :, k - 1].T - Pcs[:, :, k - 1] @ A.T
                R_sum += dz @ dz.T + H @ Ps[:, :, k] @ H.T

            Q = Q_sum / (n - 1)
            R = R_sum / (n - 1)

        # Save Q, R to JSON
        with open(json_path, 'w') as f:
            json.dump({"Q": Q.tolist(), "R": R.tolist()}, f, indent=4)
        print(f"[Kalman EM] Tuned and saved Q, R to {json_path}")

    # Final Kalman Filtering
    x = np.zeros((3, 1))
    P = np.eye(3)
    for k in range(1, n):
        z = (vel_data[k] - vel_data[k - 1]).reshape(3, 1) / dt
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
        x = x + K @ (z - H @ x)
        P = (np.eye(3) - K @ H) @ P
        acc_out[k] = x.flatten()
        x = A @ x
        P = A @ P @ A.T + Q

    return acc_out

def build_input_target_data(converted_acc, ang_acc_data, ang_vel_data,
                             TWC_force_data, TWC_moment_data):
    """
    입력 벡터와 목표 벡터를 구성하는 함수.
    Returns:
        input_data: (N, 9)
        target_data: (N, 6)
    """
    input_data = np.hstack([converted_acc, ang_acc_data, ang_vel_data])
    target_data = np.hstack([TWC_force_data, TWC_moment_data])

    print("✅ 입력 및 목표 데이터 구성 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - target_data shape: {target_data.shape}")

    return input_data, target_data


def normalize_input_target_data(input_data, target_data):
    """
    입력/목표 데이터를 [-1, 1] 범위로 정규화하며, min/max 값을 함께 반환합니다.
    데이터는 saturation 처리 후 정규화됩니다.
    
    Returns:
        input_data_norm, target_data_norm,
        input_min, input_max, target_min, target_max
    """
    # 각 열별 min/max 계산
    input_min = np.min(input_data, axis=0)
    input_max = np.max(input_data, axis=0)
    target_min = np.min(target_data, axis=0)
    target_max = np.max(target_data, axis=0)

    # Saturation 처리
    input_data_saturated = np.clip(input_data, input_min, input_max)
    target_data_saturated = np.clip(target_data, target_min, target_max)

    # 정규화 함수 정의
    def normalize(value, min_val, max_val):
        return 2 * ((value - min_val) / (max_val - min_val)) - 1

    # 정규화 수행
    input_data_norm = normalize(input_data_saturated, input_min, input_max)
    target_data_norm = normalize(target_data_saturated, target_min, target_max)

    print("✅ 입력 및 목표 데이터 정규화 완료")
    print(f" - input_data_norm shape: {input_data_norm.shape}")
    print(f" - target_data_norm shape: {target_data_norm.shape}")

    return input_data_norm, target_data_norm, input_min, input_max, target_min, target_max


# -------------------- Normalize & Denormalize --------------------
def denormalize(value, min_val, max_val):
    return ((value + 1) / 2) * (max_val - min_val)

# -------------------- MLP Model --------------------
class ForcePredictorNet(nn.Module):
    def __init__(self, input_dim=9, hidden_sizes=[6, 6], output_dim=6):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_sizes[0]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[1], output_dim)
        )

    def forward(self, x):
        return self.net(x)

# -------------------- K-Fold Training Function --------------------
def train_ann_kfold(input_data_norm, target_data_norm, converted_acc, target_data,
                    input_min, input_max, target_min, target_max,
                    n_bins=5, k_folds=5):

    bin_data = converted_acc[:, 1].reshape(-1, 1)
    est = KBinsDiscretizer(n_bins=n_bins, encode='ordinal', strategy='quantile')
    bin_idx = est.fit_transform(bin_data).astype(int).flatten()

    skf = StratifiedKFold(n_splits=k_folds, shuffle=True, random_state=42)

    mse_scores = []
    Y_pred_norm_all = np.zeros_like(target_data_norm)

    for fold, (train_val_idx, test_idx) in enumerate(skf.split(input_data_norm, bin_idx)):
        print(f"Processing Fold {fold+1}/{k_folds}...")

        train_idx, val_idx = train_test_split(
            train_val_idx, test_size=0.2, stratify=bin_idx[train_val_idx], random_state=42
        )

        # Convert to torch
        X_train = torch.tensor(input_data_norm[train_idx], dtype=torch.float32)
        Y_train = torch.tensor(target_data_norm[train_idx], dtype=torch.float32)
        X_val = torch.tensor(input_data_norm[val_idx], dtype=torch.float32)
        Y_val = torch.tensor(target_data_norm[val_idx], dtype=torch.float32)
        X_test = torch.tensor(input_data_norm[test_idx], dtype=torch.float32)
        Y_test = torch.tensor(target_data_norm[test_idx], dtype=torch.float32)

        model = ForcePredictorNet()
        optimizer = optim.Adam(model.parameters(), lr=0.001)
        criterion = nn.MSELoss()

        # Early stopping
        best_loss = np.inf
        patience = 0
        max_patience = 40
        for epoch in range(6000):
            model.train()
            optimizer.zero_grad()
            output = model(X_train)
            loss = criterion(output, Y_train)
            loss.backward()
            optimizer.step()

            # Validation
            model.eval()
            with torch.no_grad():
                val_output = model(X_val)
                val_loss = criterion(val_output, Y_val).item()

            if val_loss < best_loss:
                best_loss = val_loss
                best_model_state = model.state_dict()
                patience = 0
            else:
                patience += 1
                if patience >= max_patience:
                    break

        # Test with best model
        model.load_state_dict(best_model_state)
        model.eval()
        with torch.no_grad():
            pred_output_norm = model(X_test).cpu().numpy()
            Y_pred_norm_all[test_idx, :] = pred_output_norm

            pred_output = denormalize(pred_output_norm, target_min, target_max)
            Y_test_denorm = denormalize(Y_test.numpy(), target_min, target_max)
            mse = np.mean((pred_output - Y_test_denorm) ** 2)
            mse_scores.append(mse)

    pred_output_all = denormalize(Y_pred_norm_all, target_min, target_max)
    pred_force = pred_output_all[:, :3]
    pred_moment = pred_output_all[:, 3:6]
    comp_force = target_data - pred_output_all

    print(f"✅ Average MSE (Force Compensation): {np.mean(mse_scores):.6f}")
    return pred_output_all, comp_force, mse_scores




# def save_ann_weights_biases_to_json(model, input_min, input_max, target_min, target_max,
#                                     filename="TIC_ann_weights_biases.json"):
#     import os
#     import json
#     import torch.nn as nn

#     # 저장 경로 구성
#     save_dir = os.path.join(os.path.dirname(__file__), 'json', 'TIC')
#     os.makedirs(save_dir, exist_ok=True)
#     save_path = os.path.join(save_dir, filename)

#     # 모델 내 모든 Linear 계층 추출
#     layers = []
#     for layer in model.modules():
#         if isinstance(layer, nn.Linear):
#             layers.append(layer)

#     if len(layers) != 3:
#         raise ValueError(f"Expected 3 Linear layers in model (2 hidden + 1 output), but got {len(layers)}")

#     # 각 레이어에서 가중치와 편향 추출
#     weights1 = layers[0].weight.detach().cpu().numpy().tolist()
#     bias1 = layers[0].bias.detach().cpu().numpy().tolist()

#     weights2 = layers[1].weight.detach().cpu().numpy().tolist()
#     bias2 = layers[1].bias.detach().cpu().numpy().tolist()

#     weightsOut = layers[2].weight.detach().cpu().numpy().tolist()
#     biasOut = layers[2].bias.detach().cpu().numpy().tolist()

#     # JSON 구조 생성
#     data_to_save = {
#         "weights1": weights1,
#         "bias1": bias1,
#         "weights2": weights2,
#         "bias2": bias2,
#         "weightsOut": weightsOut,
#         "biasOut": biasOut,
#         "input_min": input_min.tolist() if hasattr(input_min, 'tolist') else input_min,
#         "input_max": input_max.tolist() if hasattr(input_max, 'tolist') else input_max,
#         "target_min": target_min.tolist() if hasattr(target_min, 'tolist') else target_min,
#         "target_max": target_max.tolist() if hasattr(target_max, 'tolist') else target_max,
#     }

#     # JSON 파일로 저장
#     with open(save_path, 'w') as json_file:
#         json.dump(data_to_save, json_file, indent=2)

#     print(f"✅ ANN 가중치 및 정규화 파라미터 저장 완료: {save_path}")


def visualize_force_moment_prediction(
    TWC_force_data, TWC_moment_data,
    pred_force, pred_moment,
    comp_force, comp_moment,
    time_profile
):
    component_names = ['Fx', 'Fy', 'Fz']
    moment_names = ['Mx', 'My', 'Mz']

    # Force Prediction vs Raw Force
    plt.figure("Force Prediction Comparison", figsize=(10, 8))
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_profile, TWC_force_data[:, i], 'r', linewidth=0.6, label='Raw Force')
        plt.plot(time_profile, pred_force[:, i], 'k', linewidth=0.8, label='Predicted Force')
        plt.title(f'{component_names[i]} Prediction vs Raw')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.legend()
        plt.grid(True)

    # Force Compensation
    plt.figure("Force Compensation Effect", figsize=(10, 8))
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_profile, TWC_force_data[:, i], 'r', linewidth=0.8, label='Uncompensated Force')
        plt.plot(time_profile, comp_force[:, i], 'k', linewidth=0.6, label='Compensated Force')
        plt.title(f'{component_names[i]} Compensation Result')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.legend()
        plt.grid(True)

    # Moment Prediction vs Raw Moment
    plt.figure("Moment Prediction Comparison", figsize=(10, 8))
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_profile, TWC_moment_data[:, i], 'r', linewidth=0.6, label='Raw Moment')
        plt.plot(time_profile, pred_moment[:, i], 'k', linewidth=0.8, label='Predicted Moment')
        plt.title(f'{moment_names[i]} Prediction vs Raw')
        plt.xlabel('Time (s)')
        plt.ylabel('Moment (Nm)')
        plt.legend()
        plt.grid(True)

    # Moment Compensation
    plt.figure("Moment Compensation Effect", figsize=(10, 8))
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_profile, TWC_moment_data[:, i], 'r', linewidth=0.8, label='Uncompensated Moment')
        plt.plot(time_profile, comp_moment[:, i], 'k', linewidth=0.6, label='Compensated Moment')
        plt.title(f'{moment_names[i]} Compensation Result')
        plt.xlabel('Time (s)')
        plt.ylabel('Moment (Nm)')
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    print("✅ TIC KalmanEM Node 시작")

    # Load & Preprocess
    raw_vel, TWC_comp_FT, ori_FT = load_and_preprocess_tic_data(data_num=2)
    lin_vel_data, ang_vel_data, TWC_force_data, TWC_moment_data = \
        split_tic_components(raw_vel, TWC_comp_FT)
    lin_acc_data, ang_acc_data, converted_acc = \
        run_kalman_and_convert(lin_vel_data, ang_vel_data)
    input_data, target_data = build_input_target_data(
        converted_acc, ang_acc_data, ang_vel_data, TWC_force_data, TWC_moment_data
    )
    input_data_norm, target_data_norm, input_min, input_max, target_min, target_max = \
        normalize_input_target_data(input_data, target_data)

    # Train & Evaluate
    best_model, pred_output_all, _ = train_ann_kfold(
        input_data_norm, target_data_norm, converted_acc, target_data,
        input_min, input_max, target_min, target_max
    )

    # 추론 결과 분리
    pred_force = pred_output_all[:, :3]
    pred_moment = pred_output_all[:, 3:6]
    time_profile = np.arange(TWC_force_data.shape[0]) * 0.01

    # Save Trained Weights to JSON (선택)
    # save_ann_weights_biases_to_json(
    #     model=best_model,
    #     input_min=input_min,
    #     input_max=input_max,
    #     target_min=target_min,
    #     target_max=target_max,
    #     filename="nrs_dlc/json/TIC/TIC_ann_weights_biases.json"
    # )

    # Visualization
    visualize_force_moment_prediction(
        TWC_force_data, TWC_moment_data,
        pred_force, pred_moment,
        TWC_force_data - pred_force,
        TWC_moment_data - pred_moment,
        time_profile
    )





if __name__ == '__main__':
    main()