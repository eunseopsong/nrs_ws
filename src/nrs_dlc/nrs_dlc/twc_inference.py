import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import StratifiedKFold, train_test_split
import json

# %% Normalize Functions
def normalize(value, min_val, max_val):
    return 2 * ((value - min_val) / (max_val - min_val)) - 1

def denormalize(value, min_val, max_val):
    return ((value + 1) / 2) * (max_val - min_val) + min_val

# %% Data Loading & Preprocessing
def load_and_preprocess_data(data_num=2, base_folder="data/TWC/training"):
    current_dir = os.path.dirname(__file__)
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    file_rot = os.path.join(data_path, 'Ori_rot.txt')
    file_ft = os.path.join(data_path, 'Ori_ft.txt')

    try:
        raw_rot = np.loadtxt(file_rot)[1:, :]
        raw_ft = np.loadtxt(file_ft)[1:, :]
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {e}")
        return None, None, None, None, None, None, None, None

    input_data = raw_rot[:, 0:9]
    force_data = raw_ft[:, 0:3]
    moment_data = raw_ft[:, 3:6]
    target_data = np.hstack([force_data, moment_data])

    input_min = np.min(input_data, axis=0)
    input_max = np.max(input_data, axis=0)
    target_min = np.min(target_data, axis=0)
    target_max = np.max(target_data, axis=0)

    input_data_sat = np.clip(input_data, input_min, input_max)
    target_data_sat = np.clip(target_data, target_min, target_max)

    input_data_norm = normalize(input_data_sat, input_min, input_max)
    target_data_norm = normalize(target_data_sat, target_min, target_max)

    print("✅ 데이터 로드 및 전처리 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - target_data shape: {target_data.shape}")
    print(f" - input_data_norm shape: {input_data_norm.shape}")
    print(f" - target_data_norm shape: {target_data_norm.shape}")

    return input_data_norm, target_data_norm, input_min, input_max, target_min, target_max, input_data, target_data

# %% ANN Definition
class TWCNet(nn.Module):
    def __init__(self, input_size=9, hidden_sizes=[6, 6], output_size=6):
        super(TWCNet, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(input_size, hidden_sizes[0]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[1], output_size),
            nn.Tanh()
        )

    def forward(self, x):
        return self.model(x)

# %% ANN Training
def train_ann(X_train, Y_train, X_val, Y_val, epochs=6000, goal=1e-8):
    print("\n🧠 Starting ANN Training")
    X_train_tensor = torch.FloatTensor(X_train.T)
    Y_train_tensor = torch.FloatTensor(Y_train.T)
    X_val_tensor = torch.FloatTensor(X_val.T)
    Y_val_tensor = torch.FloatTensor(Y_val.T)

    model = TWCNet()
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

    for epoch in range(epochs):
        model.train()
        optimizer.zero_grad()
        output = model(X_train_tensor)
        loss = criterion(output, Y_train_tensor)
        loss.backward()
        optimizer.step()

        if epoch % 500 == 0 or loss.item() < goal:
            val_loss = criterion(model(X_val_tensor), Y_val_tensor).item()
            print(f"Epoch {epoch:5d} | Train Loss: {loss.item():.6f} | Val Loss: {val_loss:.6f}")
        if loss.item() < goal:
            break

    print("✅ ANN Training Finished")
    return model

# %% Save to JSON
def save_model_to_json(model, input_min, input_max, target_min, target_max):
    json_dir = os.path.expanduser("~/nrs_ws/src/nrs_dlc/TWC/json")
    os.makedirs(json_dir, exist_ok=True)
    filename = os.path.join(json_dir, "ann_weights_biases.json")

    state_dict = model.state_dict()
    weights_biases = {
        'weights1': state_dict['model.0.weight'].cpu().numpy().tolist(),
        'bias1': state_dict['model.0.bias'].cpu().numpy().tolist(),
        'weights2': state_dict['model.2.weight'].cpu().numpy().tolist(),
        'bias2': state_dict['model.2.bias'].cpu().numpy().tolist(),
        'weightsOut': state_dict['model.4.weight'].cpu().numpy().tolist(),
        'biasOut': state_dict['model.4.bias'].cpu().numpy().tolist(),
        'input_min': input_min.tolist(),
        'input_max': input_max.tolist(),
        'target_min': target_min.tolist(),
        'target_max': target_max.tolist()
    }

    with open(filename, 'w') as f:
        json.dump(weights_biases, f, indent=4)
    print(f"✅ Weights and biases saved to: {filename}")

# %% K-Fold Cross Validation
def run_kfold_validation(input_data_norm, target_data_norm,
                         input_min, input_max, target_min, target_max,
                         n_bins=8, k_folds=8):
    print("\n🔍 Running Stratified K-Fold Cross Validation")

    target_fz = target_data_norm[:, 2]
    bins = np.linspace(np.min(target_fz), np.max(target_fz), n_bins + 1)
    bin_idx = np.digitize(target_fz, bins[:-1], right=False)

    valid_mask = np.array([np.sum(bin_idx == i) >= 2 for i in np.unique(bin_idx)])
    bin_mask = np.isin(bin_idx, np.unique(bin_idx)[valid_mask])

    input_data_filtered = input_data_norm[bin_mask]
    target_data_filtered = target_data_norm[bin_mask]
    bin_idx_filtered = bin_idx[bin_mask]

    print(f"✅ Total filtered samples: {len(input_data_filtered)}")
    print(f"✅ Valid bin counts: {np.sum(valid_mask)} / {n_bins}")

    skf = StratifiedKFold(n_splits=k_folds, shuffle=True, random_state=42)

    mse_scores = []
    Y_pred_norm_all = np.zeros_like(target_data_filtered)

    for fold, (trainval_idx, test_idx) in enumerate(skf.split(input_data_filtered, bin_idx_filtered), start=1):
        print(f"\n📂 Processing Fold {fold}/{k_folds}...")

        bin_trainval = bin_idx_filtered[trainval_idx]
        try:
            train_idx, val_idx = train_test_split(trainval_idx, test_size=0.2, stratify=bin_trainval, random_state=42)
        except:
            print("⚠️ Warning: Some classes too small, using random split instead.")
            train_idx, val_idx = train_test_split(trainval_idx, test_size=0.2, random_state=42)

        X_train = input_data_filtered[train_idx].T
        Y_train = target_data_filtered[train_idx].T
        X_val = input_data_filtered[val_idx].T
        Y_val = target_data_filtered[val_idx].T
        X_test = input_data_filtered[test_idx].T
        Y_test = target_data_filtered[test_idx].T

        model = train_ann(X_train, Y_train, X_val, Y_val)

        model.eval()
        with torch.no_grad():
            X_test_tensor = torch.FloatTensor(X_test.T)
            pred_output_norm = model(X_test_tensor).numpy()

        Y_pred_norm_all[test_idx, :] = pred_output_norm
        pred_output = denormalize(pred_output_norm, target_min, target_max)
        Y_test_denorm = denormalize(Y_test.T, target_min, target_max)

        mse = np.mean((pred_output - Y_test_denorm) ** 2)
        mse_scores.append(mse)
        print(f"📉 Fold {fold} MSE: {mse:.6f}")

    pred_output_all = denormalize(Y_pred_norm_all, target_min, target_max)
    pred_force = pred_output_all[:, :3]
    pred_moment = pred_output_all[:, 3:]
    comp_force = target_data_filtered[:, :3] - pred_force
    comp_moment = target_data_filtered[:, 3:] - pred_moment

    print("\n📊 MSE Scores per Fold:")
    print(np.round(mse_scores, 6))
    print(f"📈 Average MSE: {np.mean(mse_scores):.6f}")

    save_model_to_json(model, input_min, input_max, target_min, target_max)

# %% Main Entry Point
def main(run_kfold=True, run_ann_training=True):
    print("✅ TWC Inference Node 시작")

    input_data_norm, target_data_norm, input_min, input_max, target_min, target_max, *_ = \
        load_and_preprocess_data(data_num=2)

    if input_data_norm is None or target_data_norm is None:
        return

    if run_kfold:
        run_kfold_validation(
            input_data_norm, target_data_norm,
            input_min, input_max, target_min, target_max
        )

    if run_ann_training:
        split_idx = int(0.8 * len(input_data_norm))
        X_train = input_data_norm[:split_idx].T
        Y_train = target_data_norm[:split_idx].T
        X_val = input_data_norm[split_idx:].T
        Y_val = target_data_norm[split_idx:].T
        train_ann(X_train, Y_train, X_val, Y_val)

if __name__ == '__main__':
    main(run_kfold=True, run_ann_training=True)
