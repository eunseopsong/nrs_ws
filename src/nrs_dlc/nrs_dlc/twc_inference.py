import os
import numpy as np
from sklearn.model_selection import StratifiedKFold, train_test_split

# %% Helper Functions

def get_current_dir():
    try:
        return os.path.dirname(__file__)
    except NameError:
        return os.getcwd()

def normalize(value, min_val, max_val):
    return 2 * (value - min_val) / (max_val - min_val) - 1

def denormalize(value, min_val, max_val):
    return 0.5 * (value + 1) * (max_val - min_val) + min_val

# %% Data Loading and Normalization

def load_and_preprocess_data(data_num=2, base_folder="data/training"):
    current_dir = get_current_dir()
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    file_ft = os.path.join(data_path, 'Ori_ft.txt')
    file_rot = os.path.join(data_path, 'Ori_rot.txt')

    try:
        raw_ft = np.loadtxt(file_ft)
        raw_rot = np.loadtxt(file_rot)
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {e}")
        return None, None, None, None, None, None, None, None

    raw_ft = raw_ft[1:, :]
    raw_rot = raw_rot[1:, :]

    input_data = raw_rot[:, 0:9]
    force_data = raw_ft[:, 0:3]
    moment_data = raw_ft[:, 3:6]
    target_data = np.hstack([force_data, moment_data])

    input_min = np.min(input_data, axis=0)
    input_max = np.max(input_data, axis=0)
    input_data_saturated = np.clip(input_data, input_min, input_max)

    target_min = np.min(target_data, axis=0)
    target_max = np.max(target_data, axis=0)
    target_data_saturated = np.clip(target_data, target_min, target_max)

    input_data_norm = normalize(input_data_saturated, input_min, input_max)
    target_data_norm = normalize(target_data_saturated, target_min, target_max)

    print("✅ 데이터 로드 및 전처리 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - force_data shape: {force_data.shape}")
    print(f" - moment_data shape: {moment_data.shape}")
    print(f" - target_data shape: {target_data.shape}")
    print(f" - input_data_norm shape: {input_data_norm.shape}")
    print(f" - target_data_norm shape: {target_data_norm.shape}")

    return input_data_norm, target_data_norm, input_min, input_max, target_min, target_max, input_data, target_data

# %% Stratified K-Fold Cross Validation

def run_kfold_validation(input_data_norm, target_data_norm, k_folds=8, n_bins=8):
    bin_edges = np.histogram_bin_edges(target_data_norm[:, 2], bins=n_bins)
    bin_idx = np.digitize(target_data_norm[:, 2], bins=bin_edges)

    skf = StratifiedKFold(n_splits=k_folds, shuffle=True, random_state=42)

    for fold, (train_idx, test_idx) in enumerate(skf.split(input_data_norm, bin_idx), 1):
        print(f"\n📂 Processing Fold {fold}/{k_folds}...")

        # Stratified validation split inside training set
        bin_train_idx = bin_idx[train_idx]
        train_sub_idx, val_sub_idx = train_test_split(
            train_idx, test_size=0.2, stratify=bin_train_idx, random_state=42)

        X_train = input_data_norm[train_sub_idx].T
        Y_train = target_data_norm[train_sub_idx].T

        X_val = input_data_norm[val_sub_idx].T
        Y_val = target_data_norm[val_sub_idx].T

        X_test = input_data_norm[test_idx].T
        Y_test = target_data_norm[test_idx].T

        print(f" - X_train shape: {X_train.shape}")
        print(f" - Y_train shape: {Y_train.shape}")
        print(f" - X_val shape: {X_val.shape}")
        print(f" - Y_val shape: {Y_val.shape}")
        print(f" - X_test shape: {X_test.shape}")
        print(f" - Y_test shape: {Y_test.shape}")

# %% Main Entry Point

def main():
    print("✅ TWC Inference Node 시작")
    input_data_norm, target_data_norm, input_min, input_max, target_min, target_max, input_data, target_data = load_and_preprocess_data(data_num=2)

    if input_data_norm is not None and target_data_norm is not None:
        run_kfold_validation(input_data_norm, target_data_norm)

if __name__ == '__main__':
    main()
