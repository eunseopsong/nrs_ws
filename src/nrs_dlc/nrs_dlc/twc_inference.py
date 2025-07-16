import numpy as np
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import StratifiedKFold, train_test_split
import warnings

def load_and_preprocess_data(data_num=2):
    np.random.seed(0)
    input_data = np.random.rand(59528, 9)
    target_data = np.random.rand(59528, 6)

    input_min = input_data.min(axis=0)
    input_max = input_data.max(axis=0)
    target_min = target_data.min(axis=0)
    target_max = target_data.max(axis=0)

    input_data_norm = (input_data - input_min) / (input_max - input_min + 1e-8)
    target_data_norm = (target_data - target_min) / (target_max - target_min + 1e-8)

    print("✅ 데이터 로드 및 전처리 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - target_data shape: {target_data.shape}")
    print(f" - input_data_norm shape: {input_data_norm.shape}")
    print(f" - target_data_norm shape: {target_data_norm.shape}")

    return input_data_norm, target_data_norm, input_min, input_max, target_min, target_max, input_data, target_data

def train_ann(X_train, Y_train, X_val, Y_val, fold_idx=None):
    model = MLPRegressor(hidden_layer_sizes=(6, 6),
                         activation='tanh',
                         solver='adam',
                        #  solver='lbfgs',
                         max_iter=6000)

    model.fit(X_train.T, Y_train.T)
    Y_val_pred = model.predict(X_val.T)
    mse_val = mean_squared_error(Y_val.T, Y_val_pred)

    print(f"📈 Fold {fold_idx + 1} - Validation MSE: {mse_val:.5f}")
    return model, mse_val

def run_kfold_validation(input_data_norm, target_data_norm, n_bins=8, k_folds=8):
    print("\n🔍 Running Stratified K-Fold Cross Validation")

    _, bin_edges = np.histogram(target_data_norm[:, 2], bins=n_bins)
    bin_idx = np.digitize(target_data_norm[:, 2], bin_edges[:-1], right=True)

    counts = np.bincount(bin_idx)
    valid_bins = np.where(counts >= 2)[0]
    valid_idx = np.isin(bin_idx, valid_bins)
    input_data_norm = input_data_norm[valid_idx]
    target_data_norm = target_data_norm[valid_idx]
    bin_idx = bin_idx[valid_idx]

    print(f"✅ Total filtered samples: {len(bin_idx)}")
    print(f"✅ Valid bin counts: {len(valid_bins)} / {n_bins}\n")

    skf = StratifiedKFold(n_splits=k_folds, shuffle=True, random_state=42)
    mse_scores = []

    for fold, (train_val_idx, test_idx) in enumerate(skf.split(input_data_norm, bin_idx)):
        print(f"📂 Processing Fold {fold+1}/{k_folds}...")

        bin_train_val_idx = bin_idx[train_val_idx]
        try:
            train_sub_idx, val_sub_idx = train_test_split(
                train_val_idx,
                test_size=0.2,
                stratify=bin_train_val_idx,
                random_state=42
            )
        except ValueError:
            warnings.warn("⚠️ Warning: Some classes too small, using random split instead.")
            train_sub_idx, val_sub_idx = train_test_split(
                train_val_idx, test_size=0.2, random_state=42
            )

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

        model, mse_val = train_ann(X_train, Y_train, X_val, Y_val, fold_idx=fold)
        mse_scores.append(mse_val)

    avg_mse = np.mean(mse_scores)
    print(f"\n📊 평균 Validation MSE (Across Folds): {avg_mse:.5f}")

    plt.figure(figsize=(8, 4))
    plt.plot(range(1, k_folds + 1), mse_scores, marker='o', color='orange')
    plt.title("Fold별 Validation MSE")
    plt.xlabel("Fold")
    plt.ylabel("MSE")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    return mse_scores

def main():
    print("✅ TWC Inference Node 시작")
    input_data_norm, target_data_norm, *_ = load_and_preprocess_data(data_num=2)

    if input_data_norm is not None and target_data_norm is not None:
        run_kfold_validation(input_data_norm, target_data_norm)

if __name__ == '__main__':
    main()
