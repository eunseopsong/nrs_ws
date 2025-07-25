import os
from datetime import datetime

import hydra
import torch
from omegaconf import DictConfig, OmegaConf
from hydra.utils import to_absolute_path

# transic 내부 유틸들
from nrs_transic.utils.reformat import omegaconf_to_dict, print_dict
from nrs_transic.utils.utils import set_np_formatting, set_seed

# rl-games 관련 유틸
from nrs_transic.utils.rlgames_utils import RLGPUAlgoObserver, MultiObserver
from nrs_transic.utils.wandb_utils import WandbAlgoObserver, WandbVideoCaptureWrapper

# 모델 및 러너 정의
from nrs_transic.rl.runner import Runner
from nrs_transic.rl.network_builder import DictObsBuilder
from nrs_transic.rl.models import ModelA2CContinuousLogStd
from rl_games.algos_torch.model_builder import register_network, register_model


# def preprocess_train_config(cfg, config_dict):
#     """
#     Preprocess training config dict for rl-games.
#     """
#     train_cfg = config_dict["params"]["config"]

#     train_cfg["device"] = cfg.rl_device
#     train_cfg["population_based_training"] = False
#     train_cfg["pbt_idx"] = None
#     train_cfg["full_experiment_name"] = cfg.get("full_experiment_name")

#     try:
#         model_size_multiplier = config_dict["params"]["network"]["mlp"]["model_size_multiplier"]
#         if model_size_multiplier != 1:
#             units = config_dict["params"]["network"]["mlp"]["units"]
#             for i, u in enumerate(units):
#                 units[i] = u * model_size_multiplier
#             print(f'Modified MLP units by x{model_size_multiplier} to {units}')
#     except KeyError:
#         pass

#     return config_dict


@hydra.main(version_base="1.1", config_name="config", config_path="../../cfg")
def main(cfg: DictConfig):
    print("train.py 헤더 로딩 및 실행 성공!")


if __name__ == "__main__":
    main()
