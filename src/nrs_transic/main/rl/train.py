# main/rl/train.py

import os
from datetime import datetime

import hydra
# import isaacgym
import torch
from omegaconf import DictConfig, OmegaConf
from hydra.utils import to_absolute_path

from transic.utils.reformat import omegaconf_to_dict, print_dict
from transic.utils.utils import set_np_formatting, set_seed
from transic.utils.rlgames_utils import RLGPUAlgoObserver, MultiObserver, ComplexObsRLGPUEnv
from transic.utils.wandb_utils import WandbAlgoObserver, WandbVideoCaptureWrapper
from rl_games.common import env_configurations, vecenv
from transic.rl.runner import Runner
from transic.rl.network_builder import DictObsBuilder
from transic.rl.models import ModelA2CContinuousLogStd
from rl_games.algos_torch.model_builder import register_network, register_model
import transic_envs


def preprocess_train_config(cfg, config_dict):
    train_cfg = config_dict["params"]["config"]
    train_cfg["device"] = cfg.rl_device
    train_cfg["population_based_training"] = False
    train_cfg["pbt_idx"] = None
    train_cfg["full_experiment_name"] = cfg.get("full_experiment_name")

    try:
        model_size_multiplier = config_dict["params"]["network"]["mlp"]["model_size_multiplier"]
        if model_size_multiplier != 1:
            units = config_dict["params"]["network"]["mlp"]["units"]
            for i, u in enumerate(units):
                units[i] = u * model_size_multiplier
    except KeyError:
        pass

    return config_dict


@hydra.main(version_base="1.1", config_name="config", config_path="../cfg")
def main(cfg: DictConfig):
    if cfg.display:
        import cv2
        import numpy as np
        cv2.imshow("dummy", np.zeros((1, 1, 3), dtype=np.uint8))
        cv2.waitKey(1)

    # config 출력
    cfg_dict = omegaconf_to_dict(cfg)
    print_dict(cfg_dict)

    # 기본 출력 형식
    set_np_formatting()

    # rank 및 seed
    global_rank = int(os.getenv("RANK", "0"))
    cfg.seed = set_seed(cfg.seed, torch_deterministic=cfg.torch_deterministic, rank=global_rank)

    # 환경 생성 함수
    def create_isaacgym_env():
        kwargs = dict(
            sim_device=cfg.sim_device,
            rl_device=cfg.rl_device,
            graphics_device_id=cfg.graphics_device_id,
            multi_gpu=cfg.multi_gpu,
            cfg=cfg.task,
            display=cfg.display,
            record=cfg.capture_video,
            has_headless_arg=False,
        )
        if "pcd" not in cfg.task_name.lower():
            kwargs["headless"] = cfg.headless
            kwargs["has_headless_arg"] = True
        return transic_envs.make(**kwargs)

    # 등록
    env_configurations.register("rlgpu", {
        "vecenv_type": "RLGPU",
        "env_creator": create_isaacgym_env,
    })
    vecenv.register("RLGPU", lambda config_name, num_actors: ComplexObsRLGPUEnv(config_name))

    # 네트워크 등록
    register_model("my_continuous_a2c_logstd", ModelA2CContinuousLogStd)
    register_network("dict_obs_actor_critic", DictObsBuilder)

    rlg_config_dict = omegaconf_to_dict(cfg.rl_train)
    rlg_config_dict = preprocess_train_config(cfg, rlg_config_dict)

    observers = [RLGPUAlgoObserver()]
    if cfg.wandb_activate and global_rank == 0:
        cfg.seed += global_rank
        observers.append(WandbAlgoObserver(cfg))

    runner = Runner(MultiObserver(observers))
    runner.load(rlg_config_dict)
    runner.reset()

    prefix = "dump_" if cfg.test else ""
    experiment_dir = os.path.join(
        "runs",
        prefix + cfg.rl_train.params.config.name + "_{date:%m-%d-%H-%M-%S}".format(date=datetime.now())
    )
    os.makedirs(experiment_dir, exist_ok=True)
    with open(os.path.join(experiment_dir, "config.yaml"), "w") as f:
        f.write(OmegaConf.to_yaml(cfg))

    runner.run({
        "train": not cfg.test,
        "play": cfg.test,
        "checkpoint": cfg.checkpoint,
        "from_ckpt_epoch": cfg.from_ckpt_epoch,
        "sigma": cfg.sigma if cfg.sigma != "" else None,
        "save_rollouts": {
            "save_rollouts": cfg.save_rollouts,
            "rollout_saving_fpath": os.path.join(experiment_dir, "rollouts.hdf5"),
            "save_successful_rollouts_only": cfg.save_successful_rollouts_only,
            "num_rollouts_to_save": cfg.num_rollouts_to_save,
            "min_episode_length": cfg.min_episode_length,
        },
    })


if __name__ == "__main__":
    main()
