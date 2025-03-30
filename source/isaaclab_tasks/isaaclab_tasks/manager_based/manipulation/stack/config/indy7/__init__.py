import gymnasium as gym
import os

from . import (
    agents,  # agents 폴더가 있는 경우
    stack_ik_rel_env_cfg,
    stack_joint_pos_env_cfg,
    # 다른 설정 파일들도 필요시 import
)

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-Stack-Cube-Indy7-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": stack_joint_pos_env_cfg.Indy7CubeStackEnvCfg,
    },
    disable_env_checker=True,
)

##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Stack-Cube-Indy7-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": stack_ik_rel_env_cfg.Indy7CubeStackEnvCfg,
        # robomimic 구성이 필요한 경우 추가
        # "robomimic_bc_cfg_entry_point": os.path.join(agents.__path__[0], "robomimic/bc_rnn_low_dim.json"),
    },
    disable_env_checker=True,
)

# 필요시 다른 환경 등록