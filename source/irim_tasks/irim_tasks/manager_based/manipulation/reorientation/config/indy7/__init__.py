# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym
import os

from . import (
    agents,
    reorientation_joint_pos_env_cfg,
    reorientation_ik_rel_env_cfg,
)

##
# Register Gym environments.
##

##
# Joint Position Control
##
gym.register(
    id="Isaac-Reorientation-Indy7-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": reorientation_joint_pos_env_cfg.Indy7ReorientationEnvCfg,
    },
    disable_env_checker=True,
)


##
# Inverse Kinematics - Relative Pose Control
##
gym.register(
    id="Isaac-Reorientation-Indy7-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": reorientation_ik_rel_env_cfg.Indy7ReorientationEnvCfg,
        "robomimic_bc_cfg_entry_point": os.path.join(agents.__path__[0], "robomimic/bc_rnn_low_dim.json"),
    },
    disable_env_checker=True,
)

