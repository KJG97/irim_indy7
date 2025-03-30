# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Sub-package with environment wrappers for Isaac Lab Mimic."""

import gymnasium as gym

from .indy7_reach_ik_rel_mimic_env_cfg import INDY7ReachIKRelMimicEnvCfg
from .indy7_reach_ik_rel_mimic_env import INDY7ReachIKRelMimicEnv
##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Reach-Indy7-IK-Rel-Mimic-v0",
    entry_point="isaaclab_mimic.envs:INDY7ReachIKRelMimicEnv",
    kwargs={
        "env_cfg_entry_point": indy7_reach_ik_rel_mimic_env_cfg.INDY7ReachIKRelMimicEnvCfg
    },
    disable_env_checker=True,
)
