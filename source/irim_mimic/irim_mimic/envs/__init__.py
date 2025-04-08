# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Sub-package with environment wrappers for Isaac Lab Mimic."""

import gymnasium as gym

from .indy7_reach_ik_rel_mimic_env_cfg import INDY7ReachIKRelMimicEnvCfg
from .indy7_reach_ik_rel_mimic_env import INDY7ReachIKRelMimicEnv
from .indy7_reorientation_ik_rel_mimic_env_cfg import INDY7ReorientationIKRelMimicEnvCfg
from .indy7_reorientation_ik_rel_mimic_env import INDY7ReorientationIKRelMimicEnv

##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Reach-Indy7-IK-Rel-Mimic-v0",
    entry_point="irim_mimic.envs:INDY7ReachIKRelMimicEnv",
    kwargs={
        "env_cfg_entry_point": indy7_reach_ik_rel_mimic_env_cfg.INDY7ReachIKRelMimicEnvCfg
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Reorientation-Indy7-IK-Rel-Mimic-v0",
    entry_point="irim_mimic.envs:INDY7ReorientationIKRelMimicEnv",
    kwargs={
        "env_cfg_entry_point": indy7_reorientation_ik_rel_mimic_env_cfg.INDY7ReorientationIKRelMimicEnvCfg
    },
    disable_env_checker=True,
)