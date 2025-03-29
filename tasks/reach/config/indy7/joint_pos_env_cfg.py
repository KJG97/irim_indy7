# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from ...reach_env_cfg import ReachEnvCfg 
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg


##
# Pre-defined configs
##
from isaaclab_assets import INDY7_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class INDY7ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to franka
        self.scene.robot = INDY7_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["joint.*"], scale=0.5, use_default_offset=True
        )

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link", 
            debug_vis=False,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/tcp",
                    name="tcp",
                    offset=OffsetCfg(pos=[0.0, 0.0, 0.0])
                )
            ]
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose.body_name = "tcp"
        self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)

