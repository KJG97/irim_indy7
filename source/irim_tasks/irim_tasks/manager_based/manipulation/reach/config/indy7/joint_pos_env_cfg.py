# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
import source.irim_tasks.irim_tasks.manager_based.manipulation.reach.mdp as mdp
from source.irim_tasks.irim_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg 
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from source.irim_tasks.irim_tasks.manager_based.manipulation.reach.mdp import indy7_reach_events
##
# Pre-defined configs
##
from source.irim_assets.irim_assets.robots.indy7 import INDY7_CFG  # isort: skip

@configclass
class EventCfg:
    """Configuration for events."""

    init_indy7_arm_pose = EventTerm(
        func=indy7_reach_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.0, 0.0, -1.57, 0.0, 1.57, 0.0],
        },
    )

    randomize_indy7_joint_state = EventTerm(
        func=indy7_reach_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

##
# Environment configuration
##
@configclass
class INDY7ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # Set events
        self.events = EventCfg()

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

