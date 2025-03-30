# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg, DeformableObjectCfg, AssetBaseCfg,ArticulationCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.indy7 import INDY7_CFG  # isort: skip

WITH_HAND = False

@configclass
class Indy7CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path="/home/jkkim/IRIM_LAB/imitation_learning/DESK_USD/URDF-PF-DESK.usd",
                scale=(1.0, 1.0, 1.0) 
            ),
        )

        # Set Indy7 as robot
        self.scene.robot = INDY7_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot"
        )

        # Set actions for the specific robot type (indy7)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["joint.*"], scale=0.5, use_default_offset=True
        )
        
        # 핑거 관절 액션 설정 (여기서는 단순하게 모든 관절이 동시에 움직이도록 설정)
        # 실제 구현에서는 손가락별로 더 세분화된 제어가 필요할 수 있음
        
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["index_joint.*", "middle_joint.*", "ring_joint.*", "thumb_joint.*"],
            open_command_expr={
                "index_joint_.*": 0.0,
                "middle_joint_.*": 0.0,
                "ring_joint_.*": 0.0,
                "thumb_joint_.*": 0.0
            },
            close_command_expr={
                "index_joint_.*": 1.0,  
                "middle_joint_.*": 1.0,
                "ring_joint_.*": 1.0,
                "thumb_joint_.*": 1.0
            },
        )

        self.commands.object_pose.body_name = "tcp"

        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 2.5),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
        
        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        
        # Indy7 로봇의 프레임 설정
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",  # 베이스 링크 이름 확인 필요
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/tcp",  # 엔드 이펙터 링크 이름 확인 필요
                    name="tcp",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.0],  # 실제 offset 값 확인 필요
                    ),
                ),
            ],
        )

@configclass
class Indy7CubeLiftEnvCfg_PLAY(Indy7CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False