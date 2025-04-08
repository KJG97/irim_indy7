# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from source.irim_tasks.irim_tasks.manager_based.manipulation.reorientation import mdp
from source.irim_tasks.irim_tasks.manager_based.manipulation.reorientation.mdp import indy7_reorientation_events
from source.irim_tasks.irim_tasks.manager_based.manipulation.reorientation.reorientation_env_cfg import ReorientationEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from source.irim_assets.irim_assets.robots.indy7 import INDY7_CFG  # isort: skip


@configclass
class EventCfg:
    """Configuration for events."""

    init_indy7_arm_pose = EventTerm(
        func=indy7_reorientation_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.0, 0.0, -1.57, 0.0, 1.57, 0.0],
        },
    )

    randomize_indy7_joint_state = EventTerm(
        func=indy7_reorientation_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_cube_positions = EventTerm(
        func=indy7_reorientation_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.1, 0.4), "y": (-0.2, 0.2), "z": (0.866, 0.866), "yaw": (-3.14, 3.14, 0)},
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("Tcube")],
        },
    )


@configclass
class Indy7ReorientationEnvCfg(ReorientationEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set events
        self.events = EventCfg()

        # Set Franka as robot
        self.scene.robot = INDY7_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]

        # Add semantics to table
        self.scene.table.spawn.semantic_tags = [("class", "table")]

        # Add semantics to ground
        self.scene.plane.semantic_tags = [("class", "ground")]

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["joint.*"], scale=0.5, use_default_offset=True
        )

        # Rigid body properties of each cube
        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        # Set each stacking cube deterministically
        self.scene.Tcube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Tcube",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.0, 0.0, 0.866], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/home/jkkim/IsaacLab/irim_indy7/asset/Tcube_USD/Tcube.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "Tcube")],
            ),
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/tcp",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
        )
