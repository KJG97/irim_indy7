# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Neuromeka Indy7 robot.

The following configurations are available:

* :obj:`INDY7_CFG`: Neuromeka Indy7 robot
* :obj:`INDY7_HIGH_PD_CFG`: Neuromeka Indy7 robot with stiffer PD control

"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

WITH_HAND = False

if WITH_HAND:
    INDY7_CFG = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/jkkim/IRIM_LAB/imitation_learning/Indy7_USD/indy7.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
            ),
            # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=[-0.3, 0.0, 0.886],
            joint_pos={
                "joint1": 0.0,
                "joint2": 0.872665,
                "joint3": -2.0944,
                "joint4": 0.0,
                "joint5": -1.39626,
                "joint6": 2.44346,
                # 핑거 조인트 추가
                "index_joint_0": 0.0,
                "index_joint_1": 0.0,
                "index_joint_2": 0.0,
                "index_joint_3": 0.0,
                "middle_joint_0": 0.0,
                "middle_joint_1": 0.0,
                "middle_joint_2": 0.0,
                "middle_joint_3": 0.0,
                "ring_joint_0": 0.0,
                "ring_joint_1": 0.0,
                "ring_joint_2": 0.0,
                "ring_joint_3": 0.0,
                "thumb_joint_0": 0.0,
                "thumb_joint_1": 0.0,
                "thumb_joint_2": 0.0,
                "thumb_joint_3": 0.0,
            },
        ),
        actuators={
            "indy7_arm_upper": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-3]"],
                effort_limit=8000.0,
                velocity_limit=2.618,
                stiffness=10000.0,
                damping=1000.0,
            ),
            "indy7_arm_lower": ImplicitActuatorCfg(
                joint_names_expr=["joint[4-6]"],
                effort_limit=40000.0,
                velocity_limit=3.1416,
                stiffness=10000.0,
                damping=1000.0,
            ),
            # 핑거 조인트 추가
            "indy7_finger_index": ImplicitActuatorCfg(
                joint_names_expr=["index_joint_[0-3]"],
                effort_limit=21.0,
                velocity_limit=6.2832,
                stiffness=10000.0,
                damping=1000.0,
            ),
            "indy7_finger_middle": ImplicitActuatorCfg(
                joint_names_expr=["middle_joint_[0-3]"],
                effort_limit=21.0,
                velocity_limit=6.2832,
                stiffness=10000.0,
                damping=1000.0,
            ),
            "indy7_finger_ring": ImplicitActuatorCfg(
                joint_names_expr=["ring_joint_[0-3]"],
                effort_limit=21.0,
                velocity_limit=6.2832,
                stiffness=10000.0,
                damping=1000.0,
            ),
            "indy7_finger_thumb": ImplicitActuatorCfg(
                joint_names_expr=["thumb_joint_[0-3]"],
                effort_limit=21.0,
                velocity_limit=6.2832,
                stiffness=10000.0,
                damping=1000.0,
            ),
        },
        soft_joint_pos_limit_factor=1.0,
    )
else :
    INDY7_CFG = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/jkkim/IRIM_LAB/imitation_learning/Indy7_USD/indy7.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
            ),
            # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=[-0.3, 0.0, 0.886],
            joint_pos={
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": -1.5708,
                "joint4": 0.0,
                "joint5": 1.5708,
                "joint6": 0.0,
            },
        ),
        actuators={
            "indy7_arm_upper": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-3]"],
                effort_limit=8000.0,
                velocity_limit=2.618,
                stiffness=10000.0,
                damping=1000.0,
            ),
            "indy7_arm_lower": ImplicitActuatorCfg(
                joint_names_expr=["joint[4-6]"],
                effort_limit=40000.0,
                velocity_limit=3.1416,
                stiffness=10000.0,
                damping=1000.0,
            ),
        },
        soft_joint_pos_limit_factor=1.0,
    )

INDY7_HIGH_PD_CFG = INDY7_CFG.copy()
INDY7_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
INDY7_HIGH_PD_CFG.actuators["indy7_arm_upper"].stiffness = 1000.0
INDY7_HIGH_PD_CFG.actuators["indy7_arm_upper"].damping = 150.0
INDY7_HIGH_PD_CFG.actuators["indy7_arm_lower"].stiffness = 1000.0
INDY7_HIGH_PD_CFG.actuators["indy7_arm_lower"].damping = 150.0
"""더 강한 PD 제어를 사용하는 Neuromeka Indy7 로봇 설정.

이 설정은 differential IK를 사용한 작업 공간 제어에 유용합니다.
"""