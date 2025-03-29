# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ee_frame_pos(env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"), body_name: str = "panda_hand") -> torch.Tensor:
    robot = env.scene[robot_cfg.name]
    body_idx = robot.body_names.index(body_name)
    return robot.data.body_pos_w[:, body_idx] - env.scene.env_origins[:, 0:3]

def ee_frame_quat(env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"), body_name: str = "panda_hand") -> torch.Tensor:
    robot = env.scene[robot_cfg.name]
    body_idx = robot.body_names.index(body_name)
    return robot.data.body_quat_w[:, body_idx]


def command_reached(
    env: ManagerBasedRLEnv,
    command_name: str = "ee_pose",
    position_threshold: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    body_name: str = "panda_hand",
) -> torch.Tensor:
    """
    목표 위치에 도달했는지 확인하는 관측 함수
    
    Args:
        env: 환경 인스턴스
        command_name: 명령 이름
        position_threshold: 도달 판정 임계값(미터)
        asset_cfg: 로봇 자산 설정
        body_name: 엔드 이펙터 바디 이름
        
    Returns:
        도달 여부를 나타내는 불리언 텐서
    """
    # 로봇 객체 가져오기
    robot: Articulation = env.scene[asset_cfg.name]
    
    # 엔드 이펙터의 현재 위치 가져오기
    if hasattr(robot, "body_names") and body_name in robot.body_names:
        body_idx = robot.body_names.index(body_name)
        current_position = robot.data.body_pos_w[:, body_idx]
    else:
        raise ValueError(f"Body name '{body_name}' not found in robot asset.")
    
    # 목표 위치 가져오기 (command_manager 사용)
    command = env.command_manager.get_command(command_name)
    target_position = command[:, :3]
    
    # 현재 위치와 목표 위치 간의 거리 계산
    position_error = torch.norm(current_position - target_position, dim=1)
    
    # 거리가 임계값보다 작으면 도달로 간주
    reached = position_error < position_threshold
    
    return reached