# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import Articulation

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def reached_target_position(
    env: "ManagerBasedRLEnv",
    position_threshold: float = 0.1,
    command_name: str = "ee_pose",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    body_name: str = "tcp",
):
    """
    목표 위치에 도달했는지 확인하는 종료 조건
    
    Args:
        env: 환경 인스턴스
        position_threshold: 성공으로 간주할 위치 오차 임계값(미터)
        command_name: 명령 이름
        asset_cfg: 로봇 자산 설정
        body_name: 엔드 이펙터 바디 이름
        
    Returns:
        success: 각 환경에 대한 성공 여부를 나타내는 불리언 텐서
    """
    # 로봇 객체 가져오기
    robot: Articulation = env.scene[asset_cfg.name]

    # 로봇 베이스의 위치와 방향 가져오기
    base_pos = robot.data.root_pos_w  # 베이스 위치
    base_quat = robot.data.root_quat_w  # 베이스 방향 (쿼터니언)
    
    # 엔드 이펙터의 현재 위치 가져오기
    if body_name in robot.body_names:
        body_idx = robot.body_names.index(body_name)
        tcp_world_pos = robot.data.body_pos_w[:, body_idx]
    else:
        raise ValueError(f"Body name '{body_name}' not found in robot asset.")
    
    # 목표 위치 가져오기 (command_manager 사용)
    command = env.command_manager.get_command(command_name)
    target_position = command[:, :3]
    tcp_rel_pos = tcp_world_pos - base_pos  # 베이스 기준 TCP 상대 위치
    
    # 현재 위치와 목표 위치 간의 거리 계산
    position_error = torch.norm(tcp_rel_pos - target_position, dim=1)
    
    # 거리가 임계값보다 작으면 성공으로 간주
    success = position_error < position_threshold
    return success