# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_reoriented(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    position_threshold: float = 0.05,  # 위치 허용 오차
    orientation_threshold: float = 0.05,  # 방향 허용 오차
) -> torch.Tensor:
    """
    오브젝트가 중앙(0,0)에 위치하고 방향이 (1,0,0,0)인지 확인합니다.
    
    Args:
        env: 환경 인스턴스
        object_cfg: 대상 오브젝트 설정
        position_threshold: 중앙(0,0)으로 간주할 거리 임계값 (미터)
        orientation_threshold: 기준 방향으로 간주할 쿼터니언 차이 임계값
        
    Returns:
        torch.Tensor: 각 환경에서 오브젝트가 올바르게 재배치되었는지 여부를 나타내는 불리언 텐서
    """
    # 오브젝트 가져오기
    object: RigidObject = env.scene[object_cfg.name]

    # 오브젝트의 위치 가져오기
    object_pos = object.data.root_pos_w
    
    # 오브젝트의 방향 가져오기
    object_quat = object.data.root_quat_w

    # 1. xy 좌표가 (0,0)에 가까운지 확인
    xy_pos = object_pos[:, :2]  # x,y 좌표만 선택
    target_pos = torch.tensor([0.1, 0.0], device=env.device)
    distance_from_center = torch.linalg.vector_norm(xy_pos - target_pos, dim=1)
    is_centered = distance_from_center < position_threshold
    
    # 2. 방향이 (1,0,0,0)에 가까운지 확인
    target_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=env.device)
    
    # 쿼터니언 차이 계산 (두 쿼터니언 사이의 거리)
    # 두 쿼터니언 사이의 내적의 절대값이 1이면 동일한 방향
    quat_dot_product = torch.abs(torch.sum(object_quat * target_quat, dim=1))
    quat_distance = 1.0 - quat_dot_product  # 0이면 완전히 일치, 2이면 완전히 반대
    is_aligned = quat_distance < orientation_threshold
    # print(f"{distance_from_center.item():.4f}, {quat_distance.item():.4f}")
    # print(f"is_centered: {is_centered.item()}, is_aligned: {is_aligned.item()} \n")

    # 두 조건 모두 만족하는지 확인
    is_reoriented = torch.logical_and(is_centered, is_aligned)
    
    return is_reoriented
