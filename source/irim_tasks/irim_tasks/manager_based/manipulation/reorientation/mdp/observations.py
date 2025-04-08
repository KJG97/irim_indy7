# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def Tcube_obs(
    env: ManagerBasedRLEnv,
    Tcube_cfg: SceneEntityCfg = SceneEntityCfg("Tcube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
):
    Tcube: RigidObject = env.scene[Tcube_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    Tcube_pos_w = Tcube.data.root_pos_w
    Tcube_quat_w = Tcube.data.root_quat_w
    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]
    gripper_to_Tcube = Tcube_pos_w - ee_pos_w

    return torch.cat(
        (
            Tcube_pos_w - env.scene.env_origins,
            Tcube_quat_w,
            gripper_to_Tcube,
        ),
        dim=1,
    )

def ee_frame_pos(env: ManagerBasedRLEnv, ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_frame_pos = ee_frame.data.target_pos_w[:, 0, :] - env.scene.env_origins[:, 0:3]

    return ee_frame_pos

def ee_frame_quat(env: ManagerBasedRLEnv, ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_frame_quat = ee_frame.data.target_quat_w[:, 0, :]

    return ee_frame_quat

def object_touch(
    env: ManagerBasedRLEnv,
    ee_frame_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    touch_threshold: float = 0.03,  # 접촉 거리 임계값 (작게 설정)
) -> torch.Tensor:
    """
    로봇과 오브젝트의 접촉 여부를 감지합니다.
    
    Args:
        env: 환경 인스턴스
        ee_frame_cfg: 엔드 이펙터 프레임 설정
        object_cfg: 대상 오브젝트 설정
        touch_threshold: 접촉으로 간주할 거리 임계값 (미터)
        
    Returns:
        torch.Tensor: 각 환경에서 접촉 여부를 나타내는 불리언 텐서
    """
    # 엔드 이펙터 프레임 가져오기
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # 오브젝트 가져오기
    object: RigidObject = env.scene[object_cfg.name]

    # 저장된 초기 Tcube 위치 가져오기
    if "initial_Tcube_pos" not in env.extras:
         # 초기화되지 않은 경우 경고 또는 오류 처리 (혹은 기본값 설정)
         print("Warning: initial_Tcube_pos not found in env.extras. Returning False.")
         return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    # object_initial_pos 텐서 가져오기 (Shape: [num_envs, 3])
    object_initial_pos = env.extras["initial_Tcube_pos"]

    # 현재 엔드 이펙터 위치 가져오기
    end_effector_pos = ee_frame.data.target_pos_w[:, 0, :] # Shape: [num_envs, 3]
    
    # z축 방향 단위 벡터 생성
    z_offset = torch.zeros_like(end_effector_pos)
    z_offset[:, 2] = 0.20  
    
    # 오프셋이 적용된 엔드 이펙터 위치 계산
    adjusted_end_effector_pos = end_effector_pos - z_offset
    
    # 두 물체 사이의 거리 계산 (수정된 위치 사용)
    distance = torch.linalg.vector_norm(object_initial_pos - adjusted_end_effector_pos, dim=1)
    
    # 거리가 임계값보다 작으면 접촉으로 간주
    touched = distance < touch_threshold

    return touched

def object_center(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    position_threshold: float = 0.05,  # 위치 허용 오차
) -> torch.Tensor:
    """
    Tcube(object)의 x,y 좌표가 (0,0)에 있는지 확인합니다.
    
    Args:
        env: 환경 인스턴스
        object_cfg: 대상 오브젝트(Tcube) 설정
        position_threshold: 중앙(0,0)으로 간주할 거리 임계값 (미터)
        
    Returns:
        torch.Tensor: 각 환경에서 오브젝트가 중앙에 있는지 여부를 나타내는 불리언 텐서
    """
    # 오브젝트 가져오기
    object: RigidObject = env.scene[object_cfg.name]

    # 오브젝트의 위치 가져오기
    object_pos = object.data.root_pos_w

    # x, y 좌표만 추출 (z 좌표는 무시)
    xy_pos = object_pos[:, :2]  # [:, :2]는 모든 환경에 대해 x,y 좌표만 선택

    # x, y 좌표의 원점(0,0)으로부터의 거리 계산
    distance_from_center = torch.linalg.vector_norm(xy_pos, dim=1)
    
    # 원점에서 threshold 이내에 있으면 중앙에 있다고 판단
    is_centered = distance_from_center < position_threshold
    
    return is_centered