from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass
from source.irim_tasks.irim_tasks.manager_based.manipulation.reach.config.indy7.reach_ik_rel_env_cfg import INDY7ReachEnvCfg

@configclass
class INDY7ReachIKRelMimicEnvCfg(INDY7ReachEnvCfg, MimicEnvCfg):
    """
    Isaac Lab Mimic environment config class for Indy7 Reach IK Rel env.
    """

    def __post_init__(self):
        # post init of parents
        super().__post_init__()

        # Override the existing values
        self.datagen_config.name = "demo_src_reach_isaac_lab_task_D0"
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = True
        self.datagen_config.generation_num_trials = 10
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.max_num_failures = 25
        self.datagen_config.seed = 1

        # The following is the subtask configuration for the reach task.
        # Reach 작업은 단순히 목표 위치에 도달하는 하나의 서브태스크만 있음
        subtask_configs = []
        subtask_configs.append(
            SubTaskConfig(
                # 서브태스크 완료 시그널
                subtask_term_signal='reached',
                # 서브태스크 종료 경계의 시간 오프셋 범위
                subtask_term_offset_range=(0, 0),
                # 소스 서브태스크 세그먼트 선택 전략
                selection_strategy='random',
                # 선택 전략 함수의 선택적 매개변수
                selection_strategy_kwargs={},
                # 이 서브태스크 동안 적용할 액션 노이즈 양
                action_noise=0.03,
                # 이 서브태스크 세그먼트로 연결하기 위한 보간 단계 수
                num_interpolation_steps=5,
                # 로봇이 필요한 자세에 도달하기 위한 추가 고정 단계
                num_fixed_steps=0,
                # True인 경우 보간 단계 및 실행 중에 액션 노이즈 적용
                apply_noise_during_interpolation=False,
            )
        )
        self.subtask_configs["tcp"] = subtask_configs
