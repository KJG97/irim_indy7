from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass
from source.irim_tasks.irim_tasks.manager_based.manipulation.reorientation.config.indy7.reorientation_ik_rel_env_cfg import Indy7ReorientationEnvCfg

@configclass
class INDY7ReorientationIKRelMimicEnvCfg(Indy7ReorientationEnvCfg, MimicEnvCfg):
    """
    Isaac Lab Mimic environment config class for Indy7 Reorientation IK Rel env.
    """

    def __post_init__(self):
        # post init of parents
        super().__post_init__()

        # Override the existing values
        self.datagen_config.name = "demo_src_reorientation_isaac_lab_task_D0"
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = True
        self.datagen_config.generation_num_trials = 10
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.max_num_failures = 25
        self.datagen_config.seed = 1

        # The following are the subtask configurations for the reorientation task.
        subtask_configs = []
        subtask_configs.append(
            SubTaskConfig(
                # Each subtask involves manipulation with respect to a single object frame.
                object_ref="Tcube",
                # This key corresponds to the binary indicator in "datagen_info" that signals
                # when this subtask is finished (e.g., on a 0 to 1 edge).
                subtask_term_signal="touch",    
                # Specifies time offsets for data generation when splitting a trajectory into
                # subtask segments. Random offsets are added to the termination boundary.
                subtask_term_offset_range=(10, 20),
                # Selection strategy for the source subtask segment during data generation
                selection_strategy="nearest_neighbor_object",
                # Optional parameters for the selection strategy function
                selection_strategy_kwargs={"nn_k": 3},
                # Amount of action noise to apply during this subtask
                action_noise=0.03,
                # Number of interpolation steps to bridge to this subtask segment
                num_interpolation_steps=5,
                # Additional fixed steps for the robot to reach the necessary pose
                num_fixed_steps=0,
                # If True, apply action noise during the interpolation phase and execution
                apply_noise_during_interpolation=False,
            )
        )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         # Each subtask involves manipulation with respect to a single object frame.
        #         object_ref="Tcube",
        #         # This key corresponds to the binary indicator in "datagen_info" that signals
        #         # when this subtask is finished (e.g., on a 0 to 1 edge).   
        #         subtask_term_signal="center",       
        #         # Specifies time offsets for data generation when splitting a trajectory into
        #         # subtask segments. Random offsets are added to the termination boundary.
        #         subtask_term_offset_range=(10, 20),
        #         # Selection strategy for the source subtask segment during data generation
        #         selection_strategy="nearest_neighbor_object",   
        #         # Optional parameters for the selection strategy function
        #         selection_strategy_kwargs={"nn_k": 3},
        #         # Amount of action noise to apply during this subtask
        #         action_noise=0.03,
        #         # Number of interpolation steps to bridge to this subtask segment
        #         num_interpolation_steps=5,
        #         # Additional fixed steps for the robot to reach the necessary pose
        #         num_fixed_steps=0,
        #         # If True, apply action noise during the interpolation phase and execution
        #         apply_noise_during_interpolation=False,
        #     )
        # )   
        subtask_configs.append(
            SubTaskConfig(
                # 마지막 subtask도 Tcube 객체에 대한 작업
                object_ref="Tcube",
                # 마지막 단계이므로 종료 신호 불필요
                subtask_term_signal=None,
                # 마지막 단계이므로 시간 오프셋 없음
                subtask_term_offset_range=(0, 0),
                # 소스 subtask 세그먼트 선택 전략
                selection_strategy="nearest_neighbor_object",
                # 선택 전략 함수의 추가 매개변수
                selection_strategy_kwargs={"nn_k": 3},
                # 이 subtask 동안 적용할 액션 노이즈 양
                action_noise=0.03,
                # 이 subtask 세그먼트로 전환하기 위한 보간 단계 수
                num_interpolation_steps=5,
                # 로봇이 필요한 자세에 도달하기 위한 추가 고정 단계
                num_fixed_steps=0,
                # True이면 보간 단계와 실행 중에도 액션 노이즈 적용
                apply_noise_during_interpolation=False,
            )
        )
        self.subtask_configs["indy7"] = subtask_configs 