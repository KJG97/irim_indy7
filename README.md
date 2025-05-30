# irim_indy7

가상환경 접속
```
conda activate env_isaaclab
```

경로 접속
```
cd IsaacLab/irim_indy7/
```
Teleoperation 실행 코드

```
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Reorientation-Indy7-IK-Rel-v0 --num_envs 1 --teleop_device vivetracker
```

Record 실행 코드
```
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Reorientation-Indy7-IK-Rel-v0 --teleop_device vivetracker --dataset_file ./datasets/dataset.hdf5 --num_demos 100
```

Generating annotate demo Code
```
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5 --task Isaac-Reorientation-Indy7-IK-Rel-Mimic-v0 --auto
```

Testing Small Generating demo
```
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset_small.hdf5 --num_envs 100 --generation_num_trials 1000
```

Train
```
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/imitation_learning/robomimic/train.py --task Isaac-Reach-Indy7-IK-Rel-v0 --algo bc --dataset ./datasets/generated_dataset.hdf5
```

Play
```
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/imitation_learning/robomimic/play.py --task Isaac-Reach-Indy7-IK-Rel-v0 --num_rollouts 50 --checkpoint ${ROBOMIMIC_MODEL_PATH}/20250331231356/models/model_epoch_2000.pth

```
