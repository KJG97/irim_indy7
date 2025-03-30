# irim_indy7

Teleoperation 실행 코드

```
$ {ISAAC_LAB_PATH}/isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Reach-Indy7-IK-Rel-v0 --num_envs 1 --teleop_device vivetracker

```

Record 실행 코드
```
$ {ISAAC_LAB_PATH}/isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Reach-Indy7-IK-Rel-v0 --teleop_device vivetracker --dataset_file ./datasets/dataset.hdf5 --num_demos 10

```

Generating demo Code
```
$ {ISAAC_LAB_PATH}/isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5 --task Isaac-Reach-Indy7-IK-Rel-Mimic-v0 --auto
```
