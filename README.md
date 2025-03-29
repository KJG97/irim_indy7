# irim_indy7

Teleoperation 실행 코드

```
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Reach-Indy7-IK-Rel-v0 --num_envs 1 --teleop_device vivetracker

```

Record 실행 코드
```
./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Reach-Indy7-IK-Rel-v0 --teleop_device vivetracker --dataset_file ./datasets/dataset.hdf5 --num_demos 10

```
