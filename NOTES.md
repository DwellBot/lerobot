eval $(poetry env activate)

poetry run python lerobot/scripts/control_robot.py \
    --robot.type=aloha \
    --control.type=teleoperate \
    --control.fps=30

python lerobot/scripts/control_robot.py \
    --robot.type=aloha \
    --robot.max_relative_target=null \
    --control.type=record \
    --control.fps 30 \
    --control.repo_id=dwellbot/test \
    --control.single_task="test it" \
    --control.num_episodes=50 \
    --control.warmup_time_s=2 \
    --control.episode_time_s=30 \
    --control.reset_time_s=10 \
    --control.push_to_hub=false

python lerobot/scripts/control_robot.py \
--control.type=record \
--robot-path lerobot/configs/robot/dwell.yaml
--robot-overrides max_relative_target=null
--single-task simple_var_1
--fps 30
--repo-id dwellbot/eval_simple_var_1
--tags aloha tutorial eval
--warmup-time-s 5
--episode-time-s 15
--reset-time-s 10
--num-episodes 15
--num-image-writer-processes 1
--push-to-hub 0
--local-files-only 1
-p /data1/outputs/train/simple_var_1/checkpoints/last/pretrained_model

python lerobot/scripts/control_robot.py \
    --robot.type=aloha \
    --control.type=record \
    --control.fps=30 \
    --control.single_task="Wave" \
    --control.repo_id="dwellbot/eval_wave" \
    --control.private=true \
    --control.num_episodes=1 \
    --control.warmup_time_s=2 \
    --control.episode_time_s=30 \
    --control.reset_time_s=10 \
    --control.push_to_hub=false \
    --control.local_files_only=true \
    --control.policy.path=/data1/outputs/train/wave/checkpoints/last/pretrained_model

## Train

python lerobot/scripts/train.py \
dataset_repo_id=dwellbot/wave
policy=act_dwellbot
env=dwellbot_env
hydra.run.dir=outputs/train/simple_var_1
hydra.job.name=simple_var_1
device=cuda
wandb.enable=true

python lerobot/scripts/train.py \
 --dataset.repo_id=dwellbot/wave \
 --env.type=aloha \
 --env.task=wave \
 --policy.type=act