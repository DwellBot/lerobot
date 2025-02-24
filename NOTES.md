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
