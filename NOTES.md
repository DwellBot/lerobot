eval $(poetry env activate)

poetry run python lerobot/scripts/control_robot.py \
    --robot.type=aloha \
    --control.type=teleoperate \
    --control.fps=30
