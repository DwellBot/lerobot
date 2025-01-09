source ~/miniconda3/bin/activate
conda activate lerobot

python lerobot/scripts/visualize_dataset.py   \  
    --repo-id dwellbot/aloha_test       \
    --local-files-only 1     \
    --episode-index 0

python lerobot/scripts/control_robot.py replay \
    --robot-path lerobot/configs/robot/aloha.yaml \
    --robot-overrides max_relative_target=null \
    --fps 30 \
    --local-files-only 1 \
    --repo-id dwellbot/aloha_test  \
    --episode 0


python lerobot/scripts/train.py \
  dataset_repo_id=dwellbot/aloha_test \
  policy=act_dwellbot \
  env=aloha_real \
  hydra.run.dir=outputs/train/act_aloha_test \
  hydra.job.name=act_aloha_test \
  device=cuda \
  wandb.enable=true

  python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/aloha.yaml \
  --robot-overrides max_relative_target=null \
  --single-task SINGLE_TASK \
  --fps 30 \
  --repo-id dwellbot/eval_act_aloha_test \
  --tags aloha tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 15 \
  --reset-time-s 10 \
  --num-episodes 2 \
  --num-image-writer-processes 1 \
  -p outputs/train/act_aloha_test/checkpoints/last/pretrained_model