#!/bin/bash
# Copy LSTM and optional model weights from your local carla_lstm_mpc_project.
# Usage: ./scripts/setup_models_from_local.sh /home/supawich/Desktop/CARLA_0.9.16/carla_lstm_mpc_project

SRC="${1:-/home/supawich/Desktop/CARLA_0.9.16/carla_lstm_mpc_project}"
DEST="."
if [ ! -d "$SRC" ]; then
  echo "Usage: $0 <path_to_carla_lstm_mpc_project>"
  exit 1
fi

mkdir -p "$DEST/data/autopilot_20260208_150902/lstm_model"
mkdir -p "$DEST/data/autopilot_20260208_150902/resnet_lane_model"
mkdir -p "$DEST/weights"

if [ -f "$SRC/data/autopilot_20260208_150902/lstm_model/best_model.pth" ]; then
  cp "$SRC/data/autopilot_20260208_150902/lstm_model/best_model.pth" "$DEST/data/autopilot_20260208_150902/lstm_model/"
  echo "Copied LSTM best_model.pth"
fi
if [ -f "$SRC/data/autopilot_20260223_233442/lstm_model/best_model.pth" ]; then
  mkdir -p "$DEST/data/autopilot_20260223_233442/lstm_model"
  cp "$SRC/data/autopilot_20260223_233442/lstm_model/best_model.pth" "$DEST/data/autopilot_20260223_233442/lstm_model/"
  echo "Copied LSTM (20260223) best_model.pth"
fi
if [ -f "$SRC/data/autopilot_20260208_150902/resnet_lane_model/resnet_lane_best.pth" ]; then
  cp "$SRC/data/autopilot_20260208_150902/resnet_lane_model/resnet_lane_best.pth" "$DEST/data/autopilot_20260208_150902/resnet_lane_model/"
  echo "Copied ResNet lane best"
fi
if [ -f "$SRC/weights/ufldv2_tusimple_res18.pth" ]; then
  cp "$SRC/weights/ufldv2_tusimple_res18.pth" "$DEST/weights/"
  echo "Copied Ultra-Fast Lane weights"
fi
echo "Done. Set config.yaml temporal.trained_model_path to the path of best_model.pth you use."
