"""
Train LSTM predictor from collected CARLA autopilot data.

Input: sequence of ResNet features (10 frames)
Output: predicted vehicle state [x, y, yaw, velocity] at next frame

Usage:
    python training/train_lstm.py --data data/autopilot_20260223_233442
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import argparse
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import csv
import logging
import time

from temporal.lstm_predictor import LSTMPredictor
from utils.device_utils import get_device

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
logger = logging.getLogger(__name__)

# ─── Hyperparameters ───────────────────────────────────────────
SEQUENCE_LENGTH = 10
FEATURE_DIM = 512
STATE_DIM = 4          # x, y, yaw, velocity
HIDDEN_SIZE = 256
NUM_LAYERS = 2
DROPOUT = 0.1
BATCH_SIZE = 64
LEARNING_RATE = 1e-3
NUM_EPOCHS = 50
PATIENCE = 10          # early stopping
TRAIN_SPLIT = 0.85


class DrivingSequenceDataset(Dataset):
    """Dataset that creates (feature_sequence, target_state) pairs."""

    def __init__(self, features_list, states_list, seq_len=SEQUENCE_LENGTH):
        """
        Args:
            features_list: list of (N,) np arrays of ResNet features per frame
            states_list: list of [x, y, yaw, velocity] per frame
            seq_len: number of frames per input sequence
        """
        self.seq_len = seq_len
        self.features = features_list
        self.states = states_list
        self.valid_indices = list(range(seq_len, len(features_list)))

    def __len__(self):
        return len(self.valid_indices)

    def __getitem__(self, idx):
        target_idx = self.valid_indices[idx]
        start_idx = target_idx - self.seq_len

        seq = np.stack(self.features[start_idx:target_idx], axis=0)
        target = np.array(self.states[target_idx], dtype=np.float32)

        return torch.FloatTensor(seq), torch.FloatTensor(target)


def load_data(data_dir: str):
    """Load features and states from collected data."""
    data_dir = Path(data_dir)
    csv_path = data_dir / "data.csv"
    features_dir = data_dir / "features"

    if not csv_path.exists():
        raise FileNotFoundError(f"data.csv not found in {data_dir}")

    features_list = []
    states_list = []
    skipped = 0

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            feat_path = data_dir / row['feature_path']
            if not feat_path.exists() or row['feature_path'] == '':
                skipped += 1
                continue

            feat = np.load(str(feat_path))
            if feat.shape[0] != FEATURE_DIM:
                skipped += 1
                continue

            state = [
                float(row['x']),
                float(row['y']),
                float(row['yaw']),
                float(row['velocity']),
            ]
            features_list.append(feat)
            states_list.append(state)

    logger.info(f"Loaded {len(features_list)} samples ({skipped} skipped)")
    return features_list, states_list


def normalize_states(states_list):
    """Normalize states to zero mean, unit variance."""
    arr = np.array(states_list, dtype=np.float32)
    mean = arr.mean(axis=0)
    std = arr.std(axis=0)
    std[std < 1e-6] = 1.0
    normalized = ((arr - mean) / std).tolist()
    return normalized, mean, std


def train(data_dir: str, output_dir: str = None):
    """Train LSTM from collected data."""
    device = get_device()
    logger.info(f"Training on: {device}")

    if output_dir is None:
        output_dir = str(Path(data_dir) / "lstm_model")
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    features_list, states_list = load_data(data_dir)
    if len(features_list) < SEQUENCE_LENGTH + 10:
        logger.error("Not enough data to train. Need at least "
                      f"{SEQUENCE_LENGTH + 10} samples, got {len(features_list)}")
        return

    # Normalize states
    norm_states, state_mean, state_std = normalize_states(states_list)
    np.save(f"{output_dir}/state_mean.npy", state_mean)
    np.save(f"{output_dir}/state_std.npy", state_std)
    logger.info(f"State mean: {state_mean}")
    logger.info(f"State std:  {state_std}")

    # Split train/val
    n = len(features_list)
    split_idx = int(n * TRAIN_SPLIT)
    train_ds = DrivingSequenceDataset(features_list[:split_idx], norm_states[:split_idx])
    val_ds = DrivingSequenceDataset(features_list[split_idx:], norm_states[split_idx:])

    logger.info(f"Train: {len(train_ds)} sequences, Val: {len(val_ds)} sequences")

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True, num_workers=2)
    val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=2)

    # Model
    model = LSTMPredictor(
        input_size=FEATURE_DIM,
        hidden_size=HIDDEN_SIZE,
        num_layers=NUM_LAYERS,
        sequence_length=SEQUENCE_LENGTH,
        dropout=DROPOUT,
        output_size=STATE_DIM,
    ).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=5
    )
    criterion = nn.MSELoss()

    best_val_loss = float('inf')
    patience_counter = 0

    for epoch in range(NUM_EPOCHS):
        t0 = time.time()

        # Train
        model.train()
        train_loss = 0.0
        for seq_batch, target_batch in train_loader:
            seq_batch = seq_batch.to(device)
            target_batch = target_batch.to(device)

            pred = model(seq_batch)
            loss = criterion(pred, target_batch)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

            train_loss += loss.item() * seq_batch.size(0)

        train_loss /= len(train_ds)

        # Validate
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for seq_batch, target_batch in val_loader:
                seq_batch = seq_batch.to(device)
                target_batch = target_batch.to(device)
                pred = model(seq_batch)
                loss = criterion(pred, target_batch)
                val_loss += loss.item() * seq_batch.size(0)
        val_loss /= len(val_ds)

        scheduler.step(val_loss)
        dt = time.time() - t0

        logger.info(
            f"Epoch {epoch+1:3d}/{NUM_EPOCHS} | "
            f"train_loss={train_loss:.6f} | val_loss={val_loss:.6f} | "
            f"lr={optimizer.param_groups[0]['lr']:.2e} | {dt:.1f}s"
        )

        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            torch.save({
                'model_state_dict': model.state_dict(),
                'epoch': epoch,
                'val_loss': val_loss,
                'state_mean': state_mean.tolist(),
                'state_std': state_std.tolist(),
                'config': {
                    'input_size': FEATURE_DIM,
                    'hidden_size': HIDDEN_SIZE,
                    'num_layers': NUM_LAYERS,
                    'sequence_length': SEQUENCE_LENGTH,
                    'output_size': STATE_DIM,
                    'dropout': DROPOUT,
                }
            }, f"{output_dir}/best_model.pth")
            logger.info(f"  -> Saved best model (val_loss={val_loss:.6f})")
        else:
            patience_counter += 1
            if patience_counter >= PATIENCE:
                logger.info(f"Early stopping at epoch {epoch+1}")
                break

    logger.info(f"Training complete. Best val_loss={best_val_loss:.6f}")
    logger.info(f"Model saved to {output_dir}/best_model.pth")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, help='Path to collected data directory')
    parser.add_argument('--output', default=None, help='Output directory for model')
    args = parser.parse_args()
    train(args.data, args.output)
