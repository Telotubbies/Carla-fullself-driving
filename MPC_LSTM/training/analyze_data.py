"""
Data Analysis Script - วิเคราะห์ข้อมูลก่อน training.

แสดง statistics, distributions, correlations
"""

import sys
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import json

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from training.data_preprocessing import DataPreprocessor


def analyze_data(data_dir: str, output_dir: str = None):
    """
    Analyze dataset.
    
    Args:
        data_dir: Directory containing data.csv
        output_dir: Output directory for plots
    """
    data_dir = Path(data_dir)
    
    if output_dir is None:
        output_dir = data_dir / "analysis"
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Load data
    csv_path = data_dir / "data.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"data.csv not found in {data_dir}")
    
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} rows")
    
    # Basic statistics
    print("\n" + "="*50)
    print("Basic Statistics")
    print("="*50)
    print(df.describe())
    
    # State statistics
    state_cols = ['x', 'y', 'yaw', 'velocity']
    if all(col in df.columns for col in state_cols):
        print("\n" + "="*50)
        print("State Statistics")
        print("="*50)
        states = df[state_cols]
        print(states.describe())
        
        # Plot distributions
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        for idx, col in enumerate(state_cols):
            ax = axes[idx // 2, idx % 2]
            states[col].hist(bins=50, ax=ax)
            ax.set_title(f'{col} Distribution')
            ax.set_xlabel(col)
            ax.set_ylabel('Frequency')
        plt.tight_layout()
        plt.savefig(output_dir / "state_distributions.png")
        print(f"✅ Saved state distributions to {output_dir / 'state_distributions.png'}")
    
    # Control statistics
    control_cols = ['steering', 'throttle', 'brake']
    if all(col in df.columns for col in control_cols):
        print("\n" + "="*50)
        print("Control Statistics")
        print("="*50)
        controls = df[control_cols]
        print(controls.describe())
        
        # Plot distributions
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        for idx, col in enumerate(control_cols):
            controls[col].hist(bins=50, ax=axes[idx])
            axes[idx].set_title(f'{col} Distribution')
            axes[idx].set_xlabel(col)
            axes[idx].set_ylabel('Frequency')
        plt.tight_layout()
        plt.savefig(output_dir / "control_distributions.png")
        print(f"✅ Saved control distributions to {output_dir / 'control_distributions.png'}")
    
    # Correlation matrix
    if all(col in df.columns for col in state_cols + control_cols):
        print("\n" + "="*50)
        print("Correlation Matrix")
        print("="*50)
        corr = df[state_cols + control_cols].corr()
        print(corr)
        
        # Plot correlation
        fig, ax = plt.subplots(figsize=(10, 8))
        im = ax.imshow(corr, cmap='coolwarm', aspect='auto', vmin=-1, vmax=1)
        ax.set_xticks(range(len(corr.columns)))
        ax.set_yticks(range(len(corr.columns)))
        ax.set_xticklabels(corr.columns, rotation=45, ha='right')
        ax.set_yticklabels(corr.columns)
        ax.set_title('Correlation Matrix')
        
        # Add text annotations
        for i in range(len(corr.columns)):
            for j in range(len(corr.columns)):
                text = ax.text(j, i, f'{corr.iloc[i, j]:.2f}',
                             ha="center", va="center", color="black")
        
        plt.colorbar(im)
        plt.tight_layout()
        plt.savefig(output_dir / "correlation_matrix.png")
        print(f"✅ Saved correlation matrix to {output_dir / 'correlation_matrix.png'}")
    
    # Time series plots
    if 'step' in df.columns:
        fig, axes = plt.subplots(4, 1, figsize=(12, 12))
        for idx, col in enumerate(state_cols):
            if col in df.columns:
                axes[idx].plot(df['step'], df[col], alpha=0.5)
                axes[idx].set_title(f'{col} over Time')
                axes[idx].set_xlabel('Step')
                axes[idx].set_ylabel(col)
        plt.tight_layout()
        plt.savefig(output_dir / "state_timeseries.png")
        print(f"✅ Saved time series to {output_dir / 'state_timeseries.png'}")
    
    # Save summary
    summary = {
        'total_rows': len(df),
        'state_stats': df[state_cols].describe().to_dict() if all(col in df.columns for col in state_cols) else {},
        'control_stats': df[control_cols].describe().to_dict() if all(col in df.columns for col in control_cols) else {},
    }
    
    summary_path = output_dir / "summary.json"
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"✅ Saved summary to {summary_path}")
    
    print("\n" + "="*50)
    print("✅ Analysis complete!")
    print(f"   Output directory: {output_dir}")
    print("="*50)


def main():
    parser = argparse.ArgumentParser(description='Analyze training data')
    parser.add_argument('data_dir', type=str, help='Directory containing data.csv')
    parser.add_argument('--output', type=str, default=None, help='Output directory')
    
    args = parser.parse_args()
    
    analyze_data(args.data_dir, args.output)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

