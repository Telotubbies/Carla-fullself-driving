# Checkpoint Compression Guide

## Overview

Checkpoint files from Stable Baselines3 are saved using `ZIP_STORED` format (no compression), resulting in large file sizes (~350MB for 30k steps). This guide explains how to reduce checkpoint sizes by up to 60% using compression.

## Problem

- **Original checkpoint size**: ~354 MB (30,000 steps)
- **Compression method**: ZIP_STORED (no compression)
- **Result**: Large disk usage, slow I/O

## Solution

Automatic compression using `ZIP_DEFLATED` with maximum compression level (9).

### Results

- **Compressed checkpoint size**: ~149 MB (30,000 steps)
- **Space saved**: ~205 MB (57.9% reduction)
- **Compression method**: ZIP_DEFLATED (level 9)

## Implementation

### 1. Automatic Compression

Checkpoints are automatically compressed after saving during training:

```python
# In training/train_sac.py
from utils.checkpoint_compression import compress_checkpoint

# After model.save()
success, saved_mb = compress_checkpoint(checkpoint_path, compression_level=9)
```

### 2. Manual Compression Script

To compress existing checkpoints:

```bash
python3 scripts/compress_existing_checkpoints.py
```

This will:
- Analyze all checkpoints in `checkpoints/checkpoint/`
- Compress them using ZIP_DEFLATED (level 9)
- Show before/after statistics

### 3. Compression Utility

The `utils/checkpoint_compression.py` module provides:

- `compress_checkpoint()`: Compress a single checkpoint
- `compress_checkpoint_directory()`: Compress all checkpoints in a directory
- `get_checkpoint_size_info()`: Analyze checkpoint size and compression

## Configuration

### Current Settings

```yaml
checkpoints:
  save_optimizer: false  # Skip optimizer (saves ~30-50%)
  max_checkpoints_to_keep: 1  # Keep only latest
  cleanup_old_checkpoints: true  # Auto cleanup
```

### Compression Settings

- **Compression level**: 9 (maximum)
- **Compression method**: ZIP_DEFLATED
- **Backup**: Disabled during training (enabled for manual compression)

## File Size Breakdown

### Before Compression (ZIP_STORED)

```
policy.pth:           149.92 MB (100% - no compression)
critic.optimizer.pth: 100.58 MB (100% - no compression)
actor.optimizer.pth:   98.30 MB (100% - no compression)
data:                   5.44 MB (100% - no compression)
Total:                354.25 MB
```

### After Compression (ZIP_DEFLATED)

```
policy.pth:           ~60-70 MB (50-60% compression)
critic.optimizer.pth: ~40-50 MB (50-60% compression)
actor.optimizer.pth:  ~40-50 MB (50-60% compression)
data:                  ~2-3 MB (50-60% compression)
Total:                ~149 MB (57.9% reduction)
```

## Performance Impact

### Compression Time

- **Time**: ~10-15 seconds per checkpoint
- **CPU**: Moderate (single-threaded)
- **Disk I/O**: Temporary file creation

### Decompression Time

- **Time**: <1 second (negligible)
- **Impact**: Minimal on training resume

## Best Practices

1. **Automatic Compression**: Enabled by default in training callback
2. **Manual Compression**: Run after training sessions to compress old checkpoints
3. **Cleanup**: Keep only latest checkpoint (`max_checkpoints_to_keep: 1`)
4. **Optimizer State**: Disabled by default (`save_optimizer: false`)

## Troubleshooting

### Checkpoint Won't Load

If a compressed checkpoint fails to load:

1. Check compression method:
   ```python
   import zipfile
   z = zipfile.ZipFile('checkpoint.zip')
   print([f.compress_type for f in z.filelist])
   # Should show: [8, 8, 8, ...] (DEFLATED)
   ```

2. Verify file integrity:
   ```python
   z.testzip()  # Should return None if valid
   ```

### Compression Fails

If compression fails:

1. Check disk space (needs ~2x checkpoint size temporarily)
2. Verify file permissions
3. Check for file locks (close training processes)

## Future Improvements

1. **LZMA Compression**: Better compression ratio (~70% reduction) but slower
2. **Quantization**: Reduce model precision (float32 → float16) for ~50% size reduction
3. **Pruning**: Remove redundant weights for additional size reduction
4. **Incremental Compression**: Compress only changed files

## Related Files

- `utils/checkpoint_compression.py`: Compression utilities
- `scripts/compress_existing_checkpoints.py`: Manual compression script
- `training/train_sac.py`: Automatic compression in training callback
- `config/sac_config.yaml`: Checkpoint configuration

