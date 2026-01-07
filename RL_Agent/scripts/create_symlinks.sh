#!/bin/bash
# Create symlinks for backward compatibility

SCRIPT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent/scripts"
cd "${SCRIPT_DIR}"

# Training symlinks
ln -sf training/auto_manage.sh auto_manage.sh 2>/dev/null
ln -sf training/auto_manage.py auto_manage.py 2>/dev/null
ln -sf training/manage_training.sh manage_training.sh 2>/dev/null

# Most commonly used scripts
ln -sf monitoring/check_processes.sh check_processes.sh 2>/dev/null
ln -sf monitoring/view_status_log.sh view_status_log.sh 2>/dev/null

echo "✅ Symlinks created for backward compatibility"
