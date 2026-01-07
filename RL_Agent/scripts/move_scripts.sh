#!/bin/bash
# Move scripts to organized directories

SCRIPT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent/scripts"

# Training scripts
mv -f "${SCRIPT_DIR}/auto_manage.sh" "${SCRIPT_DIR}/training/" 2>/dev/null
mv -f "${SCRIPT_DIR}/auto_manage.py" "${SCRIPT_DIR}/training/" 2>/dev/null
mv -f "${SCRIPT_DIR}/manage_training.sh" "${SCRIPT_DIR}/training/" 2>/dev/null
mv -f "${SCRIPT_DIR}/master_pipeline.sh" "${SCRIPT_DIR}/training/" 2>/dev/null
mv -f "${SCRIPT_DIR}/stop_auto_manage.sh" "${SCRIPT_DIR}/training/" 2>/dev/null

# Monitoring scripts
mv -f "${SCRIPT_DIR}/check_processes.sh" "${SCRIPT_DIR}/monitoring/" 2>/dev/null
mv -f "${SCRIPT_DIR}/check_system_health.sh" "${SCRIPT_DIR}/monitoring/" 2>/dev/null
mv -f "${SCRIPT_DIR}/monitor_system_stability.sh" "${SCRIPT_DIR}/monitoring/" 2>/dev/null
mv -f "${SCRIPT_DIR}/system_watchdog.sh" "${SCRIPT_DIR}/monitoring/" 2>/dev/null
mv -f "${SCRIPT_DIR}/process_status_logger.sh" "${SCRIPT_DIR}/monitoring/" 2>/dev/null
mv -f "${SCRIPT_DIR}/view_status_log.sh" "${SCRIPT_DIR}/monitoring/" 2>/dev/null

# System scripts
mv -f "${SCRIPT_DIR}/fix_7800xt_primary.sh" "${SCRIPT_DIR}/system/" 2>/dev/null
mv -f "${SCRIPT_DIR}/fix_gpu_primary.sh" "${SCRIPT_DIR}/system/" 2>/dev/null
mv -f "${SCRIPT_DIR}/fix_system_stability.sh" "${SCRIPT_DIR}/system/" 2>/dev/null
mv -f "${SCRIPT_DIR}/fix_ubuntu_gpu.sh" "${SCRIPT_DIR}/system/" 2>/dev/null
mv -f "${SCRIPT_DIR}/set_gpu_power_limit.sh" "${SCRIPT_DIR}/system/" 2>/dev/null
mv -f "${SCRIPT_DIR}/setup_anti_freeze.sh" "${SCRIPT_DIR}/system/" 2>/dev/null

# Evaluation scripts
mv -f "${SCRIPT_DIR}/evaluate_model.py" "${SCRIPT_DIR}/evaluation/" 2>/dev/null
mv -f "${SCRIPT_DIR}/run_evaluation.sh" "${SCRIPT_DIR}/evaluation/" 2>/dev/null
mv -f "${SCRIPT_DIR}/run_agent.sh" "${SCRIPT_DIR}/evaluation/" 2>/dev/null

# Move README files
mv -f "${SCRIPT_DIR}/README_AUTO_MANAGE.md" "${SCRIPT_DIR}/training/" 2>/dev/null
mv -f "${SCRIPT_DIR}/README_MANAGE.md" "${SCRIPT_DIR}/training/" 2>/dev/null
mv -f "${SCRIPT_DIR}/README_STOP_AUTO.md" "${SCRIPT_DIR}/training/" 2>/dev/null

echo "✅ Scripts organized"
