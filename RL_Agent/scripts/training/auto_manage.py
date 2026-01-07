#!/usr/bin/env python3
"""
Auto Management Script (Refactored)
Replaces the monolithic bash script with robust Python process management.
"""

import os
import sys
import time
import signal
import logging
import psutil
import subprocess
import yaml
from pathlib import Path
from datetime import datetime
import carla
import glob
import re
import socket
import urllib.request
import json

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# Directories
BASE_DIR = Path("/home/a/Desktop/CARLA_0.9.16/RL_Agent")
CARLA_DIR = Path("/home/a/Desktop/CARLA_0.9.16")
LOG_DIR = BASE_DIR / "logs"
CHECKPOINT_DIR = BASE_DIR / "checkpoints_new/checkpoint"
CONFIG_FILE = BASE_DIR / "config/phase1_reward_optimized.yaml"

# Log Files
AUTO_LOG_FILE = LOG_DIR / "auto_manage.log"
TRAINING_LOG = LOG_DIR / "rl_training_new.log"
PID_FILE = BASE_DIR / ".auto_manage.pid"

# Settings
CARLA_PORT = 2000
HEALTH_CHECK_INTERVAL = 30
STUCK_THRESHOLD = 1800  # 30 minutes
CARLA_FAILURE_THRESHOLD = 3
CARLA_COOLDOWN_PERIOD = 600

# Setup Logging
logging.basicConfig(
    filename=AUTO_LOG_FILE,
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("AutoManage")
# Also print to stdout for manual running
console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
console.setFormatter(formatter)
logger.addHandler(console)

# ==============================================================================
# PROCESS UTILS (Safe Re-implementation of Bash Logic)
# ==============================================================================

def is_kernel_process(proc: psutil.Process) -> bool:
    try:
        if proc.pid == 0: return True
        # Kernel threads usually have ppid=2 or 0 (on Linux)
        if proc.ppid() in [0, 2]: return True
        return False
    except:
        return True

def safe_kill(proc: psutil.Process, reason: str):
    """Safely kill a process with timeout."""
    if is_kernel_process(proc):
        logger.warning(f"Refusing to kill kernel process {proc.pid}")
        return

    logger.warning(f"Killing process {proc.pid} ({proc.name()}). Reason: {reason}")
    try:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except psutil.TimeoutExpired:
            logger.warning(f"Process {proc.pid} did not terminate, forcing kill.")
            proc.kill()
            proc.wait(timeout=2)
    except psutil.NoSuchProcess:
        pass
    except Exception as e:
        logger.error(f"Error killing process {proc.pid}: {e}")

def find_processes(pattern: str) -> list[psutil.Process]:
    """Find processes matching command line pattern."""
    matches = []
    for p in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if p.cmdline() and any(pattern in c for c in p.cmdline()):
                 matches.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return matches

def cleanup_zombies():
    """Clean up zombie processes related to our project."""
    # psutil automatically handles some zombie logic, but we can explicitly look for them
    # Python generally reaps its own children, but we'll check system-wide if needed
    # (Simplified: In Python, calling process_iter() often refreshes the process table)
    pass # Modern psutil handles this better than bash scripting

# ==============================================================================
# COMPONENT CHECKS
# ==============================================================================

def check_carla_status() -> bool:
    """Check if CARLA server is reachable."""
    try:
        client = carla.Client('localhost', CARLA_PORT)
        client.set_timeout(5.0)
        world = client.get_world()
        _ = world.get_map() # Verify fully loaded
        return True
    except Exception:
        return False

def is_training_running() -> bool:
    """Check if train.py is running."""
    # Look for python process with 'train.py' in cmdline
    procs = find_processes("train.py")
    # Filter for our specific training script just in case
    return len(procs) > 0

def check_dashboard_status() -> bool:
    """Check if dashboard is responding."""
    try:
        with urllib.request.urlopen('http://localhost:5000/api/status', timeout=2):
            return True
    except:
        return False

# ==============================================================================
# LOGIC & ACTIONS
# ==============================================================================

def get_latest_checkpoint() -> str:
    """
    Find the latest checkpoint based on highest timestep (step count).
    
    ⚠️ IMPORTANT: For RL training, we use LAST checkpoint (highest timestep),
    NOT best checkpoint by reward. This is because RL algorithms continuously
    improve and the latest checkpoint contains the most recent learning progress.
    """
    # Check both checkpoint directories (new and old)
    checkpoint_dirs = [
        BASE_DIR / "checkpoints_new" / "checkpoint",
        CHECKPOINT_DIR
    ]
    
    all_zips = []
    seen_checkpoints = set()  # Track by name to avoid duplicates
    for checkpoint_dir in checkpoint_dirs:
        if checkpoint_dir.exists():
            zips = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
            for z in zips:
                # Only add if we haven't seen this checkpoint name before
                if z.name not in seen_checkpoints:
                    all_zips.append(z)
                    seen_checkpoints.add(z.name)
    
    if not all_zips: 
        return ""
    
    # Sort by step count (timestep) - get the one with highest timestep
    def get_steps(p):
        m = re.search(r'rl_model_(\d+)_steps', p.name)
        return int(m.group(1)) if m else -1
    
    # Filter valid checkpoints (must have non-empty config)
    valid_zips = []
    warned_checkpoints = set()  # Track warned checkpoints to avoid duplicate warnings
    for z in all_zips:
        config_path = z.parent / z.name.replace(".zip", "_config.yaml")
        if config_path.exists() and config_path.stat().st_size > 0:
            valid_zips.append(z)
        else:
            # Only warn once per checkpoint name
            if z.name not in warned_checkpoints:
                logger.warning(f"Ignoring checkpoint {z.name} (config missing or empty)")
                warned_checkpoints.add(z.name)
            
    if not valid_zips: 
        return ""
        
    # Get checkpoint with highest timestep (last checkpoint)
    latest = max(valid_zips, key=get_steps)
    latest_steps = get_steps(latest)
    logger.info(f"📂 Found latest checkpoint: {latest.name} (timestep: {latest_steps:,})")
    return str(latest)

def get_best_checkpoint_by_reward(min_reward: float = 0.0) -> str:
    """
    Find the best checkpoint(s) by reward (for use as base model).
    Falls back to latest checkpoint if no good reward found.
    
    Args:
        min_reward: Minimum reward threshold (default: 0.0 for positive rewards)
    
    Returns:
        Path to best checkpoint, or latest if no good reward found
    """
    # First try SQLite database
    db_path = BASE_DIR / "checkpoints_new" / "training_checkpoints.db"
    if db_path.exists():
        try:
            sys.path.insert(0, str(BASE_DIR))
            from utils.sqlite_checkpoint import SQLiteCheckpointManager
            manager = SQLiteCheckpointManager(str(db_path))
            best_checkpoints = manager.get_best_checkpoint_info(min_reward=min_reward, limit=1)
            manager.close()
            
            if best_checkpoints and len(best_checkpoints) > 0:
                best = best_checkpoints[0]
                timestep = best['timestep']
                reward = best.get('reward', 0)
                
                # Find corresponding zip file
                checkpoint_dirs = [
                    BASE_DIR / "checkpoints_new" / "checkpoint",
                    CHECKPOINT_DIR
                ]
                
                for checkpoint_dir in checkpoint_dirs:
                    if checkpoint_dir.exists():
                        zip_file = checkpoint_dir / f"rl_model_{timestep}_steps.zip"
                        if zip_file.exists():
                            logger.info(f"⭐ Found best checkpoint by reward: {zip_file.name} (reward: {reward:.2f}, timestep: {timestep})")
                            return str(zip_file)
        except Exception as e:
            logger.warning(f"Failed to query SQLite for best checkpoint: {e}")
    
    # Fallback: search zip files and check config files for reward
    checkpoint_dirs = [
        BASE_DIR / "checkpoints_new" / "checkpoint",
        CHECKPOINT_DIR
    ]
    
    best_reward = float('-inf')
    best_checkpoint = None
    
    for checkpoint_dir in checkpoint_dirs:
        if checkpoint_dir.exists():
            zips = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
            for z in zips:
                # Try to read reward from config file
                config_file = z.parent / z.name.replace(".zip", "_config.yaml")
                if config_file.exists() and config_file.stat().st_size > 0:
                    try:
                        import yaml
                        with open(config_file, 'r') as f:
                            config = yaml.safe_load(f)
                            # Check if reward is stored in metadata
                            reward = config.get('metadata', {}).get('reward')
                            if reward is not None and reward >= min_reward and reward > best_reward:
                                best_reward = reward
                                best_checkpoint = z
                    except:
                        pass
    
    if best_checkpoint:
        logger.info(f"⭐ Found best checkpoint by reward: {best_checkpoint.name} (reward: {best_reward:.2f})")
        return str(best_checkpoint)
    
    # Fallback to latest checkpoint
    logger.info("No checkpoint with good reward found, using latest checkpoint")
    return get_latest_checkpoint()

def start_carla():
    logger.info("Starting CARLA...")
    
    # Check if already running but effectively dead
    carla_procs = find_processes("CarlaUE4")
    if carla_procs:
        if check_carla_status():
            logger.info("CARLA is already running and responding.")
            return
        else:
            logger.warning("CARLA process found but unresponsive. Restarting...")
            for p in carla_procs: safe_kill(p, "Unresponsive CARLA")
            
    # Launch
    cmd = [
        "./CarlaUE4.sh",
        "-quality-level=Low",
        "-no-sound",
        f"-carla-port={CARLA_PORT}"
    ]
    # Check for display/xvfb logic (simplified)
    # Use DISPLAY=:0 if available, else standard environment
    env = os.environ.copy()
    if os.path.exists("/tmp/.X11-unix/X0"):
        env["DISPLAY"] = ":0"
        cmd.append("-vulkan")
    else:
        # Fallback to xvfb/opengl (assuming xvfb is set up externally or use default)
        # Using default render behavior if no :0
        pass

    try:
        subprocess.Popen(
            cmd, 
            cwd=CARLA_DIR,
            stdout=subprocess.DEVNULL, # Log to /dev/null or separate file
            stderr=subprocess.DEVNULL,
            env=env,
            start_new_session=True # Detach
        )
        logger.info("CARLA launch command issued. Waiting for readiness...")
        
        # Wait for ready
        for _ in range(60): # Wait up to 2 min
            if check_carla_status():
                logger.info("CARLA is ready.")
                return
            time.sleep(2)
        logger.error("CARLA failed to become ready.")
    except Exception as e:
        logger.error(f"Failed to launch CARLA: {e}")

def start_training():
    logger.info("Starting Training...")
    
    # Cleanup duplicates
    train_procs = find_processes("train.py")
    if train_procs:
        logger.warning(f"Found {len(train_procs)} existing training processes. Cleaning up...")
        for p in train_procs: safe_kill(p, "Restarting training")
        
    # Virtual Env Check
    venv_python = BASE_DIR / "venv/bin/python"
    python_exec = str(venv_python) if venv_python.exists() else "python3"
    
    # Checkpoint selection strategy:
    # 1. Try to use BEST checkpoint (highest reward, preferably positive) as base model
    # 2. Fall back to LATEST checkpoint (highest timestep) if no good reward found
    # This allows starting from a good base while still continuing training from latest progress
    # 
    # ⚠️ IMPORTANT: When using best checkpoint, we preserve num_timesteps from latest checkpoint
    # to ensure learning rate schedule and training progress continue correctly.
    use_best_as_base = os.environ.get('USE_BEST_CHECKPOINT', 'true').lower() == 'true'
    
    latest_checkpoint = get_latest_checkpoint()
    
    if use_best_as_base:
        best_checkpoint = get_best_checkpoint_by_reward(min_reward=0.0)  # Prefer positive rewards
        
        # If best checkpoint is different from latest, use best as base but note latest timestep
        if best_checkpoint and best_checkpoint != latest_checkpoint:
            # Extract timesteps for comparison
            def get_timestep(path):
                if not path: return 0
                m = re.search(r'rl_model_(\d+)_steps', Path(path).name)
                return int(m.group(1)) if m else 0
            
            best_timestep = get_timestep(best_checkpoint)
            latest_timestep = get_timestep(latest_checkpoint)
            
            if best_timestep < latest_timestep:
                logger.info(f"🎯 Using best checkpoint by reward as base model")
                logger.info(f"   Best checkpoint: {Path(best_checkpoint).name} (timestep: {best_timestep:,}, reward: high)")
                logger.info(f"   Latest checkpoint: {Path(latest_checkpoint).name} (timestep: {latest_timestep:,})")
                logger.info(f"   ⚠️  Note: Training will continue from timestep {latest_timestep:,} to preserve learning progress")
                checkpoint = best_checkpoint
            else:
                # Best checkpoint is also latest, use it normally
                checkpoint = best_checkpoint
                logger.info(f"🎯 Using best checkpoint (also latest): {Path(checkpoint).name}")
        elif best_checkpoint:
            checkpoint = best_checkpoint
            logger.info(f"🎯 Using best checkpoint by reward as base model")
        else:
            checkpoint = latest_checkpoint
            logger.info(f"📂 No best checkpoint found, using latest checkpoint (highest timestep)")
    else:
        checkpoint = latest_checkpoint
        logger.info(f"📂 Using latest checkpoint (highest timestep)")
    
    # Config Logic - Always use the latest config file, not checkpoint config
    # This ensures we use the latest hyperparameters and reward shaping
    config_path = CONFIG_FILE
    logger.info(f"Using config: {config_path}")
        
    cmd = [
        python_exec,
        "training/train.py",
        "--config", str(config_path),
        "--num-envs", "1"
    ]
    
    if checkpoint:
        logger.info(f"Resuming from {Path(checkpoint).name}")
        cmd.extend(["--resume", checkpoint])
    else:
        logger.info("Starting fresh training.")
        
    try:
        env_vars = os.environ.copy()
        env_vars["PYTHONUNBUFFERED"] = "1"
        with open(TRAINING_LOG, "a") as logf:
            subprocess.Popen(
                cmd,
                cwd=BASE_DIR,
                stdout=logf,
                stderr=subprocess.STDOUT,
                env=env_vars,
                start_new_session=True
            )
        logger.info("Training process launched.")
    except Exception as e:
        logger.error(f"Failed to launch training: {e}")

def start_dashboard():
    logger.info("Starting Dashboard...")
    
    # Cleanup duplicates
    dash_procs = find_processes("web_dashboard")
    for p in dash_procs: safe_kill(p, "Restarting dashboard")
    
    venv_python = BASE_DIR / "venv/bin/python"
    python_exec = str(venv_python) if venv_python.exists() else "python3"
    
    # Try uvicorn/fastapi first
    fastapi_app = BASE_DIR / "web_dashboard/app_fastapi.py"
    if fastapi_app.exists():
         cmd = [
             str(BASE_DIR / "venv/bin/uvicorn"),
             "web_dashboard.app_fastapi:app",
             "--host", "0.0.0.0",
             "--port", "5000"
         ]
         cwd = BASE_DIR
    else:
         cmd = [python_exec, "web_dashboard/app.py"]
         cwd = BASE_DIR

    try:
        log_path = LOG_DIR / "dashboard.log"
        with open(log_path, "w") as logf:
            subprocess.Popen(
                cmd,
                cwd=cwd,
                stdout=logf,
                stderr=subprocess.STDOUT,
                start_new_session=True
            )
        time.sleep(2)
        if check_dashboard_status():
            logger.info("Dashboard is ready.")
        else:
            logger.warning("Dashboard started but not yet responding.")
    except Exception as e:
        logger.error(f"Failed to launch dashboard: {e}")

# ==============================================================================
# HEALTH CHECK
# ==============================================================================

def check_training_health():
    """Verify training is actually progressing."""
    if not is_training_running():
        logger.warning("Training process is NOT running.")
        return False
        
    if not TRAINING_LOG.exists(): return True # Just started?
    
    # Check file modification time
    mtime = TRAINING_LOG.stat().st_mtime
    age = time.time() - mtime
    if age > STUCK_THRESHOLD:
        logger.warning(f"Training log stuck. No updates for {age/60:.1f} mins.")
        return False
        
    return True

def monitor_loop():
    logger.info("🚀 Auto Management Started (Python version)")
    
    # PID file
    with open(PID_FILE, 'w') as f:
        f.write(str(os.getpid()))

    try:
        while True:
            # 1. Health Checks
            carla_ok = check_carla_status()
            training_ok = check_training_health()
            dashboard_ok = check_dashboard_status()
            
            # 2. Remediation
            if not carla_ok:
                # Special logic: If training running but CARLA failing, might be load
                # But if training relies on CARLA, it will likely fail soon or hang.
                # Use cooldown/failure thresholds (simplified here)
                start_carla()
                carla_ok = check_carla_status()
                
            if not training_ok:
                # If training failed, ensure CARLA is acceptable before restarting
                if carla_ok:
                    start_training()
                else:
                    logger.warning("Skipping training restart because CARLA is down.")
                    
            if not dashboard_ok:
                start_dashboard()
                
            # 3. Wait
            time.sleep(HEALTH_CHECK_INTERVAL)
            
    except KeyboardInterrupt:
        logger.info("Stopping...")
    except Exception as e:
        logger.error(f"Manager Loop Crashed: {e}", exc_info=True)
    finally:
        if PID_FILE.exists(): PID_FILE.unlink()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "run":
        signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))
        monitor_loop()
    else:
        # Launcher behavior
        if PID_FILE.exists():
            try:
                old_pid = int(PID_FILE.read_text().strip())
                if psutil.pid_exists(old_pid):
                    print(f"Auto Manager is already running (PID {old_pid}).")
                    sys.exit(1)
                else:
                    print("Found stale PID file. Overwriting.")
            except:
                pass
        
        print("Starting Auto Manager background process...")
        subprocess.Popen([sys.executable, __file__, "run"], start_new_session=True)
        print("Started.")

