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

BASE_DIR = Path("/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC")
CARLA_DIR = Path("/home/a/Desktop/CARLA_0.9.16")
LOG_DIR = BASE_DIR / "logs"
CHECKPOINT_DIR = BASE_DIR / "checkpoints/checkpoint"
CONFIG_FILE = BASE_DIR / "config/sac_config.yaml"
AUTO_LOG_FILE = LOG_DIR / "auto_manage.log"
TRAINING_LOG = LOG_DIR / "rl_training_new.log"
PID_FILE = BASE_DIR / ".auto_manage.pid"
CARLA_PORT = 2000
HEALTH_CHECK_INTERVAL = 30
STUCK_THRESHOLD = 1800
MIN_TRAINING_AGE = 60
CARLA_FAILURE_THRESHOLD = 3
CARLA_COOLDOWN_PERIOD = 300
DASHBOARD_PORT = 5001
RESTART_COOLDOWN = 60
logging.basicConfig(
    filename=AUTO_LOG_FILE,
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("AutoManageSAC")
console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
console.setFormatter(formatter)
logger.addHandler(console)

# Import auto cleanup manager
try:
    sys.path.insert(0, str(BASE_DIR))
    from utils.auto_cleanup import AutoCleanupManager
    CLEANUP_AVAILABLE = True
except ImportError as e:
    logger.warning(f"Auto cleanup not available: {e}")
    CLEANUP_AVAILABLE = False

def find_processes(pattern: str) -> list[psutil.Process]:
    
    matches = []
    for p in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if p.cmdline() and any(pattern in c for c in p.cmdline()):
                if 'grep' not in ' '.join(p.cmdline()).lower():
                    matches.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return matches
def get_latest_checkpoint():
    
    if not CHECKPOINT_DIR.exists():
        logger.debug(f"Checkpoint directory does not exist: {CHECKPOINT_DIR}")
        return None
    zip_files = list(CHECKPOINT_DIR.glob("rl_model_*_steps.zip"))
    if not zip_files:
        logger.debug(f"No checkpoint files found in {CHECKPOINT_DIR}")
        return None
    latest = max(zip_files, key=lambda p: p.stat().st_mtime)
    logger.info(f"✅ Found checkpoint: {latest.name} ({latest.stat().st_size / (1024*1024):.1f} MB)")
    return str(latest)
def get_best_checkpoint_by_reward(min_reward: float = -float('inf')):
    
    db_path = BASE_DIR / "checkpoints" / "training_checkpoints.db"
    if not db_path.exists():
        return None
    try:
        import sqlite3
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()
        cursor.execute('SELECT timestep, reward FROM checkpoints WHERE reward >= ? ORDER BY reward DESC LIMIT 1', (min_reward,))
        result = cursor.fetchone()
        conn.close()
        if result:
            timestep = result[0]
            checkpoint_file = CHECKPOINT_DIR / f"rl_model_{timestep}_steps.zip"
            if checkpoint_file.exists():
                return str(checkpoint_file)
    except Exception as e:
        logger.debug(f"Error reading checkpoint DB: {e}")
    return None
def check_carla_status():
    
    try:
        client = carla.Client('localhost', CARLA_PORT)
        client.set_timeout(5.0)
        world = client.get_world()
        if world is not None:
            try:
                _ = world.get_map()
                return True
            except:
                return False
        return False
    except Exception as e:
        logger.debug(f"CARLA status check failed: {e}")
        return False
def start_carla():
    
    logger.info("Starting CARLA...")
    if check_carla_status():
        logger.info("CARLA is already running.")
        return
    carla_procs = find_processes("CarlaUE4")
    if carla_procs:
        logger.info("CARLA processes found. Checking if server is ready...")
        if check_carla_status():
            logger.info("CARLA server is ready.")
            return
        logger.info("CARLA processes exist but server not ready. Will start new instance.")
    time.sleep(1)
    carla_binary = CARLA_DIR / "CarlaUE4.sh"
    if not carla_binary.exists():
        logger.error(f"CARLA binary not found: {carla_binary}")
        return
    try:
        log_path = LOG_DIR / "carla.log"
        with open(log_path, "w") as logf:
            proc = subprocess.Popen(
                [str(carla_binary)],
                cwd=str(CARLA_DIR),
                stdout=logf,
                stderr=subprocess.STDOUT,
                start_new_session=True
            )
            logger.info(f"✅ CARLA process started (PID: {proc.pid}) - GUI mode enabled")
        for i in range(30):
            time.sleep(2)
            if check_carla_status():
                logger.info("✅ CARLA started successfully.")
                return
            logger.debug(f"Waiting for CARLA... ({i+1}/30)")
        logger.warning("CARLA did not start within timeout.")
    except Exception as e:
        logger.error(f"Failed to launch CARLA: {e}")
def get_latest_training_log():
    
    sac_log_pattern = LOG_DIR / "sac_training_*.log"
    sac_logs = list(glob.glob(str(sac_log_pattern)))
    if sac_logs:
        return max(sac_logs, key=os.path.getmtime)
    elif TRAINING_LOG.exists():
        return str(TRAINING_LOG)
    return None
def extract_step_count(log_file: str) -> int:
    
    if not log_file or not os.path.exists(log_file):
        return 0
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()[-1000:]
            step_patterns = [
                r'Callback:\s+Step\s+(\d+)',
                r'Step\s+(\d+)',
                r'step\s+(\d+)',
                r'timestep[=:]\s*(\d+)',
                r'num_timesteps[=:]\s*(\d+)',
                r'Training step[=:]\s*(\d+)',
            ]
            max_step = 0
            for line in reversed(lines):
                for pattern in step_patterns:
                    match = re.search(pattern, line, re.IGNORECASE)
                    if match:
                        step = int(match.group(1))
                        max_step = max(max_step, step)
            return max_step
    except Exception as e:
        logger.debug(f"Error extracting step count: {e}")
        return 0
def check_training_progress(log_file: str, last_step: int, last_check_time: float) -> tuple:
    
    if not log_file or not os.path.exists(log_file):
        return (False, 0, False)
    current_step = extract_step_count(log_file)
    mtime = os.path.getmtime(log_file)
    age = time.time() - mtime
    has_error = False
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()[-100:]
            error_keywords = [
                'ERROR', 'Exception', 'Traceback', 'Failed', 'Crash',
                'TypeError', 'RuntimeError', 'can\'t convert'
            ]
            for line in lines:
                if any(keyword in line for keyword in error_keywords):
                    has_error = True
                    break
    except:
        pass
    is_progressing = False
    if current_step > last_step:
        is_progressing = True
    elif age < 120:
        is_progressing = True
    return (is_progressing, current_step, has_error)
def check_training_health(last_step: dict, last_check_time: dict):
    
    # Try multiple patterns to find training process
    train_procs = find_processes("train_sac.py")
    if not train_procs:
        # Also try alternative patterns
        train_procs = find_processes("train_sac")
        if not train_procs:
            train_procs = find_processes("training/train_sac")
    
    if not train_procs:
        logger.debug("No training process found with any pattern")
        return (False, True, "No training process found")
    
    # Filter active processes with better error handling
    active_procs = []
    for p in train_procs:
        try:
            if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                active_procs.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
            continue
    
    if not active_procs:
        logger.debug(f"Found {len(train_procs)} training process(es) but none are active")
        return (False, True, "No active training processes (zombie)")
    for proc in active_procs:
        try:
            proc_create_time = proc.create_time()
            proc_age = time.time() - proc_create_time
            if proc_age < 30:
                logger.debug(f"Training process {proc.pid} is very new ({proc_age:.0f}s), giving it time to initialize")
                return (True, False, f"Process is new ({proc_age:.0f}s old), initializing")
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    latest_log = get_latest_training_log()
    if not latest_log:
        log_age = 0
    else:
        log_age = time.time() - os.path.getmtime(latest_log)
    if log_age < MIN_TRAINING_AGE:
        logger.debug(f"Training is new ({log_age:.0f}s old), not checking health yet")
        return (True, False, f"Training is new ({log_age:.0f}s old)")
    cpu_usage = 0.0
    for proc in active_procs:
        try:
            cpu_usage = max(cpu_usage, proc.cpu_percent(interval=0.1))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    if latest_log:
        mtime = os.path.getmtime(latest_log)
        log_age_since_update = time.time() - mtime
    else:
        log_age_since_update = 0
    proc_id = active_procs[0].pid
    last_step_for_proc = last_step.get(proc_id, 0)
    last_check = last_check_time.get(proc_id, 0)
    is_progressing, current_step, has_error = check_training_progress(
        latest_log, last_step_for_proc, last_check
    )
    last_step[proc_id] = current_step
    last_check_time[proc_id] = time.time()
    if has_error and not is_progressing:
        if log_age_since_update > STUCK_THRESHOLD:
            return (False, True, f"Error detected and stuck for {log_age_since_update:.0f}s")
        else:
            return (True, False, "Error detected but still recent, monitoring")
    if not is_progressing:
        if log_age_since_update > STUCK_THRESHOLD and cpu_usage < 1.0:
            return (False, True, f"No progress for {log_age_since_update:.0f}s, CPU: {cpu_usage:.1f}%")
        else:
            return (True, False, f"No progress but within threshold (age: {log_age_since_update:.0f}s)")
    logger.debug(f"Training healthy: step={current_step}, CPU={cpu_usage:.1f}%, log_age={log_age_since_update:.0f}s")
    return (True, False, f"Progressing: step={current_step}, CPU={cpu_usage:.1f}%")
def start_training():
    
    logger.info("Starting SAC Training...")
    train_procs = find_processes("train_sac.py")
    if train_procs:
        active_procs = [p for p in train_procs if p.is_running()]
        if active_procs:
            logger.info(f"Found {len(active_procs)} existing active training process(es). Skipping start.")
            return
        else:
            logger.warning(f"Found {len(train_procs)} zombie training process(es). Will start new one.")
    venv_python = BASE_DIR / "venv/bin/python"
    if venv_python.exists():
        python_exec = str(venv_python)
        logger.info(f"✅ Using venv Python: {python_exec}")
    else:
        python_exec = "python3"
        logger.warning(f"⚠️  venv not found at {venv_python}, using system python3")
    use_best_as_base = os.environ.get('USE_BEST_CHECKPOINT', 'true').lower() == 'true'
    latest_checkpoint = get_latest_checkpoint()
    if use_best_as_base:
        best_checkpoint = get_best_checkpoint_by_reward(min_reward=0.0)
        if best_checkpoint and best_checkpoint != latest_checkpoint:
            def get_timestep(path):
                if not path: return 0
                m = re.search(r'rl_model_(\d+)_steps', Path(path).name)
                return int(m.group(1)) if m else 0
            best_timestep = get_timestep(best_checkpoint)
            latest_timestep = get_timestep(latest_checkpoint)
            if best_timestep < latest_timestep:
                logger.info(f"🎯 Using best checkpoint by reward as base model")
                logger.info(f"   Best checkpoint: {Path(best_checkpoint).name} (timestep: {best_timestep:,})")
                logger.info(f"   Latest checkpoint: {Path(latest_checkpoint).name} (timestep: {latest_timestep:,})")
                checkpoint = best_checkpoint
            else:
                checkpoint = best_checkpoint
                logger.info(f"🎯 Using best checkpoint (also latest): {Path(checkpoint).name}")
        elif best_checkpoint:
            checkpoint = best_checkpoint
            logger.info(f"🎯 Using best checkpoint: {Path(checkpoint).name}")
        else:
            checkpoint = latest_checkpoint
            if checkpoint:
                logger.info(f"📦 Using latest checkpoint: {Path(checkpoint).name}")
    else:
        checkpoint = latest_checkpoint
        if checkpoint:
            logger.info(f"📦 Using latest checkpoint: {Path(checkpoint).name}")
    cmd = [python_exec, "training/train_sac.py", "--config", str(CONFIG_FILE)]
    if checkpoint:
        cmd.extend(["--resume", checkpoint])
        logger.info(f"   Resuming from: {Path(checkpoint).name}")
    else:
        logger.info("   Starting new training (no checkpoint found)")
    try:
        log_path = LOG_DIR / f"sac_training_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        with open(log_path, "w") as logf:
            proc = subprocess.Popen(
                cmd,
                cwd=str(BASE_DIR),
                stdout=logf,
                stderr=subprocess.STDOUT,
                start_new_session=True
            )
            logger.info(f"✅ Training process started (PID: {proc.pid}, Python: {python_exec})")
        time.sleep(3)
    except Exception as e:
        logger.error(f"Failed to launch training: {e}")
def check_dashboard_status():
    
    try:
        response = urllib.request.urlopen(f"http://localhost:{DASHBOARD_PORT}/api/status", timeout=2)
        return response.getcode() == 200
    except Exception:
        return False
def start_dashboard():
    
    logger.info("Starting SAC Dashboard...")
    dash_procs = find_processes("app_fastapi.py")
    for p in dash_procs:
        try:
            cmdline = ' '.join(p.cmdline())
            if '5001' in cmdline or 'RL_Agent_SAC' in cmdline:
                if check_dashboard_status():
                    logger.info("Dashboard is already running and responding.")
                    return
                logger.info("Dashboard process found but not responding. Will start new instance.")
        except:
            pass
    import subprocess as sp
    venv_python = BASE_DIR / "venv/bin/python"
    if venv_python.exists():
        try:
            result = sp.run([str(venv_python), "-c", "import uvicorn"],
                                  capture_output=True, timeout=2)
            if result.returncode == 0:
                python_exec = str(venv_python)
            else:
                python_exec = "python3"
        except:
            python_exec = "python3"
    else:
        python_exec = "python3"
        logger.warning(f"⚠️  venv not found at {venv_python}, using system python3")
    # Try production dashboard first, fallback to regular
    fastapi_prod = BASE_DIR / "web_dashboard/app_fastapi_production.py"
    fastapi_app = BASE_DIR / "web_dashboard/app_fastapi.py"
    
    dashboard_file = fastapi_prod if fastapi_prod.exists() else fastapi_app
    dashboard_module = "web_dashboard.app_fastapi_production:app" if fastapi_prod.exists() else "web_dashboard.app_fastapi:app"
    
    if dashboard_file.exists():
        logger.info(f"Using dashboard: {dashboard_file.name}")
        cmd = [
            python_exec, "-m", "uvicorn",
            dashboard_module,
            "--host", "0.0.0.0",
            "--port", str(DASHBOARD_PORT)
        ]
        cwd = BASE_DIR
    else:
        logger.error(f"Dashboard not found: {dashboard_file}")
        return
    try:
        log_path = LOG_DIR / f"sac_dashboard_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        with open(log_path, "w") as logf:
            proc = sp.Popen(
                cmd,
                cwd=str(cwd),
                stdout=logf,
                stderr=sp.STDOUT,
                start_new_session=True
            )
            logger.info(f"✅ Dashboard process started (PID: {proc.pid}, Python: {python_exec})")
        time.sleep(3)
        if check_dashboard_status():
            logger.info(f"✅ Dashboard is ready on port {DASHBOARD_PORT}.")
        else:
            logger.warning(f"Dashboard started but not responding on port {DASHBOARD_PORT}.")
    except Exception as e:
        logger.error(f"Failed to launch dashboard: {e}")
def monitor_loop():
    
    logger.info("🚀 SAC Auto Management Started (Rewritten Version)")
    with open(PID_FILE, 'w') as f:
        f.write(str(os.getpid()))
    logger.info("Initial startup: Starting CARLA...")
    if not check_carla_status():
        start_carla()
        time.sleep(10)
    else:
        logger.info("CARLA is already running.")
    carla_ready = False
    for i in range(10):
        if check_carla_status():
            carla_ready = True
            logger.info("✅ CARLA is ready!")
            break
        logger.debug(f"Waiting for CARLA to be ready... ({i+1}/10)")
        time.sleep(3)
    if not carla_ready:
        logger.warning("⚠️  CARLA did not become ready, but continuing...")
    if carla_ready:
        logger.info("🚀 Starting training and dashboard immediately...")
        train_procs = find_processes("train_sac.py")
        active_train_procs = [p for p in train_procs if p.is_running()] if train_procs else []
        if not active_train_procs:
            start_training()
        else:
            logger.info(f"Training already running (PID: {active_train_procs[0].pid})")
        if not check_dashboard_status():
            start_dashboard()
        logger.info("✅ Initial startup complete")
    last_step = {}
    last_check_time = {}
    last_restart_time = 0
    last_cleanup_time = 0
    CLEANUP_INTERVAL = 3600  # Run cleanup every hour
    
    # Initialize cleanup manager
    cleanup_manager = None
    if CLEANUP_AVAILABLE:
        try:
            cleanup_manager = AutoCleanupManager(
                base_dir=BASE_DIR,
                min_free_space_gb=10.0,
                keep_latest_logs=3,
                keep_latest_checkpoints=1,
                log_retention_days=3,
                checkpoint_retention_days=7,
                cleanup_interval_hours=6
            )
            logger.info("✅ Auto cleanup manager initialized")
        except Exception as e:
            logger.warning(f"Failed to initialize cleanup manager: {e}")
    
    try:
        while True:
            # Periodic cleanup check
            if cleanup_manager:
                time_since_cleanup = time.time() - last_cleanup_time
                if time_since_cleanup >= CLEANUP_INTERVAL or cleanup_manager.needs_cleanup():
                    try:
                        logger.info("🧹 Running automatic disk cleanup...")
                        result = cleanup_manager.run_cleanup(force=False)
                        if result.get('cleaned'):
                            logger.info(f"✅ Cleanup completed: freed {result['freed_space_gb']:.2f} GB")
                        last_cleanup_time = time.time()
                    except Exception as e:
                        logger.error(f"Cleanup failed: {e}", exc_info=True)
            
            carla_ok = check_carla_status()
            training_healthy, should_restart, reason = check_training_health(last_step, last_check_time)
            dashboard_ok = check_dashboard_status()
            logger.debug(f"Health check: CARLA={carla_ok}, Training={training_healthy} ({reason}), Dashboard={dashboard_ok}")
            if not carla_ok:
                logger.warning("CARLA is down, attempting to restart...")
                start_carla()
                time.sleep(10)
                carla_ok = check_carla_status()
                if carla_ok:
                    logger.info("✅ CARLA restarted successfully")
                else:
                    logger.error("❌ Failed to restart CARLA")
            if not training_healthy and should_restart:
                time_since_restart = time.time() - last_restart_time
                if time_since_restart < RESTART_COOLDOWN:
                    logger.info(f"Skipping restart (cooldown: {RESTART_COOLDOWN - time_since_restart:.0f}s remaining)")
                elif carla_ok:
                    # Double-check with multiple patterns before restarting
                    train_procs = find_processes("train_sac.py")
                    if not train_procs:
                        train_procs = find_processes("train_sac")
                    if not train_procs:
                        train_procs = find_processes("training/train_sac")
                    
                    active_procs = []
                    for p in train_procs:
                        try:
                            if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                                active_procs.append(p)
                        except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                            continue
                    
                    if not active_procs:
                        logger.warning(f"Training unhealthy: {reason}. Restarting...")
                        start_training()
                        last_restart_time = time.time()
                        last_step.clear()
                        last_check_time.clear()
                    else:
                        logger.info(f"Training unhealthy but {len(active_procs)} active process(es) found (PIDs: {[p.pid for p in active_procs]}). Monitoring...")
                else:
                    logger.warning("Skipping training restart because CARLA is down.")
            if not dashboard_ok:
                start_dashboard()
            time.sleep(HEALTH_CHECK_INTERVAL)
    except KeyboardInterrupt:
        logger.info("Stopping...")
    except Exception as e:
        logger.error(f"Manager Loop Crashed: {e}", exc_info=True)
    finally:
        if PID_FILE.exists():
            PID_FILE.unlink()
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "run":
        signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))
        monitor_loop()
    else:
        auto_manage_procs = find_processes("auto_manage.py")
        running_healthy = False
        current_pid = os.getpid()
        for proc in auto_manage_procs:
            try:
                if "RL_Agent_SAC" in ' '.join(proc.cmdline() or []):
                    # Skip self
                    if proc.pid == current_pid:
                        continue
                    try:
                        proc_status = proc.status()
                        proc_memory = proc.memory_info().rss / 1024 / 1024
                        # Check if process is actually running monitor_loop (has "run" argument)
                        cmdline_str = ' '.join(proc.cmdline() or [])
                        is_monitor_loop = 'run' in cmdline_str or 'monitor_loop' in cmdline_str
                        # Only consider healthy if it's actually running the monitor loop
                        if proc_status in ['running', 'sleeping'] and proc_memory > 1 and is_monitor_loop:
                            # Double check: verify it's actually doing work (check log activity)
                            try:
                                log_mtime = os.path.getmtime(AUTO_LOG_FILE) if AUTO_LOG_FILE.exists() else 0
                                time_since_log = time.time() - log_mtime
                                # If log hasn't been updated in last 5 minutes, process might be stuck
                                if time_since_log < 300:
                                    running_healthy = True
                                    logger.info(f"Found healthy auto_manage process PID {proc.pid}, keeping it")
                                    print("Auto_manage is already running and healthy. Skipping restart.")
                                    break
                                else:
                                    logger.warning(f"Found auto_manage process PID {proc.pid} but log stale ({time_since_log:.0f}s), will restart")
                            except:
                                running_healthy = True
                                logger.info(f"Found healthy auto_manage process PID {proc.pid}, keeping it")
                                break
                    except:
                        pass
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        if not running_healthy:
            if PID_FILE.exists():
                try:
                    old_pid = int(PID_FILE.read_text().strip())
                    if psutil.pid_exists(old_pid):
                        print(f"Found existing PID file (PID {old_pid}), but process may be stale.")
                        PID_FILE.unlink()
                except:
                    pass
            print("Starting SAC Auto Manager background process...")
            subprocess.Popen([sys.executable, __file__, "run"], start_new_session=True)
            print("Started.")