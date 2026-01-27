#!/usr/bin/env python3
"""
Comprehensive System Check Before Running Training
ตรวจสอบระบบทั้งหมดก่อนเริ่ม training
"""
import os
import sys
import subprocess
import psutil
from pathlib import Path
from typing import Dict, List, Tuple

BASE_DIR = Path("/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC")
CARLA_DIR = Path("/home/a/Desktop/CARLA_0.9.16")

# Colors
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color

errors = 0
warnings = 0

def check_pass(msg: str):
    print(f"{Colors.GREEN}✅ PASS{Colors.NC}: {msg}")

def check_fail(msg: str):
    global errors
    print(f"{Colors.RED}❌ FAIL{Colors.NC}: {msg}")
    errors += 1

def check_warn(msg: str):
    global warnings
    print(f"{Colors.YELLOW}⚠️  WARN{Colors.NC}: {msg}")
    warnings += 1

def check_info(msg: str):
    print(f"{Colors.BLUE}ℹ️  INFO{Colors.NC}: {msg}")

def check_section(title: str):
    print(f"\n{'━' * 63}")
    print(title)
    print(f"{'━' * 63}")

def check_directories():
    """Check directory structure"""
    check_section("1. DIRECTORY STRUCTURE")
    
    dirs_to_check = [
        (BASE_DIR, "Base directory"),
        (CARLA_DIR, "CARLA directory"),
        (BASE_DIR / "carla_env", "carla_env/"),
        (BASE_DIR / "models", "models/"),
        (BASE_DIR / "training", "training/"),
        (BASE_DIR / "utils", "utils/"),
        (BASE_DIR / "config", "config/"),
        (BASE_DIR / "checkpoints", "checkpoints/"),
        (BASE_DIR / "logs", "logs/"),
    ]
    
    for path, name in dirs_to_check:
        if path.exists():
            check_pass(f"{name} exists")
        else:
            check_fail(f"{name} missing: {path}")

def check_config():
    """Check configuration files"""
    check_section("2. CONFIGURATION FILES")
    
    config_file = BASE_DIR / "config" / "sac_config.yaml"
    if not config_file.exists():
        check_fail("Config file missing: config/sac_config.yaml")
        return
    
    check_pass("Config file exists: config/sac_config.yaml")
    
    # Read config
    try:
        import yaml
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        checkpoint_config = config.get('checkpoints', {})
        
        # Check save_optimizer
        save_optimizer = checkpoint_config.get('save_optimizer', True)
        if not save_optimizer:
            check_pass("save_optimizer: false (disk space optimized)")
        else:
            check_warn("save_optimizer not set to false (may use more disk space)")
        
        # Check max_checkpoints
        max_cp = checkpoint_config.get('max_checkpoints_to_keep', 1)
        if max_cp == 1:
            check_pass(f"max_checkpoints_to_keep: {max_cp} (disk space optimized)")
        else:
            check_warn(f"max_checkpoints_to_keep: {max_cp} (consider setting to 1)")
        
        # Check save_freq
        save_freq = checkpoint_config.get('save_freq', 1000)
        check_info(f"Checkpoint save frequency: {save_freq} steps")
        if save_freq >= 2000:
            check_pass("Save frequency >= 2000 (good for disk space)")
        else:
            check_warn("Save frequency < 2000 (may use more disk space)")
            
    except Exception as e:
        check_warn(f"Could not parse config: {e}")

def check_python():
    """Check Python environment"""
    check_section("3. PYTHON ENVIRONMENT")
    
    # Check Python version
    python_version = sys.version_info
    check_pass(f"Python version: {python_version.major}.{python_version.minor}.{python_version.micro}")
    
    if python_version.major >= 3 and python_version.minor >= 8:
        check_pass("Python version >= 3.8")
    else:
        check_fail("Python version < 3.8 (need 3.8+)")
    
    # Check virtual environment
    venv_path = BASE_DIR / "venv"
    if venv_path.exists():
        check_pass("Virtual environment exists: venv/")
        if (venv_path / "bin" / "activate").exists():
            check_pass("venv activation script exists")
    else:
        check_warn("Virtual environment not found (recommended to use venv)")
    
    # Check requirements
    req_file = BASE_DIR / "requirements.txt"
    if req_file.exists():
        check_pass("requirements.txt exists")
        with open(req_file, 'r') as f:
            req_count = len([l for l in f if l.strip() and not l.startswith('#')])
        check_info(f"Found {req_count} dependencies in requirements.txt")
    else:
        check_warn("requirements.txt not found")

def check_modules():
    """Check Python modules"""
    check_section("4. KEY PYTHON MODULES")
    
    key_modules = {
        'torch': 'PyTorch',
        'numpy': 'NumPy',
        'gymnasium': 'Gymnasium',
        'stable_baselines3': 'Stable-Baselines3',
        'carla': 'CARLA',
        'yaml': 'PyYAML',
        'psutil': 'psutil',
    }
    
    for module, name in key_modules.items():
        try:
            mod = __import__(module)
            version = getattr(mod, '__version__', 'unknown')
            check_pass(f"{name} installed (version: {version})")
        except ImportError:
            check_fail(f"{name} ({module}) not installed")

def check_carla():
    """Check CARLA simulator"""
    check_section("5. CARLA SIMULATOR")
    
    carla_exec = CARLA_DIR / "CarlaUE4.sh"
    if carla_exec.exists():
        check_pass("CARLA executable exists: CarlaUE4.sh")
        if os.access(carla_exec, os.X_OK):
            check_pass("CARLA executable is executable")
        else:
            check_warn("CARLA executable not executable (chmod +x needed?)")
    else:
        check_fail("CARLA executable missing: CarlaUE4.sh")
    
    # Check if CARLA is running
    carla_running = any('CarlaUE4' in p.name() for p in psutil.process_iter(['name']))
    if carla_running:
        check_info("CARLA process is running")
        # Check port
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(('localhost', 2000))
            sock.close()
            if result == 0:
                check_pass("CARLA server listening on port 2000")
            else:
                check_warn("CARLA running but port 2000 not accessible")
        except:
            pass
    else:
        check_info("CARLA not running (will be started by auto_manage)")
    
    # Check PythonAPI
    pythonapi = CARLA_DIR / "PythonAPI" / "carla"
    if pythonapi.exists():
        check_pass("CARLA PythonAPI exists")
        wheel_files = list((CARLA_DIR / "PythonAPI" / "carla" / "dist").glob("carla-0.9.16*.whl"))
        if wheel_files:
            check_pass(f"CARLA Python wheel found: {wheel_files[0].name}")
        else:
            check_warn("CARLA Python wheel not found (may need to install)")
    else:
        check_fail("CARLA PythonAPI missing")

def check_disk():
    """Check disk space"""
    check_section("6. DISK SPACE")
    
    disk = psutil.disk_usage(BASE_DIR)
    total_gb = disk.total / (1024**3)
    used_gb = disk.used / (1024**3)
    free_gb = disk.free / (1024**3)
    percent = (disk.used / disk.total) * 100
    
    check_info(f"Disk space: {used_gb:.1f}GB used / {total_gb:.1f}GB total ({percent:.1f}% used)")
    check_info(f"Available: {free_gb:.1f}GB")
    
    if percent < 80:
        check_pass("Disk usage < 80%")
    elif percent < 90:
        check_warn("Disk usage >= 80% (consider cleanup)")
    else:
        check_fail("Disk usage >= 90% (cleanup required!)")
    
    # Check checkpoint database
    db_file = BASE_DIR / "checkpoints" / "training_checkpoints.db"
    if db_file.exists():
        db_size = db_file.stat().st_size
        db_size_mb = db_size / (1024**2)
        check_info(f"Checkpoint DB size: {db_size_mb:.1f} MB")
        
        if db_size_mb < 1000:
            check_pass("Database size < 1GB")
        elif db_size_mb < 5000:
            check_warn("Database size >= 1GB (consider cleanup)")
        else:
            check_fail("Database size >= 5GB (cleanup required!)")
    else:
        check_info("Checkpoint database not found (will be created)")
    
    # Check checkpoint files
    cp_dir = BASE_DIR / "checkpoints" / "checkpoint"
    if cp_dir.exists():
        cp_files = list(cp_dir.glob("*.zip"))
        cp_count = len(cp_files)
        cp_size = sum(f.stat().st_size for f in cp_files) / (1024**2)
        check_info(f"Checkpoint files: {cp_count} files ({cp_size:.1f} MB total)")
        
        if cp_count <= 5:
            check_pass("Checkpoint count <= 5")
        elif cp_count <= 20:
            check_warn("Checkpoint count > 5 (consider cleanup)")
        else:
            check_fail("Checkpoint count > 20 (cleanup required!)")

def check_resources():
    """Check system resources"""
    check_section("7. SYSTEM RESOURCES")
    
    # RAM
    mem = psutil.virtual_memory()
    total_gb = mem.total / (1024**3)
    available_gb = mem.available / (1024**3)
    used_gb = (mem.total - mem.available) / (1024**3)
    percent = mem.percent
    
    check_info(f"RAM: {used_gb:.1f}GB used / {total_gb:.1f}GB total ({percent:.1f}% used)")
    check_info(f"Available: {available_gb:.1f}GB")
    
    if total_gb >= 16:
        check_pass("RAM >= 16GB (recommended)")
    elif total_gb >= 8:
        check_warn("RAM < 16GB (may be slow)")
    else:
        check_fail("RAM < 8GB (insufficient!)")
    
    # GPU
    try:
        import torch
        if torch.cuda.is_available():
            gpu_count = torch.cuda.device_count()
            check_pass(f"CUDA GPU detected: {gpu_count} device(s)")
            
            for i in range(gpu_count):
                props = torch.cuda.get_device_properties(i)
                mem_total = props.total_memory / (1024**2)
                mem_allocated = torch.cuda.memory_allocated(i) / (1024**2)
                mem_free = mem_total - mem_allocated
                percent = (mem_allocated / mem_total) * 100
                
                check_info(f"GPU {i} ({props.name}): {mem_allocated:.0f}MB / {mem_total:.0f}MB ({percent:.1f}%)")
                check_info(f"  Available: {mem_free:.0f}MB")
                
                if mem_total >= 8192:
                    check_pass(f"GPU {i} memory >= 8GB (good)")
                elif mem_total >= 4096:
                    check_warn(f"GPU {i} memory < 8GB (may be limited)")
                else:
                    check_fail(f"GPU {i} memory < 4GB (insufficient!)")
        else:
            check_warn("No CUDA GPU detected (will use CPU - very slow)")
    except:
        check_warn("Could not check GPU status")

def check_files():
    """Check key files"""
    check_section("8. KEY FILES")
    
    key_files = [
        "training/train_sac.py",
        "carla_env/carla_rl_env.py",
        "models/custom_policy.py",
        "utils/sqlite_checkpoint.py",
        "scripts/training/auto_manage.py",
        "web_dashboard/app_fastapi.py",
    ]
    
    for file in key_files:
        path = BASE_DIR / file
        if path.exists():
            check_pass(f"File exists: {file}")
        else:
            check_fail(f"File missing: {file}")

def check_permissions():
    """Check permissions"""
    check_section("9. PERMISSIONS")
    
    dirs_to_check = [
        (BASE_DIR, "Base directory"),
        (BASE_DIR / "checkpoints", "Checkpoints directory"),
        (BASE_DIR / "logs", "Logs directory"),
    ]
    
    for path, name in dirs_to_check:
        if path.exists() and os.access(path, os.W_OK):
            check_pass(f"{name} is writable")
        else:
            check_fail(f"{name} not writable")

def check_processes():
    """Check running processes"""
    check_section("10. RUNNING PROCESSES")
    
    # Check training
    training_procs = [p for p in psutil.process_iter(['pid', 'name', 'cmdline']) 
                     if p.info.get('cmdline') and 'train_sac.py' in ' '.join(p.info['cmdline'])]
    if training_procs:
        check_warn(f"Training process already running (may conflict)")
        for p in training_procs:
            check_info(f"  PID {p.info['pid']}: {' '.join(p.info['cmdline'][:3])}")
    else:
        check_pass("No training process running")
    
    # Check auto_manage
    auto_procs = [p for p in psutil.process_iter(['pid', 'name', 'cmdline'])
                  if p.info.get('cmdline') and 'auto_manage.py' in ' '.join(p.info['cmdline'])]
    if auto_procs:
        check_info("Auto-manager process running")
    else:
        check_info("Auto-manager not running (will start with training)")
    
    # Check dashboard
    dash_procs = [p for p in psutil.process_iter(['pid', 'name', 'cmdline'])
                  if p.info.get('cmdline') and 'app_fastapi.py' in ' '.join(p.info['cmdline'])]
    if dash_procs:
        check_info("Dashboard process running")
    else:
        check_info("Dashboard not running (will start with auto-manager)")

def main():
    print("╔═══════════════════════════════════════════════════════════════╗")
    print("║  SYSTEM PRE-FLIGHT CHECK - CARLA SAC Training                 ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    
    check_directories()
    check_config()
    check_python()
    check_modules()
    check_carla()
    check_disk()
    check_resources()
    check_files()
    check_permissions()
    check_processes()
    
    # Summary
    print(f"\n{'━' * 63}")
    print("SUMMARY")
    print(f"{'━' * 63}\n")
    
    global errors, warnings
    if errors == 0 and warnings == 0:
        print(f"{Colors.GREEN}✅ ALL CHECKS PASSED{Colors.NC}")
        print("System is ready for training!")
        return 0
    elif errors == 0:
        print(f"{Colors.YELLOW}⚠️  {warnings} WARNING(S){Colors.NC}")
        print("System is ready but has some warnings")
        return 0
    else:
        print(f"{Colors.RED}❌ {errors} ERROR(S), {warnings} WARNING(S){Colors.NC}")
        print("Please fix errors before starting training")
        return 1

if __name__ == "__main__":
    sys.exit(main())

