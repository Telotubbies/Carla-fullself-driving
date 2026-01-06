#!/usr/bin/env python3
"""
Comprehensive Pre-Flight Check System
Senior-level system validation before training starts
"""

import os
import sys
import time
import logging
import subprocess
import socket
import importlib.util
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import json

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("PreFlightCheck")

BASE_DIR = Path(__file__).parent.parent  # RL_Agent_SAC directory
CARLA_DIR = BASE_DIR.parent  # CARLA_0.9.16 directory
VENV_PYTHON = BASE_DIR / "venv" / "bin" / "python"
CONFIG_FILE = BASE_DIR / "config" / "sac_config.yaml"
CHECKPOINT_DIR = BASE_DIR / "checkpoints" / "checkpoint"
LOG_DIR = BASE_DIR / "logs"

class PreFlightCheck:
    """Comprehensive pre-flight validation system"""
    
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.passed = []
        self.start_time = time.time()
        
    def check(self, name: str, func, critical: bool = True) -> bool:
        """Run a check and record results"""
        try:
            result = func()
            if result:
                self.passed.append(name)
                logger.info(f"✅ {name}")
                return True
            else:
                if critical:
                    self.errors.append(name)
                    logger.error(f"❌ {name} - CRITICAL")
                else:
                    self.warnings.append(name)
                    logger.warning(f"⚠️  {name} - WARNING")
                return False
        except Exception as e:
            if critical:
                self.errors.append(f"{name}: {str(e)}")
                logger.error(f"❌ {name} - ERROR: {e}")
            else:
                self.warnings.append(f"{name}: {str(e)}")
                logger.warning(f"⚠️  {name} - WARNING: {e}")
            return False
    
    def check_python_environment(self) -> bool:
        """Check Python environment and dependencies"""
        if not VENV_PYTHON.exists():
            logger.error(f"Virtual environment not found: {VENV_PYTHON}")
            return False
        
        # Check critical imports
        try:
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", 
                 "import torch; import carla; import stable_baselines3; import numpy; import cv2; print('OK')"],
                capture_output=True,
                text=True,
                timeout=10
            )
            if result.returncode == 0 and "OK" in result.stdout:
                return True
            else:
                logger.error(f"Import check failed: {result.stderr}")
                return False
        except Exception as e:
            logger.error(f"Python environment check failed: {e}")
            return False
    
    def check_carla_connection(self) -> bool:
        """Check CARLA server connectivity"""
        try:
            import carla
            client = carla.Client('localhost', 2000)
            client.set_timeout(5.0)
            world = client.get_world()
            if world:
                try:
                    _ = world.get_map()
                    return True
                except:
                    return False
            return False
        except Exception as e:
            logger.debug(f"CARLA connection check: {e}")
            return False
    
    def check_carla_process(self) -> bool:
        """Check if CARLA process is running"""
        try:
            import psutil
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.info['cmdline']:
                        cmdline = ' '.join(proc.info['cmdline']).lower()
                        if 'carlaue4' in cmdline or 'carla' in cmdline:
                            return True
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            return False
        except Exception as e:
            logger.debug(f"CARLA process check: {e}")
            return False
    
    def check_gpu_availability(self) -> bool:
        """Check GPU availability and functionality"""
        try:
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", 
                 "import torch; print('CUDA' if torch.cuda.is_available() else 'CPU')"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                if "CUDA" in result.stdout:
                    logger.info("  GPU: CUDA available")
                    return True
                else:
                    logger.warning("  GPU: CUDA not available, will use CPU")
                    return True  # Not critical, can use CPU
            return False
        except Exception as e:
            logger.debug(f"GPU check: {e}")
            return True  # Not critical
    
    def check_config_file(self) -> bool:
        """Check configuration file exists and is valid"""
        if not CONFIG_FILE.exists():
            logger.error(f"Config file not found: {CONFIG_FILE}")
            return False
        
        try:
            import yaml
            with open(CONFIG_FILE, 'r') as f:
                config = yaml.safe_load(f)
            if not config:
                logger.error("Config file is empty")
                return False
            return True
        except Exception as e:
            logger.error(f"Config file validation failed: {e}")
            return False
    
    def check_directories(self) -> bool:
        """Check required directories exist"""
        dirs = [
            BASE_DIR,
            CHECKPOINT_DIR.parent,
            LOG_DIR,
            BASE_DIR / "carla_env",
            BASE_DIR / "models",
            BASE_DIR / "training",
            BASE_DIR / "utils"
        ]
        for dir_path in dirs:
            if not dir_path.exists():
                logger.error(f"Directory not found: {dir_path}")
                return False
        return True
    
    def check_disk_space(self) -> bool:
        """Check available disk space"""
        try:
            import shutil
            total, used, free = shutil.disk_usage(BASE_DIR)
            free_gb = free / (1024**3)
            if free_gb < 10:
                logger.warning(f"Low disk space: {free_gb:.1f} GB free")
                return False
            return True
        except Exception as e:
            logger.debug(f"Disk space check: {e}")
            return True  # Not critical
    
    def check_port_availability(self) -> bool:
        """Check if required ports are available"""
        ports = [2000, 2001, 5001]
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('localhost', port))
            sock.close()
            if result == 0:
                logger.info(f"  Port {port}: In use (OK if service is running)")
            else:
                logger.info(f"  Port {port}: Available")
        return True  # Not critical, just informational
    
    def check_code_syntax(self) -> bool:
        """Check Python code syntax"""
        critical_files = [
            BASE_DIR / "training" / "train_sac.py",
            BASE_DIR / "utils" / "mixed_device_sac.py",
            BASE_DIR / "carla_env" / "carla_rl_env.py",
            BASE_DIR / "web_dashboard" / "app_fastapi.py"
        ]
        
        for file_path in critical_files:
            if not file_path.exists():
                logger.warning(f"File not found: {file_path}")
                continue
            
            try:
                result = subprocess.run(
                    [str(VENV_PYTHON), "-m", "py_compile", str(file_path)],
                    capture_output=True,
                    timeout=5
                )
                if result.returncode != 0:
                    logger.error(f"Syntax error in {file_path.name}: {result.stderr.decode()}")
                    return False
            except Exception as e:
                logger.error(f"Syntax check failed for {file_path.name}: {e}")
                return False
        
        return True
    
    def check_checkpoint_system(self) -> bool:
        """Check checkpoint system functionality"""
        try:
            # Check SQLite database
            db_path = BASE_DIR / "checkpoints" / "training_checkpoints.db"
            if db_path.exists():
                import sqlite3
                conn = sqlite3.connect(str(db_path))
                cursor = conn.cursor()
                cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
                tables = cursor.fetchall()
                conn.close()
                if not tables:
                    logger.warning("Checkpoint database exists but has no tables")
                    return True  # Not critical
            return True
        except Exception as e:
            logger.debug(f"Checkpoint system check: {e}")
            return True  # Not critical
    
    def check_training_script(self) -> bool:
        """Check training script is executable"""
        train_script = BASE_DIR / "training" / "train_sac.py"
        if not train_script.exists():
            logger.error(f"Training script not found: {train_script}")
            return False
        
        # Check if script can be imported
        try:
            spec = importlib.util.spec_from_file_location("train_sac", train_script)
            if spec is None:
                logger.error("Failed to load training script")
                return False
            return True
        except Exception as e:
            logger.error(f"Training script check failed: {e}")
            return False
    
    def run_all_checks(self) -> Dict:
        """Run all pre-flight checks"""
        logger.info("=" * 70)
        logger.info("🚀 PRE-FLIGHT CHECK SYSTEM - COMPREHENSIVE VALIDATION")
        logger.info("=" * 70)
        logger.info("")
        
        # Critical checks
        logger.info("📋 CRITICAL CHECKS:")
        logger.info("-" * 70)
        self.check("Python Environment", self.check_python_environment, critical=True)
        self.check("Code Syntax", self.check_code_syntax, critical=True)
        self.check("Configuration File", self.check_config_file, critical=True)
        self.check("Required Directories", self.check_directories, critical=True)
        self.check("Training Script", self.check_training_script, critical=True)
        
        logger.info("")
        logger.info("📋 SYSTEM CHECKS:")
        logger.info("-" * 70)
        self.check("CARLA Process", self.check_carla_process, critical=False)
        self.check("CARLA Connection", self.check_carla_connection, critical=False)
        self.check("GPU Availability", self.check_gpu_availability, critical=False)
        self.check("Disk Space", self.check_disk_space, critical=False)
        self.check("Port Availability", self.check_port_availability, critical=False)
        self.check("Checkpoint System", self.check_checkpoint_system, critical=False)
        
        # Summary
        elapsed = time.time() - self.start_time
        logger.info("")
        logger.info("=" * 70)
        logger.info("📊 PRE-FLIGHT CHECK SUMMARY")
        logger.info("=" * 70)
        logger.info(f"✅ Passed: {len(self.passed)}")
        logger.info(f"⚠️  Warnings: {len(self.warnings)}")
        logger.info(f"❌ Errors: {len(self.errors)}")
        logger.info(f"⏱️  Time: {elapsed:.2f}s")
        logger.info("")
        
        if self.warnings:
            logger.info("⚠️  WARNINGS:")
            for warning in self.warnings:
                logger.info(f"   - {warning}")
            logger.info("")
        
        if self.errors:
            logger.error("❌ ERRORS (CRITICAL):")
            for error in self.errors:
                logger.error(f"   - {error}")
            logger.error("")
            logger.error("❌ PRE-FLIGHT CHECK FAILED - DO NOT START TRAINING")
            return {
                'status': 'FAILED',
                'passed': self.passed,
                'warnings': self.warnings,
                'errors': self.errors,
                'elapsed': elapsed
            }
        else:
            logger.info("✅ PRE-FLIGHT CHECK PASSED - SYSTEM READY FOR TRAINING")
            return {
                'status': 'PASSED',
                'passed': self.passed,
                'warnings': self.warnings,
                'errors': self.errors,
                'elapsed': elapsed
            }

def main():
    """Main entry point"""
    checker = PreFlightCheck()
    result = checker.run_all_checks()
    
    # Exit with appropriate code
    sys.exit(0 if result['status'] == 'PASSED' else 1)

if __name__ == "__main__":
    main()

