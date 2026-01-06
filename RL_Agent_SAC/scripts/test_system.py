#!/usr/bin/env python3
"""
Comprehensive System Test Suite
Tests all components before training starts
"""

import os
import sys
import time
import logging
import subprocess
import importlib.util
from pathlib import Path
from typing import Dict, List

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("SystemTest")

BASE_DIR = Path(__file__).parent.parent.parent
VENV_PYTHON = BASE_DIR / "venv" / "bin" / "python"

class SystemTester:
    """Comprehensive system testing"""
    
    def __init__(self):
        self.passed = []
        self.failed = []
        self.start_time = time.time()
    
    def test(self, name: str, func) -> bool:
        """Run a test"""
        try:
            logger.info(f"🧪 Testing: {name}...")
            result = func()
            if result:
                self.passed.append(name)
                logger.info(f"✅ {name} - PASSED")
                return True
            else:
                self.failed.append(name)
                logger.error(f"❌ {name} - FAILED")
                return False
        except Exception as e:
            self.failed.append(f"{name}: {str(e)}")
            logger.error(f"❌ {name} - ERROR: {e}")
            return False
    
    def test_imports(self) -> bool:
        """Test all critical imports"""
        try:
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", """
import torch
import carla
import stable_baselines3
import numpy as np
import cv2
import yaml
import sqlite3
print('All imports OK')
                """],
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.returncode == 0 and "OK" in result.stdout
        except Exception as e:
            logger.debug(f"Import test error: {e}")
            return False
    
    def test_carla_connection(self) -> bool:
        """Test CARLA connection"""
        try:
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", """
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
if world:
    _ = world.get_map()
    print('CARLA OK')
else:
    print('CARLA FAILED')
                """],
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.returncode == 0 and "OK" in result.stdout
        except Exception as e:
            logger.debug(f"CARLA test error: {e}")
            return False
    
    def test_gpu_functionality(self) -> bool:
        """Test GPU functionality"""
        try:
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", """
import torch
if torch.cuda.is_available():
    device = torch.device('cuda:0')
    x = torch.randn(10, 10).to(device)
    y = torch.randn(10, 10).to(device)
    z = torch.matmul(x, y)
    print('GPU OK')
else:
    print('GPU not available (CPU mode)')
                """],
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.returncode == 0
        except Exception as e:
            logger.debug(f"GPU test error: {e}")
            return False
    
    def test_model_loading(self) -> bool:
        """Test model loading capability"""
        try:
            # Test if we can import and instantiate model components
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", """
import sys
sys.path.insert(0, '/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC')
from models.vision_encoder import VisionEncoder, ResNetEncoder
from models.custom_policy import CustomFeatureExtractor
print('Model imports OK')
                """],
                capture_output=True,
                text=True,
                timeout=10,
                cwd=str(BASE_DIR)
            )
            return result.returncode == 0
        except Exception as e:
            logger.debug(f"Model test error: {e}")
            return False
    
    def test_config_loading(self) -> bool:
        """Test configuration loading"""
        try:
            config_file = BASE_DIR / "config" / "sac_config.yaml"
            if not config_file.exists():
                return False
            
            result = subprocess.run(
                [str(VENV_PYTHON), "-c", f"""
import yaml
with open('{config_file}', 'r') as f:
    config = yaml.safe_load(f)
assert config is not None
print('Config OK')
                """],
                capture_output=True,
                text=True,
                timeout=5,
                cwd=str(BASE_DIR)
            )
            return result.returncode == 0
        except Exception as e:
            logger.debug(f"Config test error: {e}")
            return False
    
    def test_checkpoint_system(self) -> bool:
        """Test checkpoint system"""
        try:
            db_path = BASE_DIR / "checkpoints" / "training_checkpoints.db"
            if db_path.exists():
                result = subprocess.run(
                    [str(VENV_PYTHON), "-c", f"""
import sqlite3
conn = sqlite3.connect('{db_path}')
cursor = conn.cursor()
cursor.execute('SELECT name FROM sqlite_master WHERE type=\"table\"')
tables = cursor.fetchall()
conn.close()
print('Checkpoint DB OK')
                    """],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    cwd=str(BASE_DIR)
                )
                return result.returncode == 0
            return True  # DB doesn't exist yet, that's OK
        except Exception as e:
            logger.debug(f"Checkpoint test error: {e}")
            return False
    
    def run_all_tests(self) -> Dict:
        """Run all tests"""
        logger.info("=" * 70)
        logger.info("🧪 COMPREHENSIVE SYSTEM TEST SUITE")
        logger.info("=" * 70)
        logger.info("")
        
        self.test("Critical Imports", self.test_imports)
        self.test("CARLA Connection", self.test_carla_connection)
        self.test("GPU Functionality", self.test_gpu_functionality)
        self.test("Model Loading", self.test_model_loading)
        self.test("Config Loading", self.test_config_loading)
        self.test("Checkpoint System", self.test_checkpoint_system)
        
        elapsed = time.time() - self.start_time
        logger.info("")
        logger.info("=" * 70)
        logger.info("📊 TEST SUMMARY")
        logger.info("=" * 70)
        logger.info(f"✅ Passed: {len(self.passed)}")
        logger.info(f"❌ Failed: {len(self.failed)}")
        logger.info(f"⏱️  Time: {elapsed:.2f}s")
        logger.info("")
        
        if self.failed:
            logger.error("❌ FAILED TESTS:")
            for test in self.failed:
                logger.error(f"   - {test}")
            logger.error("")
            return {'status': 'FAILED', 'passed': self.passed, 'failed': self.failed}
        else:
            logger.info("✅ ALL TESTS PASSED")
            return {'status': 'PASSED', 'passed': self.passed, 'failed': self.failed}

def main():
    """Main entry point"""
    tester = SystemTester()
    result = tester.run_all_tests()
    sys.exit(0 if result['status'] == 'PASSED' else 1)

if __name__ == "__main__":
    main()

