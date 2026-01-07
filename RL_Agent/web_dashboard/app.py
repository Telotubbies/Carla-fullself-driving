#!/usr/bin/env python3
"""
Training Dashboard Web Server
แสดงสถานะ training แบบ real-time โดยไม่กระทบ performance
"""

from flask import Flask, render_template, jsonify, request
import sqlite3
import os
import json
import re
from datetime import datetime
from pathlib import Path
import glob
import subprocess
import psutil

app = Flask(__name__)

# Paths
BASE_DIR = Path(__file__).parent.parent
CHECKPOINT_DB = BASE_DIR / "checkpoints" / "training_checkpoints.db"
LOG_DIR = BASE_DIR / "logs"

def get_latest_checkpoint():
    """อ่าน checkpoint ล่าสุดจาก zip files (priority: checkpoints_new) หรือ SQLite"""
    # ✅ Priority 1: Check zip files in checkpoints_new (new training)
    checkpoint_dirs = [
        BASE_DIR / "checkpoints_new" / "checkpoint",  # New training folder
        BASE_DIR / "checkpoints" / "checkpoint"        # Old training folder
    ]
    
    for checkpoint_dir in checkpoint_dirs:
        if checkpoint_dir.exists():
            zip_files = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
            if zip_files:
                # Get latest by modification time
                latest_zip = max(zip_files, key=lambda p: p.stat().st_mtime)
                
                # Extract timestep from filename: rl_model_8000_steps.zip -> 8000
                import re
                match = re.search(r'rl_model_(\d+)_steps\.zip', latest_zip.name)
                if match:
                    timestep = int(match.group(1))
                    mtime = latest_zip.stat().st_mtime
                    created_at = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
                    
                    return {
                        'timestep': timestep,
                        'created_at': created_at,
                        'episode': None,
                        'reward': None,
                        'path': str(latest_zip.relative_to(BASE_DIR))
                    }
    
    # ✅ Fallback: Try SQLite (old format)
    if CHECKPOINT_DB.exists():
        try:
            # ใช้ WAL mode และ timeout เพื่อให้อ่านข้อมูลล่าสุด
            conn = sqlite3.connect(str(CHECKPOINT_DB), timeout=5.0)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            # ตรวจสอบว่ามีข้อมูลหรือไม่
            cursor.execute("SELECT COUNT(*) as count FROM checkpoints")
            count = cursor.fetchone()['count']
            
            if count == 0:
                conn.close()
                return None  # Database ว่าง (ถูก clear แล้ว)
            
            cursor.execute("""
                SELECT timestep, created_at, episode, reward 
                FROM checkpoints 
                ORDER BY timestep DESC 
                LIMIT 1
            """)
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'timestep': row['timestep'],
                    'created_at': row['created_at'],
                    'episode': row['episode'],
                    'reward': row['reward']
                }
        except sqlite3.OperationalError as e:
            # Database อาจจะถูก lock หรือกำลัง clear
            print(f"Database locked or being cleared: {e}")
            return None
        except Exception as e:
            print(f"Error reading checkpoint: {e}")
    
    return None

def get_all_checkpoints():
    """อ่าน checkpoint ทั้งหมด"""
    if not CHECKPOINT_DB.exists():
        return []
    
    try:
        conn = sqlite3.connect(str(CHECKPOINT_DB))
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        
        cursor.execute("""
            SELECT timestep, created_at, episode, reward 
            FROM checkpoints 
            ORDER BY timestep ASC
        """)
        rows = cursor.fetchall()
        conn.close()
        
        return [dict(row) for row in rows]
    except Exception as e:
        print(f"Error reading checkpoints: {e}")
    
    return []

def get_latest_training_log():
    """หาล่าสุด training log file"""
    # ✅ Priority: Always use rl_training_new.log if it exists and is being updated
    primary_log = LOG_DIR / "rl_training_new.log"
    if primary_log.exists():
        # Check if log is being updated (modified in last 5 minutes)
        import time
        mtime = primary_log.stat().st_mtime
        if time.time() - mtime < 300:  # Updated in last 5 minutes
            return primary_log
    
    # Fallback: Try multiple patterns to find the latest training log
    patterns = [
        "rl_training_new.log",  # ✅ New training log (priority)
        "rl_training.log",      # Old training log
        "training_auto_*.log",  # Old pattern
        "training_*.log"        # New pattern with timestamp
    ]
    
    all_log_files = []
    for pattern in patterns:
        log_files = list(LOG_DIR.glob(pattern))
        all_log_files.extend(log_files)
    
    if not all_log_files:
        return None
    
    # Sort by modification time and get the latest
    latest = max(all_log_files, key=lambda p: p.stat().st_mtime)
    return latest

def parse_training_metrics(log_file):
    """Parse metrics จาก log file"""
    if not log_file or not log_file.exists():
        return {}
    
    metrics = {
        'current_step': 0,
        'reward': None,
        'episode_reward': None,
        'episode_length': None,
        'fps': None,
        'iterations': None,
        'batch_size': None,
        'n_steps': None,
        'rollout_count': None,
        'episode_count': None,
        'last_update': None
    }
    
    try:
        # Read last 2000 lines (เพิ่มขึ้นเพื่อให้เจอ metrics table)
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
            last_lines = lines[-2000:] if len(lines) > 2000 else lines
            
            # Parse for metrics table (format จาก stable-baselines3)
            # Look for the table format:
            # | ep_len_mean     | 107       |
            # | ep_rew_mean     | -1.13e+04 |
            # | fps             | 7         |
            # | total_timesteps | 59096     |
            
            # Find the last metrics table
            table_start = -1
            for i in range(len(last_lines) - 1, -1, -1):
                if '| rollout/' in last_lines[i] or '| time/' in last_lines[i]:
                    table_start = i
                    break
            
            if table_start >= 0:
                # Parse table lines
                for i in range(table_start, min(table_start + 20, len(last_lines))):
                    line = last_lines[i].strip()
                    
                    # Episode length
                    if 'ep_len_mean' in line:
                        match = re.search(r'ep_len_mean\s*\|\s*([\d.]+)', line)
                        if match:
                            metrics['episode_length'] = int(float(match.group(1)))
                    
                    # Episode reward
                    if 'ep_rew_mean' in line:
                        match = re.search(r'ep_rew_mean\s*\|\s*([-\d.e+]+)', line)
                        if match:
                            metrics['episode_reward'] = float(match.group(1))
                    
                    # FPS
                    if 'fps' in line.lower() and '|' in line and 'time/' in last_lines[max(0, i-5):i+1]:
                        match = re.search(r'fps\s*\|\s*([\d.]+)', line)
                        if match:
                            metrics['fps'] = int(float(match.group(1)))
                    
                    # Iterations
                    if 'iterations' in line and '|' in line:
                        match = re.search(r'iterations\s*\|\s*(\d+)', line)
                        if match:
                            metrics['iterations'] = int(match.group(1))
                    
                    # Batch size (from config or logs)
                    if 'batch_size' in line.lower() and '|' in line:
                        match = re.search(r'batch_size\s*\|\s*(\d+)', line)
                        if match:
                            metrics['batch_size'] = int(match.group(1))
                    
                    # N steps (rollout size)
                    if 'n_steps' in line.lower() and '|' in line:
                        match = re.search(r'n_steps\s*\|\s*(\d+)', line)
                        if match:
                            metrics['n_steps'] = int(match.group(1))
                    
                    # Total timesteps
                    if 'total_timesteps' in line:
                        match = re.search(r'total_timesteps\s*\|\s*(\d+)', line)
                        if match:
                            metrics['current_step'] = int(match.group(1))
            
            # Fallback: Parse individual lines if table not found
            if metrics['current_step'] == 0:
                # Try to find total_timesteps first
                for line in reversed(last_lines):
                    if 'total_timesteps' in line:
                        match = re.search(r'total_timesteps\s*\|\s*(\d+)', line)
                        if match:
                            metrics['current_step'] = int(match.group(1))
                            break
                
                # ✅ Fallback 2: Parse from "Callback: Step X" (new training log format)
                if metrics['current_step'] == 0:
                    for line in reversed(last_lines):
                        if 'Callback: Step' in line:
                            match = re.search(r'Callback: Step (\d+)', line)
                            if match:
                                metrics['current_step'] = int(match.group(1))
                                break
            
            # Parse episode length from callback logs
            if metrics['episode_length'] is None:
                for line in reversed(last_lines):
                    if 'Episode length:' in line:
                        match = re.search(r'Episode length:\s*(\d+)', line)
                        if match:
                            metrics['episode_length'] = int(match.group(1))
                            break
            
            # Count episodes and rollouts from logs
            episode_count = 0
            rollout_count = 0
            for line in last_lines:
                if 'Episode reward:' in line or 'Episode length:' in line:
                    episode_count += 1
                # Count rollout starts (including current one)
                if '_on_rollout_start' in line:
                    rollout_count += 1
                # Also count completed rollouts
                elif '_on_rollout_end' in line:
                    rollout_count += 1
            
            # If we see rollout_start but no end yet, count it as 1 (current rollout)
            if rollout_count == 0:
                # Check if there's a rollout in progress
                for line in reversed(last_lines):
                    if '_on_rollout_start' in line:
                        rollout_count = 1
                        break
            
            metrics['episode_count'] = episode_count
            metrics['rollout_count'] = rollout_count
            
            # Try to get batch_size and n_steps from config or training logs
            if metrics['batch_size'] is None:
                # Try from log messages
                for line in reversed(last_lines):
                    if 'batch_size' in line.lower() and ('=' in line or ':' in line):
                        match = re.search(r'batch_size[=:]\s*(\d+)', line, re.IGNORECASE)
                        if match:
                            metrics['batch_size'] = int(match.group(1))
                            break
                
                # Fallback: Try to load from checkpoint config
                if metrics['batch_size'] is None:
                    try:
                        checkpoint_dir = BASE_DIR / "checkpoints_new" / "checkpoint"
                        config_files = list(checkpoint_dir.glob("*_config.yaml"))
                        if config_files:
                            # Get latest config
                            latest_config = max(config_files, key=lambda p: p.stat().st_mtime)
                            import yaml
                            with open(latest_config, 'r') as f:
                                config = yaml.safe_load(f)
                                batch_size = config.get('training', {}).get('ppo', {}).get('batch_size')
                                if batch_size:
                                    metrics['batch_size'] = batch_size
                    except:
                        pass
            
            if metrics['n_steps'] is None:
                # Try from log messages
                for line in reversed(last_lines):
                    if 'PPO will collect' in line:
                        match = re.search(r'collect\s+(\d+)\s+steps', line, re.IGNORECASE)
                        if match:
                            metrics['n_steps'] = int(match.group(1))
                            break
                    elif 'n_steps' in line.lower() and ('=' in line or ':' in line):
                        match = re.search(r'n_steps[=:]\s*(\d+)', line, re.IGNORECASE)
                        if match:
                            metrics['n_steps'] = int(match.group(1))
                            break
                
                # Fallback: Try to load from checkpoint config
                if metrics['n_steps'] is None:
                    try:
                        checkpoint_dir = BASE_DIR / "checkpoints_new" / "checkpoint"
                        config_files = list(checkpoint_dir.glob("*_config.yaml"))
                        if config_files:
                            # Get latest config
                            latest_config = max(config_files, key=lambda p: p.stat().st_mtime)
                            import yaml
                            with open(latest_config, 'r') as f:
                                config = yaml.safe_load(f)
                                n_steps = config.get('training', {}).get('ppo', {}).get('n_steps')
                                if n_steps:
                                    metrics['n_steps'] = n_steps
                    except:
                        pass
            
            # Fallback: Get latest reward from step logs
            if metrics['reward'] is None:
                for line in reversed(last_lines):
                    if 'reward=' in line and 'Completed' in line:
                        match = re.search(r'reward=([-\d.]+)', line)
                        if match:
                            metrics['reward'] = float(match.group(1))
                            break
                    # New format: Callback: Step X - Episode reward: Y
                    elif 'Episode reward:' in line:
                        match = re.search(r'Episode reward:\s*([-\d.]+)', line)
                        if match:
                            metrics['reward'] = float(match.group(1))
                            if metrics['episode_reward'] is None:
                                metrics['episode_reward'] = float(match.group(1))
                            break
        
        # Get last update time
        if log_file.exists():
            mtime = log_file.stat().st_mtime
            metrics['last_update'] = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')

        # Final Fallback: If reward is still None but we have episode_reward, use that
        if metrics['reward'] is None and metrics['episode_reward'] is not None:
            metrics['reward'] = metrics['episode_reward']
    
    except Exception as e:
        print(f"Error parsing log: {e}")
    
    return metrics

def get_system_metrics():
    """อ่าน CPU และ GPU metrics"""
    metrics = {
        'cpu': {
            'usage': 0,
            'temp': None,
            'cores': psutil.cpu_count(),
            'freq': None
        },
        'gpu': {
            'usage': None,
            'temp': None,
            'temp_junction': None,
            'temp_edge': None,
            'memory_used': None,
            'memory_total': None,
            'memory_used_mb': None,
            'memory_total_mb': None,
            'memory_percent': None,
            'name': None
        }
    }
    
    # CPU Usage
    try:
        metrics['cpu']['usage'] = psutil.cpu_percent(interval=0.1)
        metrics['cpu']['freq'] = psutil.cpu_freq().current if psutil.cpu_freq() else None
    except:
        pass
    
    # CPU Temperature (ใช้ sensors)
    try:
        result = subprocess.run(
            ['sensors'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            # หา CPU temp (มักจะเป็น Package id 0 หรือ Tdie)
            for line in result.stdout.split('\n'):
                if 'Package id 0' in line or 'Tdie' in line or 'CPU Temperature' in line:
                    match = re.search(r'\+?([\d.]+)°C', line)
                    if match:
                        metrics['cpu']['temp'] = float(match.group(1))
                        break
    except:
        pass
    
    # GPU (AMD - ใช้ rocm-smi)
    try:
        result = subprocess.run(
            ['rocm-smi', '--showtemp', '--showuse', '--showmemuse', '--showproductname', '--json'],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0:
            try:
                gpu_data = json.loads(result.stdout)
                # rocm-smi returns data in format: {"card0": {...}}
                for card_key, card_data in gpu_data.items():
                    if 'card' in card_key.lower() or isinstance(card_data, dict):
                        # Temperature - get both junction and edge
                        temp_junction = card_data.get('Temperature (Sensor junction) (C)', None)
                        temp_edge = card_data.get('Temperature (Sensor edge) (C)', None)
                        
                        # Store both temperatures
                        if temp_junction:
                            try:
                                temp_j = float(temp_junction) if isinstance(temp_junction, str) else temp_junction
                                metrics['gpu']['temp_junction'] = temp_j
                                metrics['gpu']['temp'] = temp_j  # Use junction as main temp (hottest)
                            except (ValueError, TypeError):
                                pass
                        
                        if temp_edge:
                            try:
                                temp_e = float(temp_edge) if isinstance(temp_edge, str) else temp_edge
                                metrics['gpu']['temp_edge'] = temp_e
                                # Use edge as main temp if junction not available
                                if metrics['gpu']['temp'] is None:
                                    metrics['gpu']['temp'] = temp_e
                            except (ValueError, TypeError):
                                pass
                        
                        # GPU Usage
                        gpu_use = card_data.get('GPU use (%)', None)
                        if gpu_use is not None:
                            try:
                                if isinstance(gpu_use, str):
                                    # Remove % if present
                                    gpu_use = re.sub(r'%', '', gpu_use).strip()
                                metrics['gpu']['usage'] = float(gpu_use)
                            except (ValueError, TypeError):
                                pass
                        
                        # Memory - try different formats
                        mem_use = card_data.get('Memory use (%)', None)
                        if mem_use is not None:
                            try:
                                if isinstance(mem_use, str):
                                    mem_use = re.sub(r'%', '', mem_use).strip()
                                metrics['gpu']['memory_percent'] = float(mem_use)
                            except (ValueError, TypeError):
                                pass
                        
                        # Memory total and used from Memory dict
                        mem_info = card_data.get('Memory', {})
                        if isinstance(mem_info, dict):
                            mem_total = mem_info.get('Total Memory (MB)', None)
                            mem_used = mem_info.get('Used Memory (MB)', None)
                            
                            if mem_total is not None:
                                try:
                                    if isinstance(mem_total, str):
                                        mem_total = re.sub(r'[^\d.]', '', mem_total)
                                    mem_total_val = float(mem_total)
                                    metrics['gpu']['memory_total_mb'] = mem_total_val
                                    metrics['gpu']['memory_total'] = mem_total_val / 1024.0  # Convert to GB
                                except (ValueError, TypeError):
                                    pass
                            
                            if mem_used is not None:
                                try:
                                    if isinstance(mem_used, str):
                                        mem_used = re.sub(r'[^\d.]', '', mem_used)
                                    mem_used_val = float(mem_used)
                                    metrics['gpu']['memory_used_mb'] = mem_used_val
                                    metrics['gpu']['memory_used'] = mem_used_val / 1024.0  # Convert to GB
                                except (ValueError, TypeError):
                                    pass
                            
                            # Calculate percentage if we have both
                            if metrics['gpu']['memory_used_mb'] is not None and metrics['gpu']['memory_total_mb'] is not None:
                                try:
                                    metrics['gpu']['memory_percent'] = (metrics['gpu']['memory_used_mb'] / metrics['gpu']['memory_total_mb']) * 100.0
                                except (ZeroDivisionError, TypeError):
                                    pass
                        
                        # GPU Name - try multiple fields
                        name = (card_data.get('Card Series') or 
                               card_data.get('Card series') or 
                               card_data.get('Card Model') or
                               card_data.get('Card model') or
                               card_data.get('Card SKU') or
                               card_data.get('Card SKU') or
                               card_data.get('Card vendor') or
                               card_data.get('Card Vendor'))
                        if name and name != 'Unknown' and str(name).strip():
                            metrics['gpu']['name'] = str(name).strip()
                        
                        break
            except (json.JSONDecodeError, KeyError, ValueError) as e:
                # Fallback: parse text output
                lines = result.stdout.split('\n')
                for line in lines:
                    # Temperature (Sensor junction) - hottest
                    if 'Temperature (Sensor junction)' in line:
                        match = re.search(r'([\d.]+)\s*C', line)
                        if match:
                            try:
                                metrics['gpu']['temp'] = float(match.group(1))
                            except (ValueError, TypeError):
                                pass
                    # GPU use
                    elif 'GPU use' in line or 'GPU Use' in line:
                        match = re.search(r'([\d.]+)\s*%', line)
                        if match:
                            try:
                                metrics['gpu']['usage'] = float(match.group(1))
                            except (ValueError, TypeError):
                                pass
                    # Memory use
                    elif 'Memory use' in line or 'Memory Use' in line:
                        match = re.search(r'([\d.]+)\s*%', line)
                        if match:
                            try:
                                metrics['gpu']['memory_used'] = float(match.group(1))
                            except (ValueError, TypeError):
                                pass
                    # GPU Name
                    elif 'Card' in line and ('Series' in line or 'Model' in line or 'SKU' in line):
                        match = re.search(r'Card\s+(?:Series|Model|SKU|vendor):\s*(.+)', line, re.IGNORECASE)
                        if match:
                            name = match.group(1).strip()
                            if name and name != 'Unknown':
                                metrics['gpu']['name'] = name
    except Exception as e:
        # Log error for debugging but don't fail
        print(f"GPU metrics error (rocm-smi): {e}")
        pass
    
    # GPU (NVIDIA - fallback)
    if metrics['gpu']['temp'] is None:
        try:
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=temperature.gpu,utilization.gpu,memory.used,memory.total,name', 
                 '--format=csv,noheader,nounits'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                parts = result.stdout.strip().split(', ')
                if len(parts) >= 5:
                    metrics['gpu']['temp'] = float(parts[0])
                    metrics['gpu']['usage'] = float(parts[1])
                    # NVIDIA memory is in MB
                    metrics['gpu']['memory_used_mb'] = float(parts[2])
                    metrics['gpu']['memory_total_mb'] = float(parts[3])
                    metrics['gpu']['memory_used'] = float(parts[2]) / 1024.0  # Convert to GB
                    metrics['gpu']['memory_total'] = float(parts[3]) / 1024.0  # Convert to GB
                    # Calculate percentage
                    if metrics['gpu']['memory_total_mb'] > 0:
                        metrics['gpu']['memory_percent'] = (metrics['gpu']['memory_used_mb'] / metrics['gpu']['memory_total_mb']) * 100.0
                    metrics['gpu']['name'] = parts[4]
        except:
            pass
    
    return metrics

def get_power_consumption():
    """คำนวณการใช้ไฟและค่าไฟทั้งระบบ (real-time + สะสม)"""
    power_info = {
        'cpu_power': 0,
        'gpu_power': 0,
        'other_power': 50,  # Base power for other components
        'total_power': 0,
        'current_power_watt': 0,  # Power ณ ปัจจุบัน
        'cumulative_energy_kwh': 0,  # หน่วยสะสม
        'cumulative_cost_baht': 0,  # ค่าไฟสะสม
        'rate_per_kwh': 7,
        'last_update': None
    }
    
    # File to store cumulative data
    power_log_file = BASE_DIR / "logs" / "power_consumption.json"
    
    # Load cumulative data
    cumulative_data = {
        'total_energy_kwh': 0,
        'last_power_watt': 0,
        'last_update_time': None,
        'session_start_time': None
    }
    
    # Backup old data before loading
    old_total_energy = 0
    old_session_start = None
    
    try:
        if power_log_file.exists():
            with open(power_log_file, 'r') as f:
                file_data = json.load(f)
                # Backup old values before overwriting
                old_total_energy = file_data.get('total_energy_kwh', 0)
                old_session_start = file_data.get('session_start_time', None)
                cumulative_data = file_data
    except Exception as e:
        print(f"Error loading power data: {e}")
        pass
    
    try:
        # Get CPU usage and estimate power (ทั้งระบบ)
        cpu_percent = psutil.cpu_percent(interval=0.1)
        cpu_base_power = 80  # Typical desktop CPU TDP
        power_info['cpu_power'] = cpu_base_power * (cpu_percent / 100)
        
        # Get GPU power from rocm-smi (real power)
        try:
            result = subprocess.run(
                ['rocm-smi', '--showpower', '--json'],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                gpu_data = json.loads(result.stdout)
                if 'card0' in gpu_data:
                    card_data = gpu_data['card0']
                    # Look for actual power consumption
                    for key, value in card_data.items():
                        if 'power' in key.lower() or 'watt' in key.lower():
                            val_str = str(value)
                            match = re.search(r'(\d+\.?\d*)', val_str)
                            if match:
                                potential_watt = float(match.group(1))
                                if 10 <= potential_watt <= 500:  # Reasonable range
                                    power_info['gpu_power'] = potential_watt
                                    break
        except:
            # Fallback: estimate GPU power based on usage
            try:
                result = subprocess.run(
                    ['rocm-smi', '--showuse', '--json'],
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                if result.returncode == 0:
                    gpu_data = json.loads(result.stdout)
                    if 'card0' in gpu_data:
                        gpu_use = gpu_data['card0'].get('GPU use (%)', 0)
                        if isinstance(gpu_use, str):
                            gpu_use = float(re.search(r'(\d+\.?\d*)', gpu_use).group(1))
                        # Estimate: 200W base, scales with usage
                        power_info['gpu_power'] = 200 * (gpu_use / 100) if gpu_use else 200
            except:
                power_info['gpu_power'] = 200  # Conservative estimate
        
        # Calculate total power (ทั้งเครื่อง)
        power_info['total_power'] = power_info['cpu_power'] + power_info['gpu_power'] + power_info['other_power']
        power_info['current_power_watt'] = power_info['total_power']
        
        # Calculate cumulative energy (สะสม)
        current_time = psutil.time.time()
        
        # Preserve old total_energy_kwh if it exists - NEVER LOSE THIS DATA
        preserved_energy = cumulative_data.get('total_energy_kwh', 0)
        if preserved_energy == 0 and old_total_energy > 0:
            # Restore from backup if current value is 0 but backup has data
            preserved_energy = old_total_energy
            cumulative_data['total_energy_kwh'] = old_total_energy
            print(f"Restored total_energy_kwh from backup: {old_total_energy} kWh")
        
        # Always use the maximum of preserved and old energy (safety check)
        preserved_energy = max(preserved_energy, old_total_energy)
        if preserved_energy > cumulative_data.get('total_energy_kwh', 0):
            cumulative_data['total_energy_kwh'] = preserved_energy
        
        # Initialize session start time if not exists (but preserve old data)
        if cumulative_data['session_start_time'] is None:
            if old_session_start is not None:
                # Restore old session start time if available
                cumulative_data['session_start_time'] = old_session_start
                print(f"Restored session_start_time from backup: {old_session_start}")
            else:
                # Only set new session start if no old data exists AND no energy recorded
                if preserved_energy == 0:
                    cumulative_data['session_start_time'] = current_time
                else:
                    # We have energy but no session start - estimate from last_update_time
                    if cumulative_data.get('last_update_time'):
                        # Estimate: session started 1 hour before last update (conservative)
                        cumulative_data['session_start_time'] = cumulative_data['last_update_time'] - 3600
                    else:
                        # No data at all - start fresh
                        cumulative_data['session_start_time'] = current_time
        
        # Calculate energy since last update
        # IMPORTANT: Always preserve existing total_energy_kwh, never reset it
        if cumulative_data['last_update_time'] is not None:
            # Normal update: add increment to existing total
            time_diff_hours = (current_time - cumulative_data['last_update_time']) / 3600
            if time_diff_hours > 0:
                # Use average of last power and current power
                avg_power = (cumulative_data['last_power_watt'] + power_info['total_power']) / 2
                energy_increment = (avg_power / 1000) * time_diff_hours
                # Always add to preserved energy, never replace
                cumulative_data['total_energy_kwh'] = preserved_energy + energy_increment
            else:
                # Time didn't advance (shouldn't happen, but be safe)
                cumulative_data['total_energy_kwh'] = preserved_energy
        else:
            # First run or restart: check if we have existing energy to preserve
            if preserved_energy > 0:
                # We have existing energy - just keep it (don't recalculate)
                # This happens on restart when last_update_time is None but we have old data
                cumulative_data['total_energy_kwh'] = preserved_energy
                # Set last_update_time to current time to resume tracking
                cumulative_data['last_update_time'] = current_time
            elif cumulative_data['session_start_time']:
                # Truly first run: calculate from session start
                session_hours = (current_time - cumulative_data['session_start_time']) / 3600
                if session_hours > 0:
                    session_energy = (power_info['total_power'] / 1000) * session_hours
                    cumulative_data['total_energy_kwh'] = session_energy
                else:
                    cumulative_data['total_energy_kwh'] = 0
            else:
                # No data at all: start fresh
                cumulative_data['total_energy_kwh'] = 0
        
        # Update cumulative data
        cumulative_data['last_power_watt'] = power_info['total_power']
        cumulative_data['last_update_time'] = current_time
        
        # Save cumulative data (with backup and validation)
        try:
            # Create backup before writing
            backup_file = power_log_file.with_suffix('.json.backup')
            if power_log_file.exists():
                import shutil
                shutil.copy2(power_log_file, backup_file)
            
            # Validate data before saving (ensure total_energy_kwh is never lost)
            if cumulative_data.get('total_energy_kwh', 0) < preserved_energy:
                # This should never happen, but if it does, restore from preserved
                print(f"WARNING: Energy would decrease from {preserved_energy} to {cumulative_data.get('total_energy_kwh', 0)}. Restoring preserved value.")
                cumulative_data['total_energy_kwh'] = preserved_energy
            
            # Ensure we have valid session_start_time if we have energy
            if cumulative_data.get('total_energy_kwh', 0) > 0 and cumulative_data.get('session_start_time') is None:
                # Restore from backup if available
                if old_session_start is not None:
                    cumulative_data['session_start_time'] = old_session_start
                else:
                    # Estimate session start from last_update_time if available
                    if cumulative_data.get('last_update_time'):
                        # Assume session started 1 hour before last update (conservative)
                        cumulative_data['session_start_time'] = cumulative_data['last_update_time'] - 3600
            
            # Write new data
            with open(power_log_file, 'w') as f:
                json.dump(cumulative_data, f, indent=2)
            
            # Verify write was successful
            try:
                with open(power_log_file, 'r') as f:
                    verify_data = json.load(f)
                    if verify_data.get('total_energy_kwh', 0) < preserved_energy:
                        # Write failed or data corrupted, restore from backup
                        raise ValueError("Data verification failed")
            except:
                # Restore from backup
                if backup_file.exists():
                    import shutil
                    shutil.copy2(backup_file, power_log_file)
                    print(f"Restored power data from backup due to verification failure")
                    
        except Exception as e:
            print(f"Error saving power data: {e}")
            # Try to restore from backup if write failed
            try:
                backup_file = power_log_file.with_suffix('.json.backup')
                if backup_file.exists():
                    import shutil
                    shutil.copy2(backup_file, power_log_file)
                    print(f"Restored power data from backup")
            except Exception as restore_error:
                print(f"Failed to restore from backup: {restore_error}")
        
        # Set cumulative values
        power_info['cumulative_energy_kwh'] = cumulative_data['total_energy_kwh']
        power_info['cumulative_cost_baht'] = power_info['cumulative_energy_kwh'] * power_info['rate_per_kwh']
        power_info['last_update'] = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S')
        
        # Session runtime
        if cumulative_data['session_start_time']:
            session_seconds = current_time - cumulative_data['session_start_time']
            power_info['session_runtime_hours'] = session_seconds / 3600
        
    except Exception as e:
        print(f"Error calculating power: {e}")
    
    return power_info

def get_training_status():
    """ตรวจสอบว่า training กำลังทำงานหรือไม่"""
    status = {'running': False}
    
    try:
        # Get system memory info first
        system_mem = psutil.virtual_memory()
        system_mem_percent = system_mem.percent
        system_mem_total_gb = system_mem.total / 1024 / 1024 / 1024
        system_mem_used_gb = system_mem.used / 1024 / 1024 / 1024
        
        # Find training process using psutil
        # Look for the actual Python process (not bash wrapper)
        for proc in psutil.process_iter(['pid', 'name', 'memory_info', 'cmdline', 'cpu_percent']):
            try:
                cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                proc_name = proc.name().lower()
                
                # Check if it's training process and actual Python (not bash)
                if 'train.py' in cmdline and 'grep' not in cmdline:
                    # Prefer actual Python process over bash wrapper
                    if 'python' in proc_name or 'python3' in proc_name:
                        mem_info = proc.memory_info()
                        mem_gb = mem_info.rss / 1024 / 1024 / 1024
                        mem_mb = mem_info.rss / 1024 / 1024
                        
                        status = {
                            'running': True,
                            'pid': str(proc.pid),
                            'cpu': f"{proc.cpu_percent(interval=0.1):.1f}",
                            'memory': f"{system_mem_percent:.1f}",  # System memory percent
                            'memory_mb': f"{mem_mb:.0f}",  # Process memory in MB
                            'memory_gb': f"{mem_gb:.2f}",  # Process memory in GB
                            'system_mem_total_gb': f"{system_mem_total_gb:.1f}",
                            'system_mem_used_gb': f"{system_mem_used_gb:.1f}"
                        }
                        break
                    elif not status['running']:
                        # Fallback to bash process if no Python found yet
                        mem_info = proc.memory_info()
                        mem_gb = mem_info.rss / 1024 / 1024 / 1024
                        mem_mb = mem_info.rss / 1024 / 1024
                        
                        status = {
                            'running': True,
                            'pid': str(proc.pid),
                            'cpu': f"{proc.cpu_percent(interval=0.1):.1f}",
                            'memory': f"{system_mem_percent:.1f}",
                            'memory_mb': f"{mem_mb:.0f}",
                            'memory_gb': f"{mem_gb:.2f}",
                            'system_mem_total_gb': f"{system_mem_total_gb:.1f}",
                            'system_mem_used_gb': f"{system_mem_used_gb:.1f}"
                        }
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        # If no training process found, still return system memory info
        if not status['running']:
            status['memory'] = f"{system_mem_percent:.1f}"
            status['system_mem_total_gb'] = f"{system_mem_total_gb:.1f}"
            status['system_mem_used_gb'] = f"{system_mem_used_gb:.1f}"
    except Exception as e:
        print(f"Error getting training status: {e}")
    
    return status

@app.route('/')
def index():
    """หน้า dashboard หลัก - React version"""
    # Check if React build exists, otherwise fallback to old dashboard
    react_index = Path(__file__).parent / 'static' / 'react' / 'index.html'
    if react_index.exists():
        return open(react_index, 'r', encoding='utf-8').read()
    else:
        # Fallback to old dashboard
        return render_template('dashboard.html')

@app.route('/assets/<path:filename>')
def react_assets(filename):
    """Serve React build assets"""
    return send_from_directory(
        Path(__file__).parent / 'static' / 'react' / 'assets',
        filename
    )

@app.route('/api/status')
def api_status():
    """API: สถานะ training"""
    checkpoint = get_latest_checkpoint()
    log_file = get_latest_training_log()
    metrics = parse_training_metrics(log_file)
    status = get_training_status()
    system_metrics = get_system_metrics()
    power_info = get_power_consumption()
    
    # Calculate progress
    # ใช้ค่าเริ่มต้นจาก config หรือ 0 ถ้าไม่มี checkpoint
    started = 0  # เริ่มจาก 0 เมื่อ clear DB แล้ว
    target = 500000
    current = checkpoint['timestep'] if checkpoint else (metrics.get('current_step', 0) or 0)
    progress = ((current - started) / (target - started)) * 100 if current > started and target > started else 0
    
    return jsonify({
        'checkpoint': checkpoint,
        'metrics': metrics,
        'status': status,
        'system': system_metrics,
        'power': power_info,
        'progress': {
            'current': current,
            'started': started,
            'target': target,
            'remaining': target - current,
            'percentage': round(progress, 2)
        },
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    })

@app.route('/api/checkpoints')
def api_checkpoints():
    """API: checkpoint history"""
    checkpoints = get_all_checkpoints()
    return jsonify({
        'checkpoints': checkpoints,
        'count': len(checkpoints)
    })

@app.route('/api/metrics/history')
def api_metrics_history():
    """API: metrics history (parse จาก training logs)"""
    log_file = get_latest_training_log()
    
    if not log_file or not log_file.exists():
        # Fallback to checkpoints if no log file
        checkpoints = get_all_checkpoints()
        history = {
            'timesteps': [c['timestep'] for c in checkpoints],
            'rewards': [c['reward'] if c['reward'] else 0 for c in checkpoints],
            'episodes': [c['episode'] if c['episode'] else 0 for c in checkpoints],
            'dates': [c['created_at'] for c in checkpoints]
        }
        return jsonify(history)
    
    # Parse episode data from logs
    timesteps = []
    rewards = []
    episodes = []
    dates = []
    
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
            
            # Parse episode rewards and lengths from callback logs
            for line in lines:
                # Format: "Callback: Step X - Episode reward: Y, Episode length: Z"
                if 'Callback: Step' in line and 'Episode reward:' in line:
                    step_match = re.search(r'Callback: Step (\d+)', line)
                    reward_match = re.search(r'Episode reward:\s*([-\d.]+)', line)
                    length_match = re.search(r'Episode length:\s*(\d+)', line)
                    
                    if step_match and reward_match:
                        timestep = int(step_match.group(1))
                        reward = float(reward_match.group(1))
                        episode_length = int(length_match.group(1)) if length_match else 0
                        
                        timesteps.append(timestep)
                        rewards.append(reward)
                        episodes.append(episode_length)
                        
                        # Try to get date from log line
                        date_match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', line)
                        if date_match:
                            dates.append(date_match.group(1))
                        else:
                            dates.append(None)
            
            # Also parse from metrics table (ep_rew_mean, ep_len_mean, total_timesteps)
            # This gives us aggregated metrics per iteration
            # Improved: Find metrics tables by looking for "rollout/" marker
            i = 0
            while i < len(lines):
                if '| rollout/' in lines[i]:
                    # Found start of metrics table, collect all three metrics
                    table_metrics = {}
                    
                    # Search next 30 lines for the three metrics
                    for j in range(i, min(i+30, len(lines))):
                        line = lines[j]
                        
                        # Extract total_timesteps
                        if 'total_timesteps' in line and '|' in line:
                            match = re.search(r'total_timesteps\s*\|\s*(\d+)', line)
                            if match:
                                table_metrics['timestep'] = int(match.group(1))
                        
                        # Extract ep_rew_mean
                        if 'ep_rew_mean' in line and '|' in line:
                            match = re.search(r'ep_rew_mean\s*\|\s*([-\d.e+]+)', line)
                            if match:
                                table_metrics['reward'] = float(match.group(1))
                        
                        # Extract ep_len_mean
                        if 'ep_len_mean' in line and '|' in line:
                            match = re.search(r'ep_len_mean\s*\|\s*([\d.]+)', line)
                            if match:
                                table_metrics['episode'] = int(float(match.group(1)))
                        
                        # Stop if we found all three
                        if len(table_metrics) == 3:
                            break
                    
                    # If we have all three metrics, add to history
                    if len(table_metrics) == 3:
                        ts = table_metrics['timestep']
                        # Check if this timestep already exists (from callback logs)
                        if ts not in timesteps:
                            timesteps.append(ts)
                            rewards.append(table_metrics['reward'])
                            episodes.append(table_metrics['episode'])
                            dates.append(None)
                
                i += 1
        
        # Sort by timestep
        if timesteps:
            sorted_data = sorted(zip(timesteps, rewards, episodes, dates))
            timesteps, rewards, episodes, dates = zip(*sorted_data)
            timesteps = list(timesteps)
            rewards = list(rewards)
            episodes = list(episodes)
            dates = list(dates)
    
    except Exception as e:
        print(f"Error parsing training history: {e}")
        # Fallback to checkpoints
        checkpoints = get_all_checkpoints()
        timesteps = [c['timestep'] for c in checkpoints]
        rewards = [c['reward'] if c['reward'] else 0 for c in checkpoints]
        episodes = [c['episode'] if c['episode'] else 0 for c in checkpoints]
        dates = [c['created_at'] for c in checkpoints]
    
    history = {
        'timesteps': timesteps,
        'rewards': rewards,
        'episodes': episodes,
        'dates': dates
    }
    
    return jsonify(history)

# Demo endpoints (lazy import with correct path)
@app.route('/api/demo/reset', methods=['POST'])
def api_demo_reset():
    """Reset demo simulation"""
    try:
        import sys
        from pathlib import Path
        demo_path = Path(__file__).parent
        if str(demo_path) not in sys.path:
            sys.path.insert(0, str(demo_path))
        from demo_simulator import reset_demo
        state = reset_demo()
        return jsonify({'success': True, 'state': state})
    except ImportError as e:
        # Demo simulator not available - return graceful error
        return jsonify({
            'success': False, 
            'error': 'Demo simulator not available',
            'message': 'Demo feature is disabled or not properly configured'
        }), 200  # Return 200 instead of 500 for graceful handling
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Demo reset error: {error_msg}")
        # Return 200 with error info instead of 500 for better UX
        return jsonify({
            'success': False, 
            'error': str(e),
            'message': 'Failed to reset demo simulation'
        }), 200

@app.route('/api/demo/step', methods=['POST'])
def api_demo_step():
    """Step demo simulation - Full Self-Driving or Manual Control"""
    try:
        import sys
        from pathlib import Path
        demo_path = Path(__file__).parent
        if str(demo_path) not in sys.path:
            sys.path.insert(0, str(demo_path))
        from demo_simulator import step_demo
        data = request.get_json() or {}
        action = data.get('action')  # None = Full Self-Driving (AI), [steer, throttle, brake] = Manual
        
        # If action is explicitly None (from JSON), convert to Python None
        # If action is not provided or empty list, use None for self-driving
        if action is None or (isinstance(action, list) and len(action) == 0):
            action = None  # Full Self-Driving mode
        
        state = step_demo(action)
        return jsonify({'success': True, 'state': state})
    except ImportError as e:
        # Demo simulator not available - return graceful error
        return jsonify({
            'success': False, 
            'error': 'Demo simulator not available',
            'message': 'Demo feature is disabled or not properly configured'
        }), 200  # Return 200 instead of 500 for graceful handling
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Demo step error: {error_msg}")
        # Return 200 with error info instead of 500 for better UX
        return jsonify({
            'success': False, 
            'error': str(e),
            'message': 'Failed to step demo simulation'
        }), 200

@app.route('/api/demo/state', methods=['GET'])
def api_demo_state():
    """Get current demo state"""
    try:
        import sys
        from pathlib import Path
        demo_path = Path(__file__).parent
        if str(demo_path) not in sys.path:
            sys.path.insert(0, str(demo_path))
        from demo_simulator import get_simulator
        simulator = get_simulator()
        state = simulator.get_state()
        return jsonify({'success': True, 'state': state})
    except ImportError as e:
        # Demo simulator not available - return graceful error
        return jsonify({
            'success': False, 
            'error': 'Demo simulator not available',
            'message': 'Demo feature is disabled or not properly configured'
        }), 200  # Return 200 instead of 500 for graceful handling
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Demo state error: {error_msg}")
        # Return 200 with error info instead of 500 for better UX
        return jsonify({
            'success': False, 
            'error': str(e),
            'message': 'Failed to get demo state'
        }), 200

@app.route('/api/logs/auto_manage')
def api_auto_manage_log():
    """API: อ่าน auto_manage.log แบบ real-time"""
    auto_manage_log = LOG_DIR / "auto_manage.log"
    
    try:
        lines = []
        if auto_manage_log.exists():
            # อ่านบรรทัดสุดท้าย 100 บรรทัด
            with open(auto_manage_log, 'r', encoding='utf-8', errors='ignore') as f:
                all_lines = f.readlines()
                lines = all_lines[-100:] if len(all_lines) > 100 else all_lines
            
            # Get file modification time
            mtime = auto_manage_log.stat().st_mtime
            last_update = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
        else:
            last_update = None
        
        return jsonify({
            'success': True,
            'lines': [line.rstrip('\n') for line in lines],
            'count': len(lines),
            'last_update': last_update,
            'file_exists': auto_manage_log.exists()
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e),
            'lines': [],
            'count': 0
        }), 500

if __name__ == '__main__':
    print("=" * 70)
    print("🚀 Starting Training Dashboard")
    print("=" * 70)
    print(f"📊 Dashboard: http://localhost:5000")
    print(f"📡 API: http://localhost:5000/api/status")
    print(f"🎮 Demo: http://localhost:5000#demo")
    print()
    print("⚠️  Note: Dashboard อ่านข้อมูลเท่านั้น ไม่กระทบ training performance")
    print("=" * 70)
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

