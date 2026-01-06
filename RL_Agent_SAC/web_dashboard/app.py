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
BASE_DIR = Path(__file__).parent.parent
CHECKPOINT_DB = BASE_DIR / "checkpoints" / "training_checkpoints.db"
LOG_DIR = BASE_DIR / "logs"
ALGORITHM = "SAC"
def get_latest_checkpoint():
    
    checkpoint_dirs = [
        BASE_DIR / "checkpoints_new" / "checkpoint",
        BASE_DIR / "checkpoints" / "checkpoint"
    ]
    for checkpoint_dir in checkpoint_dirs:
        if checkpoint_dir.exists():
            zip_files = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
            if zip_files:
                latest_zip = max(zip_files, key=lambda p: p.stat().st_mtime)
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
    if CHECKPOINT_DB.exists():
        try:
            conn = sqlite3.connect(str(CHECKPOINT_DB), timeout=5.0)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) as count FROM checkpoints")
            count = cursor.fetchone()['count']
            if count == 0:
                conn.close()
                return None
            cursor.execute("SELECT * FROM checkpoints ORDER BY timestep DESC LIMIT 1")
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
            print(f"Database locked or being cleared: {e}")
            return None
        except Exception as e:
            print(f"Error reading checkpoint: {e}")
    return None
def get_all_checkpoints():
    
    if not CHECKPOINT_DB.exists():
        return []
    try:
        conn = sqlite3.connect(str(CHECKPOINT_DB))
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM checkpoints ORDER BY timestep DESC")
        rows = cursor.fetchall()
        conn.close()
        return [dict(row) for row in rows]
    except Exception as e:
        print(f"Error reading checkpoints: {e}")
    return []
def get_latest_training_log():
    
    patterns = [
        "sac_training_*.log",
        "rl_training_new.log",
        "rl_training.log",
        "training_*.log"
    ]
    all_log_files = []
    for pattern in patterns:
        log_files = list(LOG_DIR.glob(pattern))
        all_log_files.extend(log_files)
    if not all_log_files:
        return None
    latest = max(all_log_files, key=lambda p: p.stat().st_mtime)
    import time
    mtime = latest.stat().st_mtime
    if time.time() - mtime < 300:
        return latest
    return latest
def parse_training_metrics(log_file):
    
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
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
            last_lines = lines[-2000:] if len(lines) > 2000 else lines
            table_start = -1
            for i in range(len(last_lines) - 1, -1, -1):
                if '| rollout/' in last_lines[i] or '| time/' in last_lines[i]:
                    table_start = i
                    break
            if table_start >= 0:
                for i in range(table_start, min(table_start + 20, len(last_lines))):
                    line = last_lines[i].strip()
                    if 'ep_len_mean' in line:
                        match = re.search(r'ep_len_mean\s*\|\s*([\d.]+)', line)
                        if match:
                            metrics['episode_length'] = int(float(match.group(1)))
                    if 'ep_rew_mean' in line:
                        match = re.search(r'ep_rew_mean\s*\|\s*([-\d.e+]+)', line)
                        if match:
                            metrics['episode_reward'] = float(match.group(1))
                    if 'fps' in line.lower() and '|' in line and 'time/' in last_lines[max(0, i-5):i+1]:
                        match = re.search(r'fps\s*\|\s*([\d.]+)', line)
                        if match:
                            metrics['fps'] = int(float(match.group(1)))
                    if 'iterations' in line and '|' in line:
                        match = re.search(r'iterations\s*\|\s*(\d+)', line)
                        if match:
                            metrics['iterations'] = int(match.group(1))
                    if 'batch_size' in line.lower() and '|' in line:
                        match = re.search(r'batch_size\s*\|\s*(\d+)', line)
                        if match:
                            metrics['batch_size'] = int(match.group(1))
                    if 'n_steps' in line.lower() and '|' in line:
                        match = re.search(r'n_steps\s*\|\s*(\d+)', line)
                        if match:
                            metrics['n_steps'] = int(match.group(1))
                    if 'total_timesteps' in line:
                        match = re.search(r'total_timesteps\s*\|\s*(\d+)', line)
                        if match:
                            metrics['current_step'] = int(match.group(1))
            if metrics['current_step'] == 0:
                for line in reversed(last_lines):
                    if 'total_timesteps' in line:
                        match = re.search(r'total_timesteps\s*\|\s*(\d+)', line)
                        if match:
                            metrics['current_step'] = int(match.group(1))
                            break
                # Don't use Callback: Step as it's local step, not global timestep
                # Only use total_timesteps from rollout table
                if metrics['current_step'] == 0:
                    # Try to find from checkpoint if available
                    try:
                        checkpoint = get_latest_checkpoint()
                        if checkpoint and checkpoint.get('timestep'):
                            metrics['current_step'] = checkpoint['timestep']
                    except:
                        pass
            if metrics['episode_length'] is None:
                for line in reversed(last_lines):
                    if 'Episode length:' in line:
                        match = re.search(r'Episode length:\s*(\d+)', line)
                        if match:
                            metrics['episode_length'] = int(match.group(1))
                            break
            episode_count = 0
            rollout_count = 0
            for line in last_lines:
                if 'Episode reward:' in line or 'Episode length:' in line:
                    episode_count += 1
                if '_on_rollout_start' in line:
                    rollout_count += 1
                elif '_on_rollout_end' in line:
                    rollout_count += 1
            if rollout_count == 0:
                for line in reversed(last_lines):
                    if '_on_rollout_start' in line:
                        rollout_count = 1
                        break
            metrics['episode_count'] = episode_count
            metrics['rollout_count'] = rollout_count
            if metrics['batch_size'] is None:
                for line in reversed(last_lines):
                    if 'batch_size' in line.lower() and ('=' in line or ':' in line):
                        match = re.search(r'batch_size[=:]\s*(\d+)', line, re.IGNORECASE)
                        if match:
                            metrics['batch_size'] = int(match.group(1))
                            break
                if metrics['batch_size'] is None:
                    try:
                        checkpoint_dir = BASE_DIR / "checkpoints_new" / "checkpoint"
                        config_files = list(checkpoint_dir.glob("*_config.yaml"))
                        if config_files:
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
                if metrics['n_steps'] is None:
                    try:
                        checkpoint_dir = BASE_DIR / "checkpoints_new" / "checkpoint"
                        config_files = list(checkpoint_dir.glob("*_config.yaml"))
                        if config_files:
                            latest_config = max(config_files, key=lambda p: p.stat().st_mtime)
                            import yaml
                            with open(latest_config, 'r') as f:
                                config = yaml.safe_load(f)
                                n_steps = config.get('training', {}).get('ppo', {}).get('n_steps')
                                if n_steps:
                                    metrics['n_steps'] = n_steps
                    except:
                        pass
            if metrics['reward'] is None:
                for line in reversed(last_lines):
                    if 'reward=' in line and 'Completed' in line:
                        match = re.search(r'reward=([-\d.]+)', line)
                        if match:
                            metrics['reward'] = float(match.group(1))
                            break
                    elif 'Episode reward:' in line:
                        match = re.search(r'Episode reward:\s*([-\d.]+)', line)
                        if match:
                            metrics['reward'] = float(match.group(1))
                            if metrics['episode_reward'] is None:
                                metrics['episode_reward'] = float(match.group(1))
                            break
        if log_file.exists():
            mtime = log_file.stat().st_mtime
            metrics['last_update'] = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
        if metrics['reward'] is None and metrics['episode_reward'] is not None:
            metrics['reward'] = metrics['episode_reward']
    except Exception as e:
        print(f"Error parsing log: {e}")
    return metrics
def get_system_metrics():
    
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
    try:
        metrics['cpu']['usage'] = psutil.cpu_percent(interval=0.1)
        metrics['cpu']['freq'] = psutil.cpu_freq().current if psutil.cpu_freq() else None
    except:
        pass
    try:
        result = subprocess.run(
            ['sensors'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'Package id 0' in line or 'Tdie' in line or 'CPU Temperature' in line:
                    match = re.search(r'\+?([\d.]+)°C', line)
                    if match:
                        metrics['cpu']['temp'] = float(match.group(1))
                        break
    except:
        pass
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
                for card_key, card_data in gpu_data.items():
                    if 'card' in card_key.lower() or isinstance(card_data, dict):
                        temp_junction = card_data.get('Temperature (Sensor junction) (C)', None)
                        temp_edge = card_data.get('Temperature (Sensor edge) (C)', None)
                        if temp_junction:
                            try:
                                temp_j = float(temp_junction) if isinstance(temp_junction, str) else temp_junction
                                metrics['gpu']['temp_junction'] = temp_j
                                metrics['gpu']['temp'] = temp_j
                            except (ValueError, TypeError):
                                pass
                        if temp_edge:
                            try:
                                temp_e = float(temp_edge) if isinstance(temp_edge, str) else temp_edge
                                metrics['gpu']['temp_edge'] = temp_e
                                if metrics['gpu']['temp'] is None:
                                    metrics['gpu']['temp'] = temp_e
                            except (ValueError, TypeError):
                                pass
                        gpu_use = card_data.get('GPU use (%)', None)
                        if gpu_use is not None:
                            try:
                                if isinstance(gpu_use, str):
                                    gpu_use = re.sub(r'%', '', gpu_use).strip()
                                metrics['gpu']['usage'] = float(gpu_use)
                            except (ValueError, TypeError):
                                pass
                        mem_use = card_data.get('Memory use (%)', None)
                        if mem_use is not None:
                            try:
                                if isinstance(mem_use, str):
                                    mem_use = re.sub(r'%', '', mem_use).strip()
                                metrics['gpu']['memory_percent'] = float(mem_use)
                            except (ValueError, TypeError):
                                pass
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
                                    metrics['gpu']['memory_total'] = mem_total_val / 1024.0
                                except (ValueError, TypeError):
                                    pass
                            if mem_used is not None:
                                try:
                                    if isinstance(mem_used, str):
                                        mem_used = re.sub(r'[^\d.]', '', mem_used)
                                    mem_used_val = float(mem_used)
                                    metrics['gpu']['memory_used_mb'] = mem_used_val
                                    metrics['gpu']['memory_used'] = mem_used_val / 1024.0
                                except (ValueError, TypeError):
                                    pass
                            if metrics['gpu']['memory_used_mb'] is not None and metrics['gpu']['memory_total_mb'] is not None:
                                try:
                                    metrics['gpu']['memory_percent'] = (metrics['gpu']['memory_used_mb'] / metrics['gpu']['memory_total_mb']) * 100.0
                                except (ZeroDivisionError, TypeError):
                                    pass
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
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'Temperature (Sensor junction)' in line:
                        match = re.search(r'([\d.]+)\s*C', line)
                        if match:
                            try:
                                metrics['gpu']['temp'] = float(match.group(1))
                            except (ValueError, TypeError):
                                pass
                    elif 'GPU use' in line or 'GPU Use' in line:
                        match = re.search(r'([\d.]+)\s*%', line)
                        if match:
                            try:
                                metrics['gpu']['usage'] = float(match.group(1))
                            except (ValueError, TypeError):
                                pass
                    elif 'Memory use' in line or 'Memory Use' in line:
                        match = re.search(r'([\d.]+)\s*%', line)
                        if match:
                            try:
                                metrics['gpu']['memory_used'] = float(match.group(1))
                            except (ValueError, TypeError):
                                pass
                    elif 'Card' in line and ('Series' in line or 'Model' in line or 'SKU' in line):
                        match = re.search(r'Card\s+(?:Series|Model|SKU|vendor):\s*(.+)', line, re.IGNORECASE)
                        if match:
                            name = match.group(1).strip()
                            if name and name != 'Unknown':
                                metrics['gpu']['name'] = name
    except Exception as e:
        print(f"GPU metrics error (rocm-smi): {e}")
        pass
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
                    metrics['gpu']['memory_used_mb'] = float(parts[2])
                    metrics['gpu']['memory_total_mb'] = float(parts[3])
                    metrics['gpu']['memory_used'] = float(parts[2]) / 1024.0
                    metrics['gpu']['memory_total'] = float(parts[3]) / 1024.0
                    if metrics['gpu']['memory_total_mb'] > 0:
                        metrics['gpu']['memory_percent'] = (metrics['gpu']['memory_used_mb'] / metrics['gpu']['memory_total_mb']) * 100.0
                    metrics['gpu']['name'] = parts[4]
        except:
            pass
    return metrics
def get_power_consumption():
    
    power_info = {
        'cpu_power': 0,
        'gpu_power': 0,
        'other_power': 50,
        'total_power': 0,
        'current_power_watt': 0,
        'cumulative_energy_kwh': 0,
        'cumulative_cost_baht': 0,
        'rate_per_kwh': 7,
        'last_update': None
    }
    power_log_file = BASE_DIR / "logs" / "power_consumption.json"
    cumulative_data = {
        'total_energy_kwh': 0,
        'last_power_watt': 0,
        'last_update_time': None,
        'session_start_time': None
    }
    old_total_energy = 0
    old_session_start = None
    try:
        if power_log_file.exists():
            with open(power_log_file, 'r') as f:
                file_data = json.load(f)
                old_total_energy = file_data.get('total_energy_kwh', 0)
                old_session_start = file_data.get('session_start_time', None)
                cumulative_data = file_data
    except Exception as e:
        print(f"Error loading power data: {e}")
        pass
    try:
        cpu_percent = psutil.cpu_percent(interval=0.1)
        cpu_base_power = 80
        power_info['cpu_power'] = cpu_base_power * (cpu_percent / 100)
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
                    for key, value in card_data.items():
                        if 'power' in key.lower() or 'watt' in key.lower():
                            val_str = str(value)
                            match = re.search(r'(\d+\.?\d*)', val_str)
                            if match:
                                potential_watt = float(match.group(1))
                                if 10 <= potential_watt <= 500:
                                    power_info['gpu_power'] = potential_watt
                                    break
        except:
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
                        power_info['gpu_power'] = 200 * (gpu_use / 100) if gpu_use else 200
            except:
                power_info['gpu_power'] = 200
        power_info['total_power'] = power_info['cpu_power'] + power_info['gpu_power'] + power_info['other_power']
        power_info['current_power_watt'] = power_info['total_power']
        current_time = psutil.time.time()
        preserved_energy = cumulative_data.get('total_energy_kwh', 0)
        if preserved_energy == 0 and old_total_energy > 0:
            preserved_energy = old_total_energy
            cumulative_data['total_energy_kwh'] = old_total_energy
            print(f"Restored total_energy_kwh from backup: {old_total_energy} kWh")
        preserved_energy = max(preserved_energy, old_total_energy)
        if preserved_energy > cumulative_data.get('total_energy_kwh', 0):
            cumulative_data['total_energy_kwh'] = preserved_energy
        if cumulative_data['session_start_time'] is None:
            if old_session_start is not None:
                cumulative_data['session_start_time'] = old_session_start
                print(f"Restored session_start_time from backup: {old_session_start}")
            else:
                if preserved_energy == 0:
                    cumulative_data['session_start_time'] = current_time
                else:
                    if cumulative_data.get('last_update_time'):
                        cumulative_data['session_start_time'] = cumulative_data['last_update_time'] - 3600
                    else:
                        cumulative_data['session_start_time'] = current_time
        if cumulative_data['last_update_time'] is not None:
            time_diff_hours = (current_time - cumulative_data['last_update_time']) / 3600
            if time_diff_hours > 0:
                avg_power = (cumulative_data['last_power_watt'] + power_info['total_power']) / 2
                energy_increment = (avg_power / 1000) * time_diff_hours
                cumulative_data['total_energy_kwh'] = preserved_energy + energy_increment
            else:
                cumulative_data['total_energy_kwh'] = preserved_energy
        else:
            if preserved_energy > 0:
                cumulative_data['total_energy_kwh'] = preserved_energy
                cumulative_data['last_update_time'] = current_time
            elif cumulative_data['session_start_time']:
                session_hours = (current_time - cumulative_data['session_start_time']) / 3600
                if session_hours > 0:
                    session_energy = (power_info['total_power'] / 1000) * session_hours
                    cumulative_data['total_energy_kwh'] = session_energy
                else:
                    cumulative_data['total_energy_kwh'] = 0
            else:
                cumulative_data['total_energy_kwh'] = 0
        cumulative_data['last_power_watt'] = power_info['total_power']
        cumulative_data['last_update_time'] = current_time
        try:
            backup_file = power_log_file.with_suffix('.json.backup')
            if power_log_file.exists():
                import shutil
                shutil.copy2(power_log_file, backup_file)
            if cumulative_data.get('total_energy_kwh', 0) < preserved_energy:
                print(f"WARNING: Energy would decrease from {preserved_energy} to {cumulative_data.get('total_energy_kwh', 0)}. Restoring preserved value.")
                cumulative_data['total_energy_kwh'] = preserved_energy
            if cumulative_data.get('total_energy_kwh', 0) > 0 and cumulative_data.get('session_start_time') is None:
                if old_session_start is not None:
                    cumulative_data['session_start_time'] = old_session_start
                else:
                    if cumulative_data.get('last_update_time'):
                        cumulative_data['session_start_time'] = cumulative_data['last_update_time'] - 3600
            with open(power_log_file, 'w') as f:
                json.dump(cumulative_data, f, indent=2)
            try:
                with open(power_log_file, 'r') as f:
                    verify_data = json.load(f)
                    if verify_data.get('total_energy_kwh', 0) < preserved_energy:
                        raise ValueError("Data verification failed")
            except:
                if backup_file.exists():
                    import shutil
                    shutil.copy2(backup_file, power_log_file)
                    print(f"Restored power data from backup due to verification failure")
        except Exception as e:
            print(f"Error saving power data: {e}")
            try:
                backup_file = power_log_file.with_suffix('.json.backup')
                if backup_file.exists():
                    import shutil
                    shutil.copy2(backup_file, power_log_file)
                    print(f"Restored power data from backup")
            except Exception as restore_error:
                print(f"Failed to restore from backup: {restore_error}")
        power_info['cumulative_energy_kwh'] = cumulative_data['total_energy_kwh']
        power_info['cumulative_cost_baht'] = power_info['cumulative_energy_kwh'] * power_info['rate_per_kwh']
        power_info['last_update'] = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S')
        if cumulative_data['session_start_time']:
            session_seconds = current_time - cumulative_data['session_start_time']
            power_info['session_runtime_hours'] = session_seconds / 3600
    except Exception as e:
        print(f"Error calculating power: {e}")
    return power_info
def get_training_status():
    
    status = {'running': False}
    try:
        system_mem = psutil.virtual_memory()
        system_mem_percent = system_mem.percent
        system_mem_total_gb = system_mem.total / 1024 / 1024 / 1024
        system_mem_used_gb = system_mem.used / 1024 / 1024 / 1024
        for proc in psutil.process_iter(['pid', 'name', 'memory_info', 'cmdline', 'cpu_percent']):
            try:
                cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                proc_name = proc.name().lower()
                if ('train_sac.py' in cmdline or 'train.py' in cmdline) and 'grep' not in cmdline:
                    if 'python' in proc_name or 'python3' in proc_name:
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
                        break
                    elif not status['running']:
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
        if not status['running']:
            status['memory'] = f"{system_mem_percent:.1f}"
            status['system_mem_total_gb'] = f"{system_mem_total_gb:.1f}"
            status['system_mem_used_gb'] = f"{system_mem_used_gb:.1f}"
    except Exception as e:
        print(f"Error getting training status: {e}")
    return status
@app.route('/')
def index():
    
    react_index = Path(__file__).parent / 'static' / 'react' / 'index.html'
    if react_index.exists():
        return open(react_index, 'r', encoding='utf-8').read()
    else:
        return render_template('dashboard.html')
@app.route('/assets/<path:filename>')
def react_assets(filename):
    
    return send_from_directory(
        Path(__file__).parent / 'static' / 'react' / 'assets',
        filename
    )
@app.route('/api/status')
def api_status():
    
    checkpoint = get_latest_checkpoint()
    log_file = get_latest_training_log()
    metrics = parse_training_metrics(log_file)
    status = get_training_status()
    system_metrics = get_system_metrics()
    power_info = get_power_consumption()
    started = 0
    target = 500000
    metrics_step = metrics.get('current_step', 0) or 0
    checkpoint_step = checkpoint.get('timestep', 0) if checkpoint else 0
    current = max(metrics_step, checkpoint_step)
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
        'algorithm': ALGORITHM,
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    })
@app.route('/api/checkpoints')
def api_checkpoints():
    
    checkpoints = get_all_checkpoints()
    return jsonify({
        'checkpoints': checkpoints,
        'count': len(checkpoints)
    })
@app.route('/api/metrics/history')
def api_metrics_history():
    
    log_file = get_latest_training_log()
    if not log_file or not log_file.exists():
        checkpoints = get_all_checkpoints()
        history = {
            'timesteps': [c['timestep'] for c in checkpoints],
            'rewards': [c['reward'] if c['reward'] else 0 for c in checkpoints],
            'episodes': [c['episode'] if c['episode'] else 0 for c in checkpoints],
            'dates': [c['created_at'] for c in checkpoints]
        }
        return jsonify(history)
    timesteps = []
    rewards = []
    episodes = []
    dates = []
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
            for line in lines:
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
                        date_match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', line)
                        if date_match:
                            dates.append(date_match.group(1))
                        else:
                            dates.append(None)
            i = 0
            while i < len(lines):
                if '| rollout/' in lines[i]:
                    table_metrics = {}
                    for j in range(i, min(i+30, len(lines))):
                        line = lines[j]
                        if 'total_timesteps' in line and '|' in line:
                            match = re.search(r'total_timesteps\s*\|\s*(\d+)', line)
                            if match:
                                table_metrics['timestep'] = int(match.group(1))
                        if 'ep_rew_mean' in line and '|' in line:
                            match = re.search(r'ep_rew_mean\s*\|\s*([-\d.e+]+)', line)
                            if match:
                                table_metrics['reward'] = float(match.group(1))
                        if 'ep_len_mean' in line and '|' in line:
                            match = re.search(r'ep_len_mean\s*\|\s*([\d.]+)', line)
                            if match:
                                table_metrics['episode'] = int(float(match.group(1)))
                        if len(table_metrics) == 3:
                            break
                    if len(table_metrics) == 3:
                        ts = table_metrics['timestep']
                        if ts not in timesteps:
                            timesteps.append(ts)
                            rewards.append(table_metrics['reward'])
                            episodes.append(table_metrics['episode'])
                            dates.append(None)
                i += 1
        if timesteps:
            sorted_data = sorted(zip(timesteps, rewards, episodes, dates))
            timesteps, rewards, episodes, dates = zip(*sorted_data)
            timesteps = list(timesteps)
            rewards = list(rewards)
            episodes = list(episodes)
            dates = list(dates)
    except Exception as e:
        print(f"Error parsing training history: {e}")
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
@app.route('/api/demo/reset', methods=['POST'])
def api_demo_reset():
    
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
        return jsonify({
            'success': False,
            'error': 'Demo simulator not available',
            'message': 'Demo feature is disabled or not properly configured'
        }), 200
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Demo reset error: {error_msg}")
        return jsonify({
            'success': False,
            'error': str(e),
            'message': 'Failed to reset demo simulation'
        }), 200
@app.route('/api/demo/step', methods=['POST'])
def api_demo_step():
    
    try:
        import sys
        from pathlib import Path
        demo_path = Path(__file__).parent
        if str(demo_path) not in sys.path:
            sys.path.insert(0, str(demo_path))
        from demo_simulator import step_demo
        data = request.get_json() or {}
        action = data.get('action')
        if action is None or (isinstance(action, list) and len(action) == 0):
            action = None
        state = step_demo(action)
        return jsonify({'success': True, 'state': state})
    except ImportError as e:
        return jsonify({
            'success': False,
            'error': 'Demo simulator not available',
            'message': 'Demo feature is disabled or not properly configured'
        }), 200
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Demo step error: {error_msg}")
        return jsonify({
            'success': False,
            'error': str(e),
            'message': 'Failed to step demo simulation'
        }), 200
@app.route('/api/demo/state', methods=['GET'])
def api_demo_state():
    
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
        return jsonify({
            'success': False,
            'error': 'Demo simulator not available',
            'message': 'Demo feature is disabled or not properly configured'
        }), 200
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Demo state error: {error_msg}")
        return jsonify({
            'success': False,
            'error': str(e),
            'message': 'Failed to get demo state'
        }), 200
@app.route('/api/logs/auto_manage')
def api_auto_manage_log():
    
    auto_manage_log = LOG_DIR / "auto_manage.log"
    try:
        lines = []
        if auto_manage_log.exists():
            with open(auto_manage_log, 'r', encoding='utf-8', errors='ignore') as f:
                all_lines = f.readlines()
                lines = all_lines[-100:] if len(all_lines) > 100 else all_lines
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