import torch
import logging
from typing import Optional, Tuple
import psutil
import time
class MixedDeviceManager:
    
    def __init__(
        self,
        gpu_device: torch.device,
        cpu_device: torch.device = torch.device('cpu'),
        gpu_memory_threshold: float = 0.85,
        gpu_util_threshold: float = 0.90,
        check_interval: int = 100
    ):
        
        self.gpu_device = gpu_device
        self.cpu_device = cpu_device
        self.gpu_memory_threshold = gpu_memory_threshold
        self.gpu_util_threshold = gpu_util_threshold
        self.check_interval = check_interval
        self.use_mixed = False
        self.gpu_memory_usage = 0.0
        self.gpu_utilization = 0.0
        self.last_check_step = 0
        self.cpu_batch_ratio = 0.0
        self.stats = {
            'gpu_batches': 0,
            'cpu_batches': 0,
            'gpu_oom_events': 0,
            'cpu_fallback_events': 0
        }
        logging.info("✅ Mixed Device Manager initialized")
        logging.info(f"   GPU: {gpu_device}")
        logging.info(f"   CPU: {cpu_device}")
        logging.info(f"   Memory threshold: {gpu_memory_threshold*100:.1f}%")
        logging.info(f"   Utilization threshold: {gpu_util_threshold*100:.1f}%")
    def get_gpu_memory_usage(self) -> float:
        
        if self.gpu_device.type != 'cuda':
            return 0.0
        try:
            gpu_id = self.gpu_device.index if self.gpu_device.index is not None else 0
            import subprocess
            import re
            try:
                cmd = ['rocm-smi', '--showmemuse']
                if gpu_id > 0:
                    cmd.extend(['-d', str(gpu_id)])
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'VRAM%' in line and 'Memory Allocated' in line:
                            match = re.search(r'VRAM%\):\s*(\d+)', line)
                            if not match:
                                match = re.search(r':\s*(\d+)%', line)
                            if match:
                                usage = float(match.group(1)) / 100.0
                                logging.info(f"Got GPU memory from rocm-smi: {usage*100:.1f}%")
                                return min(usage, 1.0)
            except (subprocess.TimeoutExpired, FileNotFoundError, ValueError) as e:
                logging.debug(f"rocm-smi failed: {e}")
                pass
            try:
                result = subprocess.run(
                    ['nvidia-smi', '--query-gpu=memory.used,memory.total',
                     '--format=csv,noheader,nounits', f'--id={gpu_id}'],
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                if result.returncode == 0:
                    parts = result.stdout.strip().split(', ')
                    if len(parts) == 2:
                        used = float(parts[0])
                        total = float(parts[1])
                        usage = used / total if total > 0 else 0.0
                        return min(usage, 1.0)
            except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
                pass
            allocated = torch.cuda.memory_allocated(gpu_id)
            reserved = torch.cuda.memory_reserved(gpu_id)
            total = torch.cuda.get_device_properties(gpu_id).total_memory
            usage = reserved / total if total > 0 else 0.0
            return min(usage, 1.0)
        except Exception as e:
            logging.debug(f"Failed to get GPU memory usage: {e}")
            return 0.0
    def get_gpu_utilization(self) -> float:
        
        if self.gpu_device.type != 'cuda':
            return 0.0
        try:
            import subprocess
            gpu_id = self.gpu_device.index if self.gpu_device.index is not None else 0
            try:
                result = subprocess.run(
                    ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits', f'--id={gpu_id}'],
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                if result.returncode == 0:
                    util = float(result.stdout.strip()) / 100.0
                    return min(util, 1.0)
            except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
                pass
            try:
                cmd = ['rocm-smi', '--showuse']
                if gpu_id > 0:
                    cmd.extend(['-d', str(gpu_id)])
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'GPU use' in line or 'GPU Use' in line:
                            match = re.search(r'GPU use \(%\):\s*(\d+)', line)
                            if not match:
                                match = re.search(r':\s*(\d+)%', line)
                            if match:
                                util = float(match.group(1)) / 100.0
                                logging.debug(f"Got GPU utilization from rocm-smi: {util*100:.1f}%")
                                return min(util, 1.0)
            except (subprocess.TimeoutExpired, FileNotFoundError, ValueError) as e:
                logging.debug(f"rocm-smi utilization failed: {e}")
                pass
            current_allocated = torch.cuda.memory_allocated(self.gpu_device.index or 0)
            time.sleep(0.01)
            new_allocated = torch.cuda.memory_allocated(self.gpu_device.index or 0)
            if abs(new_allocated - current_allocated) > 1024 * 1024:
                return 0.8
            else:
                return 0.3
        except Exception as e:
            logging.debug(f"Failed to get GPU utilization: {e}")
            return 0.5
    def should_use_cpu(self, step: int) -> Tuple[bool, float]:
        
        if step - self.last_check_step < self.check_interval:
            return self.use_mixed, self.cpu_batch_ratio
        self.last_check_step = step
        self.gpu_memory_usage = self.get_gpu_memory_usage()
        self.gpu_utilization = self.get_gpu_utilization()
        if not self.use_mixed:
            logging.info(f"🔄 Enabling mixed GPU/CPU mode (always on)")
            logging.info(f"   GPU Memory: {self.gpu_memory_usage*100:.1f}%")
            logging.info(f"   GPU Utilization: {self.gpu_utilization*100:.1f}%")
        self.use_mixed = True
        memory_over_threshold = self.gpu_memory_usage > self.gpu_memory_threshold
        util_over_threshold = self.gpu_utilization > self.gpu_util_threshold
        if memory_over_threshold:
            excess_memory = (self.gpu_memory_usage - self.gpu_memory_threshold) / (1.0 - self.gpu_memory_threshold)
            self.cpu_batch_ratio = min(excess_memory * 0.5, 0.5)
        elif util_over_threshold:
            excess_util = (self.gpu_utilization - self.gpu_util_threshold) / (1.0 - self.gpu_util_threshold)
            self.cpu_batch_ratio = min(excess_util * 0.3, 0.3)
        else:
            self.cpu_batch_ratio = 0.3
        return self.use_mixed, self.cpu_batch_ratio
    def get_device_for_batch(self, batch_idx: int, total_batches: int) -> torch.device:
        
        if not self.use_mixed:
            return self.gpu_device
        cpu_batch_count = int(total_batches * self.cpu_batch_ratio)
        if cpu_batch_count > 0:
            cpu_batch_indices = [
                int(i * total_batches / cpu_batch_count)
                for i in range(cpu_batch_count)
            ]
            if batch_idx in cpu_batch_indices:
                self.stats['cpu_batches'] += 1
                return self.cpu_device
        self.stats['gpu_batches'] += 1
        return self.gpu_device
    def handle_oom(self, step: int) -> bool:
        
        self.stats['gpu_oom_events'] += 1
        logging.warning(f"⚠️  GPU OOM at step {step} - enabling CPU fallback")
        if self.gpu_device.type == 'cuda':
            torch.cuda.empty_cache()
        self.use_mixed = True
        self.cpu_batch_ratio = min(self.cpu_batch_ratio + 0.2, 0.7)
        return True
    def get_stats(self) -> dict:
        
        return {
            **self.stats,
            'gpu_memory_usage': self.gpu_memory_usage,
            'gpu_utilization': self.gpu_utilization,
            'use_mixed': self.use_mixed,
            'cpu_batch_ratio': self.cpu_batch_ratio
        }
    def log_stats(self, step: int):
        
        stats = self.get_stats()
        logging.info(f"📊 Mixed Device Stats (step {step}):")
        logging.info(f"   GPU Memory: {stats['gpu_memory_usage']*100:.1f}%")
        logging.info(f"   GPU Utilization: {stats['gpu_utilization']*100:.1f}%")
        logging.info(f"   Mixed Mode: {stats['use_mixed']}")
        logging.info(f"   CPU Batch Ratio: {stats['cpu_batch_ratio']*100:.1f}%")
        logging.info(f"   GPU Batches: {stats['gpu_batches']}, CPU Batches: {stats['cpu_batches']}")
        logging.info(f"   OOM Events: {stats['gpu_oom_events']}, CPU Fallbacks: {stats['cpu_fallback_events']}")