#!/usr/bin/env python3
"""
Analyze training logs to find why training stopped.
"""
import re
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def find_training_logs(log_dir: Path) -> List[Path]:
    """Find all training log files"""
    return sorted(log_dir.glob("sac_training_*.log"), key=lambda x: x.stat().st_mtime, reverse=True)

def analyze_log_file(log_path: Path) -> Dict:
    """Analyze a single log file for errors and issues"""
    result = {
        'file': log_path.name,
        'size_mb': log_path.stat().st_size / (1024 * 1024),
        'errors': [],
        'warnings': [],
        'last_lines': [],
        'crash_reason': None,
        'last_step': None,
        'training_duration': None
    }
    
    try:
        with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        
        if not lines:
            result['errors'].append("Empty log file")
            return result
        
        # Get last 20 lines
        result['last_lines'] = lines[-20:]
        
        # Find errors
        error_patterns = [
            r'ERROR',
            r'Exception',
            r'Traceback',
            r'RuntimeError',
            r'TypeError',
            r'ValueError',
            r'KeyError',
            r'AttributeError',
            r'Failed',
            r'Crash',
            r'cannot be multiplied',
            r'dimension',
            r'mat1.*mat2'
        ]
        
        for i, line in enumerate(lines):
            for pattern in error_patterns:
                if re.search(pattern, line, re.IGNORECASE):
                    result['errors'].append(f"Line {i+1}: {line.strip()}")
                    break
        
        # Find warnings
        for i, line in enumerate(lines):
            if 'WARNING' in line or 'WARN' in line:
                result['warnings'].append(f"Line {i+1}: {line.strip()}")
        
        # Find last training step
        step_pattern = r'step[=:\s]+(\d+)'
        for line in reversed(lines):
            match = re.search(step_pattern, line, re.IGNORECASE)
            if match:
                result['last_step'] = int(match.group(1))
                break
        
        # Find crash reason
        crash_patterns = [
            (r'RuntimeError.*mat1.*mat2', 'Dimension mismatch in neural network'),
            (r'RuntimeError.*shape', 'Shape mismatch error'),
            (r'Exception.*type.*RuntimeError', 'Runtime error'),
            (r'Training.*stopped', 'Training stopped'),
            (r'Training.*complete', 'Training completed'),
            (r'KeyboardInterrupt', 'Manually stopped'),
            (r'OutOfMemoryError', 'Out of memory'),
            (r'CUDA.*out.*memory', 'GPU out of memory'),
        ]
        
        for pattern, reason in crash_patterns:
            for line in reversed(lines):
                if re.search(pattern, line, re.IGNORECASE):
                    result['crash_reason'] = reason
                    break
            if result['crash_reason']:
                break
        
        # Calculate training duration
        time_pattern = r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})'
        times = []
        for line in lines:
            match = re.search(time_pattern, line)
            if match:
                try:
                    times.append(datetime.strptime(match.group(1), '%Y-%m-%d %H:%M:%S'))
                except:
                    pass
        
        if len(times) >= 2:
            duration = times[-1] - times[0]
            result['training_duration'] = str(duration)
        
    except Exception as e:
        result['errors'].append(f"Failed to analyze log: {e}")
    
    return result

def main():
    base_dir = Path(__file__).parent.parent
    log_dir = base_dir / "logs"
    
    logger.info("🔍 Training Log Analysis")
    logger.info("=" * 60)
    
    log_files = find_training_logs(log_dir)
    
    if not log_files:
        logger.warning("No training log files found")
        return
    
    logger.info(f"Found {len(log_files)} training log file(s)")
    logger.info("")
    
    # Analyze latest 5 logs
    for log_file in log_files[:5]:
        logger.info(f"📄 Analyzing: {log_file.name}")
        result = analyze_log_file(log_file)
        
        logger.info(f"   Size: {result['size_mb']:.2f} MB")
        
        if result['last_step']:
            logger.info(f"   Last step: {result['last_step']}")
        
        if result['training_duration']:
            logger.info(f"   Duration: {result['training_duration']}")
        
        if result['crash_reason']:
            logger.warning(f"   ⚠️  Crash reason: {result['crash_reason']}")
        
        if result['errors']:
            logger.error(f"   ❌ Found {len(result['errors'])} error(s):")
            for error in result['errors'][:5]:  # Show first 5
                logger.error(f"      {error}")
        
        if result['warnings']:
            logger.warning(f"   ⚠️  Found {len(result['warnings'])} warning(s)")
        
        logger.info("")
    
    # Summary
    logger.info("📊 Summary:")
    logger.info(f"   Total log files: {len(log_files)}")
    logger.info(f"   Analyzed: {min(5, len(log_files))}")

if __name__ == "__main__":
    main()

