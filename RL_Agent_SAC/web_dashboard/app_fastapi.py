from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi import Request
from fastapi.middleware.cors import CORSMiddleware
import sqlite3
import os
import json
import re
from datetime import datetime
from pathlib import Path
import glob
import subprocess
import psutil
from typing import Optional, List, Dict, Any
import asyncio
from contextlib import asynccontextmanager
import sys
import importlib.util
flask_app_path = Path(__file__).parent / "app.py"
if flask_app_path.exists():
    spec = importlib.util.spec_from_file_location("flask_app", flask_app_path)
    flask_app = importlib.util.module_from_spec(spec)
    sys.modules["flask_app"] = flask_app
    spec.loader.exec_module(flask_app)
    get_latest_checkpoint = flask_app.get_latest_checkpoint
    get_all_checkpoints = flask_app.get_all_checkpoints
    get_latest_training_log = flask_app.get_latest_training_log
    parse_training_metrics = flask_app.parse_training_metrics
    get_system_metrics = flask_app.get_system_metrics
    get_power_consumption = flask_app.get_power_consumption
    get_training_status = flask_app.get_training_status
else:
    def get_latest_checkpoint():
        return None
    def get_all_checkpoints():
        return []
    def get_latest_training_log():
        return None
    def parse_training_metrics(log_file):
        return {}
    def get_system_metrics():
        return {'cpu': {}, 'gpu': {}}
    def get_power_consumption():
        return {}
    def get_training_status():
        return {'running': False}
BASE_DIR = Path(__file__).parent.parent
TEMPLATE_DIR = BASE_DIR / "web_dashboard" / "templates"
STATIC_DIR = BASE_DIR / "web_dashboard" / "static"
REACT_BUILD_DIR = STATIC_DIR / "react"
ALGORITHM = "SAC"
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("=" * 70)
    print("🚀 Starting FastAPI Training Dashboard - SAC")
    print("=" * 70)
    print(f"📊 Dashboard: http://localhost:5001")
    print(f"📡 API: http://localhost:5001/api/status")
    print(f"📚 API Docs: http://localhost:5001/docs")
    print(f"🎯 Algorithm: SAC (Soft Actor-Critic)")
    print()
    print("✅ FastAPI Dashboard - High Performance Mode")
    print("   - Async/await support")
    print("   - Lower resource usage (~30-50% less memory)")
    print("   - Better performance (~4x faster)")
    print("   - SAC-specific log parsing")
    if REACT_BUILD_DIR.exists() and (REACT_BUILD_DIR / "index.html").exists():
        print("✅ React Dashboard: Enabled")
    else:
        print("⚠️  React Dashboard: Not built (using fallback)")
    print("=" * 70)
    yield
    print("Shutting down FastAPI Dashboard...")
app = FastAPI(
    title="RL Training Dashboard",
    description="High Performance Training Dashboard with FastAPI",
    version="2.0.0",
    lifespan=lifespan
)
templates = Jinja2Templates(directory=str(TEMPLATE_DIR))
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
if REACT_BUILD_DIR.exists():
    app.mount("/assets", StaticFiles(directory=str(REACT_BUILD_DIR / "assets")), name="react_assets")
    if (REACT_BUILD_DIR / "index.html").exists():
        print("✅ React build detected - serving React dashboard")
    else:
        print("⚠️  React build directory exists but index.html not found")
else:
    print("⚠️  React build not found - falling back to old dashboard")
@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    
    react_index = REACT_BUILD_DIR / "index.html"
    if react_index.exists():
        return FileResponse(react_index)
    else:
        return templates.TemplateResponse("dashboard.html", {"request": request})
@app.get("/api/status")
async def api_status():
    
    loop = asyncio.get_event_loop()
    checkpoint = await loop.run_in_executor(None, get_latest_checkpoint)
    log_file = await loop.run_in_executor(None, get_latest_training_log)
    metrics = await loop.run_in_executor(None, parse_training_metrics, log_file)
    status = await loop.run_in_executor(None, get_training_status)
    system_metrics = await loop.run_in_executor(None, get_system_metrics)
    power_info = await loop.run_in_executor(None, get_power_consumption)
    started = 0
    target = 500000
    metrics_step = metrics.get('current_step', 0) or 0
    checkpoint_step = checkpoint.get('timestep', 0) if checkpoint else 0
    current = max(metrics_step, checkpoint_step)
    progress = ((current - started) / (target - started)) * 100 if current > started and target > started else 0
    return {
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
    }
@app.get("/api/checkpoints")
async def api_checkpoints():
    
    loop = asyncio.get_event_loop()
    checkpoints = await loop.run_in_executor(None, get_all_checkpoints)
    return {
        'checkpoints': checkpoints,
        'count': len(checkpoints)
    }
@app.get("/api/metrics/history")
async def api_metrics_history():
    
    loop = asyncio.get_event_loop()
    log_file = await loop.run_in_executor(None, get_latest_training_log)
    if not log_file or not log_file.exists():
        checkpoints = await loop.run_in_executor(None, get_all_checkpoints)
        history = {
            'timesteps': [c['timestep'] for c in checkpoints],
            'rewards': [c['reward'] if c['reward'] else 0 for c in checkpoints],
            'episodes': [c['episode'] if c['episode'] else 0 for c in checkpoints],
            'dates': [c['created_at'] for c in checkpoints]
        }
        return history
    def parse_history_from_log(log_file):
        import re
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
        return {
            'timesteps': timesteps,
            'rewards': rewards,
            'episodes': episodes,
            'dates': dates
        }
    history = await loop.run_in_executor(None, parse_history_from_log, log_file)
    return history
@app.post("/api/demo/reset")
async def api_demo_reset():
    
    return {'success': False, 'error': '2D Demo is disabled'}
@app.post("/api/demo/step")
async def api_demo_step(request: Request):
    
    return {'success': False, 'error': '2D Demo is disabled'}
@app.get("/api/demo/state")
async def api_demo_state():
    
    return {'success': False, 'error': '2D Demo is disabled'}
@app.get("/api/logs/auto_manage")
async def api_auto_manage_log():
    
    LOG_DIR = BASE_DIR / "logs"
    auto_manage_log = LOG_DIR / "auto_manage.log"
    try:
        loop = asyncio.get_event_loop()
        def read_log():
            lines = []
            if auto_manage_log.exists():
                with open(auto_manage_log, 'r', encoding='utf-8', errors='ignore') as f:
                    all_lines = f.readlines()
                    lines = all_lines[-100:] if len(all_lines) > 100 else all_lines
                mtime = auto_manage_log.stat().st_mtime
                last_update = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
            else:
                last_update = None
            return {
                'success': True,
                'lines': [line.rstrip('\n') for line in lines],
                'count': len(lines),
                'last_update': last_update,
                'file_exists': auto_manage_log.exists()
            }
        result = await loop.run_in_executor(None, read_log)
        return result
    except Exception as e:
        return {
            'success': False,
            'error': str(e),
            'lines': [],
            'count': 0
        }
@app.get("/api/camera/live")
async def get_live_camera():
    
    LOG_DIR = BASE_DIR / "logs"
    snapshot_path = LOG_DIR / "live_camera.jpg"
    if snapshot_path.exists():
        return FileResponse(snapshot_path, media_type="image/jpeg")
    else:
        return {"error": "Camera offline"}
@app.get("/api/evaluations")
async def get_evaluations():
    
    LOG_DIR = BASE_DIR / "logs"
    eval_dir = LOG_DIR / "evaluations"
    if not eval_dir.exists():
        return {"latest": None, "history": []}
    reports = []
    for f in eval_dir.glob("eval_report_*.json"):
        try:
            with open(f, 'r') as fp:
                data = json.load(fp)
                reports.append(data)
        except: pass
    reports.sort(key=lambda x: x.get('timestamp', ''), reverse=True)
    return {
        "latest": reports[0] if reports else None,
        "history": reports[:5]
    }
if __name__ == '__main__':
    import uvicorn
    print("=" * 70)
    print("🚀 Starting FastAPI Training Dashboard")
    print("=" * 70)
    print(f"📊 Dashboard: http://localhost:5000")
    print(f"📡 API: http://localhost:5000/api/status")
    print(f"📚 API Docs: http://localhost:5000/docs")
    print(f"🎮 Demo: http://localhost:5000#demo")
    print()
    print("✅ FastAPI Dashboard - High Performance Mode")
    if REACT_BUILD_DIR.exists() and (REACT_BUILD_DIR / "index.html").exists():
        print("✅ React Dashboard: Enabled")
    else:
        print("⚠️  React Dashboard: Not built (using fallback)")
    print("=" * 70)
    uvicorn.run(
        "app_fastapi:app",
        host="0.0.0.0",
        port=5001,
        log_level="info",
        workers=1,
        loop="asyncio"
    )