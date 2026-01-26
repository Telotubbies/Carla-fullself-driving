"""
Production-Ready FastAPI Dashboard for CARLA SAC Training
Features:
- Error handling & logging
- Caching & rate limiting
- Health checks & monitoring
- Security headers
- Performance optimization
- Real-time updates
"""
from fastapi import FastAPI, HTTPException, Request, Depends
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.security import HTTPBearer
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from slowapi.middleware import SlowAPIMiddleware
import sqlite3
import os
import json
import re
import logging
from datetime import datetime, timedelta
from pathlib import Path
import glob
import subprocess
import psutil
from typing import Optional, List, Dict, Any
import asyncio
from contextlib import asynccontextmanager
import sys
import importlib.util
from functools import lru_cache
import time
from cachetools import TTLCache
import traceback

# ============================================================================
# Configuration
# ============================================================================
BASE_DIR = Path(__file__).parent.parent
TEMPLATE_DIR = BASE_DIR / "web_dashboard" / "templates"
STATIC_DIR = BASE_DIR / "web_dashboard" / "static"
REACT_BUILD_DIR = STATIC_DIR / "react"
ALGORITHM = "SAC"

# Production settings
PRODUCTION = os.getenv("PRODUCTION", "false").lower() == "true"
DEBUG = os.getenv("DEBUG", "false").lower() == "true"
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
API_RATE_LIMIT = os.getenv("API_RATE_LIMIT", "100/minute")
CACHE_TTL = int(os.getenv("CACHE_TTL", "5"))  # seconds

# ============================================================================
# Logging Setup
# ============================================================================
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format='%(asctime)s | %(name)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.FileHandler(BASE_DIR / "logs" / "dashboard_production.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("DashboardProduction")

# ============================================================================
# Import Flask App Functions (Legacy Support)
# ============================================================================
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

# ============================================================================
# Caching
# ============================================================================
cache = TTLCache(maxsize=100, ttl=CACHE_TTL)

def get_cached_or_compute(key: str, compute_func, *args, **kwargs):
    """Get from cache or compute and cache"""
    if key in cache:
        return cache[key]
    result = compute_func(*args, **kwargs)
    cache[key] = result
    return result

# ============================================================================
# Rate Limiting
# ============================================================================
limiter = Limiter(key_func=get_remote_address)

# ============================================================================
# FastAPI App Setup
# ============================================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    logger.info("=" * 70)
    logger.info("🚀 Starting Production FastAPI Dashboard - SAC")
    logger.info("=" * 70)
    logger.info(f"📊 Dashboard: http://localhost:5001")
    logger.info(f"📡 API: http://localhost:5001/api/status")
    logger.info(f"📚 API Docs: http://localhost:5001/docs")
    logger.info(f"🏥 Health: http://localhost:5001/health")
    logger.info(f"🎯 Algorithm: SAC (Soft Actor-Critic)")
    logger.info(f"🔧 Mode: {'PRODUCTION' if PRODUCTION else 'DEVELOPMENT'}")
    logger.info(f"📝 Log Level: {LOG_LEVEL}")
    logger.info(f"⚡ Rate Limit: {API_RATE_LIMIT}")
    logger.info(f"💾 Cache TTL: {CACHE_TTL}s")
    
    if REACT_BUILD_DIR.exists() and (REACT_BUILD_DIR / "index.html").exists():
        logger.info("✅ React Dashboard: Enabled")
    else:
        logger.warning("⚠️  React Dashboard: Not built (using fallback)")
    
    logger.info("=" * 70)
    yield
    logger.info("Shutting down Production FastAPI Dashboard...")

app = FastAPI(
    title="RL Training Dashboard (Production)",
    description="Production-Ready Training Dashboard with FastAPI",
    version="3.0.0",
    lifespan=lifespan,
    docs_url="/docs" if not PRODUCTION else None,  # Disable docs in production
    redoc_url="/redoc" if not PRODUCTION else None,
)

# ============================================================================
# Middleware
# ============================================================================
# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "http://localhost:5173,http://localhost:3000").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Compression
app.add_middleware(GZipMiddleware, minimum_size=1000)

# Rate Limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)
app.add_middleware(SlowAPIMiddleware)

# Security Headers (Production)
if PRODUCTION:
    @app.middleware("http")
    async def add_security_headers(request: Request, call_next):
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        return response

# ============================================================================
# Static Files
# ============================================================================
if REACT_BUILD_DIR.exists():
    app.mount("/assets", StaticFiles(directory=str(REACT_BUILD_DIR / "assets")), name="react_assets")

# ============================================================================
# Error Handlers
# ============================================================================
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler"""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "message": str(exc) if DEBUG else "An error occurred",
            "path": str(request.url),
            "timestamp": datetime.now().isoformat()
        }
    )

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """HTTP exception handler"""
    logger.warning(f"HTTP {exc.status_code}: {exc.detail} - {request.url}")
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.detail,
            "path": str(request.url),
            "timestamp": datetime.now().isoformat()
        }
    )

# ============================================================================
# Health Check Endpoints
# ============================================================================
@app.get("/health")
async def health_check():
    """Health check endpoint"""
    try:
        # Check database
        db_path = BASE_DIR / "checkpoints" / "training_checkpoints.db"
        db_ok = db_path.exists() if db_path else True
        
        # Check logs directory
        logs_dir = BASE_DIR / "logs"
        logs_ok = logs_dir.exists() and os.access(logs_dir, os.W_OK)
        
        # Check system resources
        mem = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        
        health_status = {
            "status": "healthy" if (db_ok and logs_ok) else "degraded",
            "timestamp": datetime.now().isoformat(),
            "checks": {
                "database": "ok" if db_ok else "error",
                "logs": "ok" if logs_ok else "error",
                "memory": {
                    "percent": mem.percent,
                    "available_gb": round(mem.available / (1024**3), 2)
                },
                "disk": {
                    "percent": (disk.used / disk.total) * 100,
                    "free_gb": round(disk.free / (1024**3), 2)
                }
            },
            "version": "3.0.0",
            "algorithm": ALGORITHM
        }
        
        status_code = 200 if health_status["status"] == "healthy" else 503
        return JSONResponse(content=health_status, status_code=status_code)
    except Exception as e:
        logger.error(f"Health check failed: {e}", exc_info=True)
        return JSONResponse(
            status_code=503,
            content={
                "status": "unhealthy",
                "error": str(e),
                "timestamp": datetime.now().isoformat()
            }
        )

@app.get("/health/ready")
async def readiness_check():
    """Readiness check (for Kubernetes)"""
    return {"status": "ready", "timestamp": datetime.now().isoformat()}

@app.get("/health/live")
async def liveness_check():
    """Liveness check (for Kubernetes)"""
    return {"status": "alive", "timestamp": datetime.now().isoformat()}

# ============================================================================
# Main Routes
# ============================================================================
@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    """Serve React dashboard or fallback"""
    react_index = REACT_BUILD_DIR / "index.html"
    if react_index.exists():
        return FileResponse(react_index)
    else:
        from fastapi.templating import Jinja2Templates
        templates = Jinja2Templates(directory=str(TEMPLATE_DIR))
        return templates.TemplateResponse("dashboard.html", {"request": request})

# ============================================================================
# API Endpoints
# ============================================================================
@app.get("/api/status")
@limiter.limit(API_RATE_LIMIT)
async def api_status(request: Request):
    """Get training status with caching"""
    try:
        cache_key = "api_status"
        
        async def compute_status():
            loop = asyncio.get_event_loop()
            checkpoint = await loop.run_in_executor(None, get_latest_checkpoint)
            log_file = await loop.run_in_executor(None, get_latest_training_log)
            metrics = await loop.run_in_executor(None, parse_training_metrics, log_file)
            status = await loop.run_in_executor(None, get_training_status)
            system_metrics = await loop.run_in_executor(None, get_system_metrics)
            power_info = await loop.run_in_executor(None, get_power_consumption)
            
            # Transform system_metrics
            import psutil
            mem = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            cpu_percent = system_metrics.get('cpu', {}).get('usage', 0)
            
            # Get CPU temperature (not GPU)
            cpu_temp = None
            if 'cpu' in system_metrics and system_metrics['cpu'].get('temp') is not None:
                cpu_temp = system_metrics['cpu'].get('temp')
            
            transformed_system = {
                'cpu': {
                    'usage': cpu_percent,
                    'percent': cpu_percent,
                    'used_gb': round(mem.used / (1024**3), 2),
                    'total_gb': round(mem.total / (1024**3), 2),
                    'free_gb': round(mem.available / (1024**3), 2),
                    'temperature': round(cpu_temp, 1) if cpu_temp is not None else None,
                },
                'memory': {
                    'percent': mem.percent,
                    'used_gb': mem.used / (1024**3),
                    'total_gb': mem.total / (1024**3),
                    'free_gb': mem.available / (1024**3),
                },
                'disk': {
                    'used_percent': (disk.used / disk.total) * 100,
                    'used_gb': disk.used / (1024**3),
                    'total_gb': disk.total / (1024**3),
                    'free_gb': disk.free / (1024**3),
                },
            }
            
            # Add GPU if available
            if 'gpu' in system_metrics and system_metrics['gpu'].get('name'):
                gpu_data = system_metrics['gpu']
                # Safely get GPU memory values - fix 0.0/0.0 GB bug
                gpu_memory_used = None
                gpu_memory_total = None
                
                # Try memory_used_mb first (more reliable)
                if gpu_data.get('memory_used_mb') is not None:
                    gpu_memory_used = float(gpu_data.get('memory_used_mb', 0)) / 1024.0
                elif gpu_data.get('memory_used') is not None:
                    gpu_memory_used = float(gpu_data.get('memory_used', 0))
                    if gpu_memory_used < 0.1:  # If < 0.1 GB, might be in MB
                        gpu_memory_used = gpu_memory_used * 1024.0 / 1024.0  # Already in GB
                
                # Try memory_total_mb first (more reliable)
                if gpu_data.get('memory_total_mb') is not None:
                    gpu_memory_total = float(gpu_data.get('memory_total_mb', 0)) / 1024.0
                elif gpu_data.get('memory_total') is not None:
                    gpu_memory_total = float(gpu_data.get('memory_total', 0))
                    if gpu_memory_total < 0.1:  # If < 0.1 GB, might be in MB
                        gpu_memory_total = gpu_memory_total * 1024.0 / 1024.0  # Already in GB
                
                # Fallback: use GPU name to guess memory (if known)
                if gpu_memory_total is None or gpu_memory_total == 0:
                    gpu_name = gpu_data.get('name', '').lower()
                    if '7800 xt' in gpu_name or 'rx 7800' in gpu_name:
                        gpu_memory_total = 16.0  # 16 GB for RX 7800 XT
                    elif '6800' in gpu_name:
                        gpu_memory_total = 16.0
                    elif '6700' in gpu_name:
                        gpu_memory_total = 12.0
                
                # Only add GPU if we have valid data
                if gpu_memory_total and gpu_memory_total > 0:
                    transformed_system['gpu'] = [{
                        'name': gpu_data.get('name', 'Unknown'),
                        'memory_used': round(gpu_memory_used or 0.0, 2),
                        'memory_total': round(gpu_memory_total, 2),
                        'utilization': gpu_data.get('usage', 0) or 0,
                        'temperature': gpu_data.get('temp', 0) or 0,
                    }]
            
            # Transform power info
            transformed_power = {
                'power_draw': power_info.get('current_power_watt', 0),
                'energy_used': power_info.get('cumulative_energy_kwh', 0),
                'cost_estimate': power_info.get('cumulative_cost_baht', 0),
            }
            
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
                'system': transformed_system,
                'power': transformed_power,
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
        
        # Use cache if available
        if cache_key in cache:
            result = cache[cache_key]
        else:
            result = await compute_status()
            cache[cache_key] = result
        
        return result
    except Exception as e:
        logger.error(f"Error in api_status: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/checkpoints")
@limiter.limit(API_RATE_LIMIT)
async def api_checkpoints(request: Request):
    """Get all checkpoints"""
    try:
        loop = asyncio.get_event_loop()
        checkpoints = await loop.run_in_executor(None, get_all_checkpoints)
        return {
            'checkpoints': checkpoints,
            'count': len(checkpoints),
            'timestamp': datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Error in api_checkpoints: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/metrics/history")
@limiter.limit(API_RATE_LIMIT)
async def api_metrics_history(request: Request):
    """Get training metrics history"""
    try:
        loop = asyncio.get_event_loop()
        log_file = await loop.run_in_executor(None, get_latest_training_log)
        
        if not log_file or not log_file.exists():
            checkpoints = await loop.run_in_executor(None, get_all_checkpoints)
            result = [{
                'step': c['timestep'],
                'reward': c['reward'] if c['reward'] else 0,
                'timestamp': c['created_at']
            } for c in checkpoints]
            return result
        
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
                                dates.append(date_match.group(1) if date_match else None)
            except Exception as e:
                logger.error(f"Error parsing history: {e}")
                checkpoints = get_all_checkpoints()
                timesteps = [c['timestep'] for c in checkpoints]
                rewards = [c['reward'] if c['reward'] else 0 for c in checkpoints]
                episodes = [c['episode'] if c['episode'] else 0 for c in checkpoints]
                dates = [c['created_at'] for c in checkpoints]
            
            if timesteps:
                sorted_data = sorted(zip(timesteps, rewards, episodes, dates))
                timesteps, rewards, episodes, dates = zip(*sorted_data)
                return {
                    'timesteps': list(timesteps),
                    'rewards': list(rewards),
                    'episodes': list(episodes),
                    'dates': list(dates)
                }
            return {'timesteps': [], 'rewards': [], 'episodes': [], 'dates': []}
        
        history = await loop.run_in_executor(None, parse_history_from_log, log_file)
        result = [{
            'step': history['timesteps'][i],
            'reward': history['rewards'][i] if i < len(history['rewards']) else 0,
            'timestamp': history['dates'][i] if i < len(history['dates']) and history['dates'][i] else None
        } for i in range(len(history['timesteps']))]
        return result
    except Exception as e:
        logger.error(f"Error in api_metrics_history: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/logs/sac_training")
@limiter.limit("10/minute")
async def api_sac_training_log(request: Request):
    """Get SAC training log"""
    try:
        LOG_DIR = BASE_DIR / "logs"
        sac_logs = list(LOG_DIR.glob("sac_training_*.log"))
        if not sac_logs:
            return {'content': '', 'filename': None}
        
        latest_log = max(sac_logs, key=lambda p: p.stat().st_mtime)
        with open(latest_log, 'r', encoding='utf-8', errors='ignore') as f:
            all_lines = f.readlines()
            lines = all_lines[-500:] if len(all_lines) > 500 else all_lines
            content = ''.join(lines)
        
        return {
            'content': content,
            'filename': latest_log.name,
            'size': latest_log.stat().st_size,
            'modified': datetime.fromtimestamp(latest_log.stat().st_mtime).isoformat()
        }
    except Exception as e:
        logger.error(f"Error reading training log: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/logs")
@limiter.limit("10/minute")
async def api_logs(request: Request):
    """Get system log (auto_manage.log)"""
    try:
        LOG_DIR = BASE_DIR / "logs"
        auto_manage_log = LOG_DIR / "auto_manage.log"
        
        if not auto_manage_log.exists():
            return {
                'content': 'Log file not found',
                'filename': None
            }
        
        with open(auto_manage_log, 'r', encoding='utf-8', errors='ignore') as f:
            all_lines = f.readlines()
            lines = all_lines[-500:] if len(all_lines) > 500 else all_lines
            content = ''.join(lines)
        
        return {
            'content': content,
            'filename': 'auto_manage.log',
            'size': auto_manage_log.stat().st_size,
            'modified': datetime.fromtimestamp(auto_manage_log.stat().st_mtime).isoformat()
        }
    except Exception as e:
        logger.error(f"Error reading system log: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/logs/auto_manage")
@limiter.limit("10/minute")
async def api_auto_manage_log(request: Request):
    """Get auto-manager log"""
    try:
        LOG_DIR = BASE_DIR / "logs"
        auto_manage_log = LOG_DIR / "auto_manage.log"
        
        if not auto_manage_log.exists():
            return {
                'success': False,
                'error': 'Log file not found',
                'lines': [],
                'count': 0
            }
        
        with open(auto_manage_log, 'r', encoding='utf-8', errors='ignore') as f:
            all_lines = f.readlines()
            lines = all_lines[-100:] if len(all_lines) > 100 else all_lines
        
        mtime = auto_manage_log.stat().st_mtime
        return {
            'success': True,
            'lines': [line.rstrip('\n') for line in lines],
            'count': len(lines),
            'last_update': datetime.fromtimestamp(mtime).isoformat(),
            'file_exists': True
        }
    except Exception as e:
        logger.error(f"Error reading auto-manage log: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

# ============================================================================
# Main Entry Point
# ============================================================================
if __name__ == '__main__':
    import uvicorn
    
    logger.info("=" * 70)
    logger.info("🚀 Starting Production FastAPI Dashboard")
    logger.info("=" * 70)
    
    uvicorn.run(
        "app_fastapi_production:app",
        host=os.getenv("HOST", "0.0.0.0"),
        port=int(os.getenv("PORT", "5001")),
        log_level=LOG_LEVEL.lower(),
        workers=int(os.getenv("WORKERS", "1")),
        loop="asyncio",
        access_log=True,
        reload=DEBUG
    )

