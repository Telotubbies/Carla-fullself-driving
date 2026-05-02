"""
MLflow Tracker
จัดการการ track experiments, parameters, metrics, และ artifacts ด้วย MLflow
"""

import mlflow
import mlflow.pytorch
from typing import Dict, Any, Optional
import os
from pathlib import Path


class MLflowTracker:
    """MLflow tracker สำหรับ CARLA training"""
    
    def __init__(
        self,
        experiment_name: str = "carla_sac_curriculum",
        tracking_uri: Optional[str] = None,
        run_name: Optional[str] = None
    ):
        """
        Args:
            experiment_name: ชื่อ experiment
            tracking_uri: URI สำหรับ MLflow tracking server (default: ./mlruns)
            run_name: ชื่อ run (optional)
        """
        self.experiment_name = experiment_name
        
        # ตั้งค่า tracking URI
        if tracking_uri is None:
            tracking_uri = "./mlruns"
        
        mlflow.set_tracking_uri(tracking_uri)
        
        # สร้างหรือดึง experiment
        try:
            self.experiment_id = mlflow.create_experiment(experiment_name)
        except:
            experiment = mlflow.get_experiment_by_name(experiment_name)
            self.experiment_id = experiment.experiment_id
        
        mlflow.set_experiment(experiment_name)
        
        self.run_name = run_name
        self.active_run = None
        
        print(f"✅ MLflow Tracker initialized")
        print(f"   Experiment: {experiment_name}")
        print(f"   Tracking URI: {tracking_uri}")
    
    def start_run(self, run_name: Optional[str] = None, tags: Optional[Dict[str, str]] = None):
        """เริ่ม MLflow run"""
        if self.active_run is not None:
            print("⚠️ Active run already exists, ending it first")
            self.end_run()
        
        if run_name is None:
            run_name = self.run_name
        
        self.active_run = mlflow.start_run(run_name=run_name)
        
        # เพิ่ม tags
        if tags:
            mlflow.set_tags(tags)
        
        print(f"🚀 MLflow run started: {run_name or 'unnamed'}")
        return self.active_run
    
    def end_run(self):
        """จบ MLflow run"""
        if self.active_run is not None:
            mlflow.end_run()
            print(f"✅ MLflow run ended")
            self.active_run = None
    
    def log_params(self, params: Dict[str, Any]):
        """Log parameters"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_params")
            return
        
        mlflow.log_params(params)
    
    def log_param(self, key: str, value: Any):
        """Log single parameter"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_param")
            return
        
        mlflow.log_param(key, value)
    
    def log_metrics(self, metrics: Dict[str, float], step: Optional[int] = None):
        """Log metrics"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_metrics")
            return
        
        mlflow.log_metrics(metrics, step=step)
    
    def log_metric(self, key: str, value: float, step: Optional[int] = None):
        """Log single metric"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_metric")
            return
        
        mlflow.log_metric(key, value, step=step)
    
    def log_artifact(self, local_path: str, artifact_path: Optional[str] = None):
        """Log artifact (file)"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_artifact")
            return
        
        mlflow.log_artifact(local_path, artifact_path)
    
    def log_artifacts(self, local_dir: str, artifact_path: Optional[str] = None):
        """Log artifacts (directory)"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_artifacts")
            return
        
        mlflow.log_artifacts(local_dir, artifact_path)
    
    def log_model(self, model, artifact_path: str = "model"):
        """Log PyTorch model"""
        if self.active_run is None:
            print("⚠️ No active run, skipping log_model")
            return
        
        mlflow.pytorch.log_model(model, artifact_path)
    
    def set_tag(self, key: str, value: str):
        """Set tag"""
        if self.active_run is None:
            print("⚠️ No active run, skipping set_tag")
            return
        
        mlflow.set_tag(key, value)
    
    def set_tags(self, tags: Dict[str, str]):
        """Set multiple tags"""
        if self.active_run is None:
            print("⚠️ No active run, skipping set_tags")
            return
        
        mlflow.set_tags(tags)
    
    def log_curriculum_stage(self, stage_name: str, stage_idx: int, progress: float):
        """Log curriculum stage information"""
        self.log_metric("curriculum/stage_idx", stage_idx)
        self.log_metric("curriculum/progress", progress)
        self.set_tag("curriculum_stage", stage_name)
    
    def log_episode_metrics(
        self,
        episode: int,
        reward: float,
        length: int,
        collision: bool,
        success: bool,
        **kwargs
    ):
        """Log episode-level metrics"""
        metrics = {
            "episode/reward": reward,
            "episode/length": length,
            "episode/collision": 1.0 if collision else 0.0,
            "episode/success": 1.0 if success else 0.0,
        }
        
        # เพิ่ม metrics อื่นๆ
        for key, value in kwargs.items():
            metrics[f"episode/{key}"] = value
        
        self.log_metrics(metrics, step=episode)
    
    def log_training_metrics(
        self,
        step: int,
        loss: Optional[float] = None,
        actor_loss: Optional[float] = None,
        critic_loss: Optional[float] = None,
        alpha_loss: Optional[float] = None,
        **kwargs
    ):
        """Log training-level metrics"""
        metrics = {}
        
        if loss is not None:
            metrics["training/loss"] = loss
        if actor_loss is not None:
            metrics["training/actor_loss"] = actor_loss
        if critic_loss is not None:
            metrics["training/critic_loss"] = critic_loss
        if alpha_loss is not None:
            metrics["training/alpha_loss"] = alpha_loss
        
        # เพิ่ม metrics อื่นๆ
        for key, value in kwargs.items():
            metrics[f"training/{key}"] = value
        
        if metrics:
            self.log_metrics(metrics, step=step)
    
    def get_run_id(self) -> Optional[str]:
        """ดึง run ID ปัจจุบัน"""
        if self.active_run is None:
            return None
        return self.active_run.info.run_id
    
    def get_artifact_uri(self) -> Optional[str]:
        """ดึง artifact URI"""
        if self.active_run is None:
            return None
        return mlflow.get_artifact_uri()
    
    @staticmethod
    def get_mlflow_ui_url(port: int = 5000) -> str:
        """ดึง URL สำหรับ MLflow UI"""
        return f"http://localhost:{port}"
    
    @staticmethod
    def start_mlflow_ui(tracking_uri: str = "./mlruns", port: int = 5000):
        """เริ่ม MLflow UI server (ต้องรันใน subprocess แยก)"""
        import subprocess
        
        cmd = f"mlflow ui --backend-store-uri {tracking_uri} --port {port}"
        print(f"🌐 Starting MLflow UI at {MLflowTracker.get_mlflow_ui_url(port)}")
        print(f"   Command: {cmd}")
        print(f"   Run this command in a separate terminal:")
        print(f"   {cmd}")
        
        return cmd
