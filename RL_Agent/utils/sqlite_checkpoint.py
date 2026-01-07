"""
SQLite Checkpoint Manager for RL Training
Provides persistent checkpoint storage with metadata tracking
"""

import sqlite3
import pickle
import gzip
import json
import os
from pathlib import Path
from typing import Dict, Optional, List, Tuple, Any
from datetime import datetime
import logging
import numpy as np
import torch


class SQLiteCheckpointManager:
    """
    Manages checkpoints in SQLite database
    Provides atomic writes, query capabilities, and resume functionality
    """
    
    def __init__(self, db_path: str, enable_wal: bool = True):
        """
        Initialize SQLite checkpoint manager
        
        Args:
            db_path: Path to SQLite database file
            enable_wal: Enable Write-Ahead Logging for better performance
        """
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Connect to database
        self.conn = sqlite3.connect(str(self.db_path), timeout=30.0)
        self.conn.row_factory = sqlite3.Row
        
        # Enable WAL mode for better performance
        if enable_wal:
            self.conn.execute('PRAGMA journal_mode=WAL')
            self.conn.execute('PRAGMA synchronous=NORMAL')  # Faster than FULL
            self.conn.execute('PRAGMA cache_size=-64000')  # 64MB cache
            self.conn.execute('PRAGMA temp_store=MEMORY')
        
        # Create tables
        self._create_tables()
        
        logging.info(f"✅ SQLite checkpoint manager initialized: {self.db_path}")
    
    def clear_database(self):
        """Clear all data from database (keep structure)"""
        try:
            cursor = self.conn.cursor()
            cursor.execute('DELETE FROM checkpoints')
            cursor.execute('DELETE FROM training_stats')
            cursor.execute('DELETE FROM episodes')
            self.conn.commit()
            logging.info("✅ Database cleared (all tables emptied)")
        except Exception as e:
            logging.error(f"Failed to clear database: {e}", exc_info=True)
            raise
    
    def reset_database(self):
        """Reset database: drop all tables and recreate"""
        try:
            cursor = self.conn.cursor()
            cursor.execute('DROP TABLE IF EXISTS checkpoints')
            cursor.execute('DROP TABLE IF EXISTS training_stats')
            cursor.execute('DROP TABLE IF EXISTS episodes')
            self.conn.commit()
            self._create_tables()
            logging.info("✅ Database reset (tables recreated)")
        except Exception as e:
            logging.error(f"Failed to reset database: {e}", exc_info=True)
            raise
    
    def _create_tables(self):
        """Create database tables if they don't exist"""
        cursor = self.conn.cursor()
        
        # Checkpoints table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS checkpoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestep INTEGER NOT NULL,
                episode INTEGER,
                reward REAL,
                model_data BLOB NOT NULL,
                optimizer_data BLOB,
                training_state BLOB,
                metadata TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                UNIQUE(timestep)
            )
        ''')
        
        # Training stats table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS training_stats (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestep INTEGER NOT NULL,
                iteration INTEGER,
                mean_reward REAL,
                mean_episode_length REAL,
                learning_rate REAL,
                value_loss REAL,
                policy_loss REAL,
                entropy_loss REAL,
                fps REAL,
                metadata TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # Episodes table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS episodes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                episode_num INTEGER NOT NULL,
                timestep_start INTEGER,
                timestep_end INTEGER,
                reward REAL,
                length INTEGER,
                collision BOOLEAN,
                goal_reached BOOLEAN,
                distance_to_goal REAL,
                metadata TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # Create indexes for faster queries
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_checkpoints_timestep ON checkpoints(timestep)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_stats_timestep ON training_stats(timestep)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_episodes_num ON episodes(episode_num)')
        
        self.conn.commit()
    
    def _get_connection(self):
        """
        Get a database connection (thread-safe)
        Creates a new connection if called from a different thread
        """
        import threading
        current_thread_id = threading.get_ident()
        
        # Check if we're in the main thread (where self.conn was created)
        if not hasattr(self, '_main_thread_id'):
            self._main_thread_id = threading.get_ident()
        
        # If in main thread, use existing connection
        if current_thread_id == self._main_thread_id:
            return self.conn
        
        # Otherwise, create a new connection for this thread
        if not hasattr(self, '_thread_connections'):
            self._thread_connections = {}
        
        if current_thread_id not in self._thread_connections:
            conn = sqlite3.connect(str(self.db_path), timeout=30.0)
            conn.row_factory = sqlite3.Row
            # Enable WAL mode for better performance
            conn.execute('PRAGMA journal_mode=WAL')
            conn.execute('PRAGMA synchronous=NORMAL')
            conn.execute('PRAGMA cache_size=-64000')
            conn.execute('PRAGMA temp_store=MEMORY')
            self._thread_connections[current_thread_id] = conn
            logging.debug(f"Created new SQLite connection for thread {current_thread_id}")
        
        return self._thread_connections[current_thread_id]
    
    def save_checkpoint(
        self,
        model: Any,
        timestep: int,
        episode: Optional[int] = None,
        reward: Optional[float] = None,
        metadata: Optional[Dict] = None
    ) -> int:
        """
        Save checkpoint to SQLite database (thread-safe)
        
        Args:
            model: PPO model to save
            timestep: Current training timestep
            episode: Current episode number
            reward: Current reward
            metadata: Additional metadata dict
        
        Returns:
            Checkpoint ID
        """
        conn = None
        try:
            # Get thread-safe connection
            conn = self._get_connection()
            
            # Serialize model data
            model_buffer = self._serialize_model(model)
            
            # Serialize optimizer state if available
            optimizer_buffer = None
            if hasattr(model, 'policy') and hasattr(model.policy, 'optimizer'):
                optimizer_buffer = self._serialize_optimizer(model.policy.optimizer)
            
            # Serialize training state
            training_state = {
                'n_steps': getattr(model, 'n_steps', 0),
                'num_timesteps': getattr(model, 'num_timesteps', timestep),
                'learning_rate': getattr(model, 'lr_schedule', lambda _: 0.0)(1.0) if hasattr(model, 'lr_schedule') else None
            }
            training_state_buffer = gzip.compress(pickle.dumps(training_state))
            
            # Serialize metadata
            metadata_json = json.dumps(metadata) if metadata else None
            
            # Insert into database
            cursor = conn.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO checkpoints 
                (timestep, episode, reward, model_data, optimizer_data, training_state, metadata)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (
                timestep,
                episode,
                reward,
                model_buffer,
                optimizer_buffer,
                training_state_buffer,
                metadata_json
            ))
            
            checkpoint_id = cursor.lastrowid
            conn.commit()
            
            logging.info(f"💾 Checkpoint saved to SQLite: timestep={timestep}, id={checkpoint_id}")
            return checkpoint_id
            
        except Exception as e:
            logging.error(f"Failed to save checkpoint: {e}", exc_info=True)
            if conn:
                try:
                    conn.rollback()
                except:
                    pass
            raise
    
    def load_checkpoint(self, timestep: Optional[int] = None) -> Optional[Dict]:
        """
        Load checkpoint from SQLite database
        
        Args:
            timestep: Specific timestep to load (None = latest/highest timestep)
        
        Returns:
            Dict with model, optimizer, training_state, metadata or None
        
        ⚠️ IMPORTANT: For RL training, we use LAST checkpoint (highest timestep),
        NOT best checkpoint by reward. This is because RL algorithms continuously
        improve and the latest checkpoint contains the most recent learning progress.
        """
        try:
            cursor = self.conn.cursor()
            
            if timestep is not None:
                cursor.execute('''
                    SELECT * FROM checkpoints 
                    WHERE timestep = ?
                    ORDER BY id DESC
                    LIMIT 1
                ''', (timestep,))
            else:
                cursor.execute('''
                    SELECT * FROM checkpoints 
                    ORDER BY timestep DESC
                    LIMIT 1
                ''')
            
            row = cursor.fetchone()
            if row is None:
                logging.warning("No checkpoint found in database")
                return None
            
            # Deserialize data
            model = self._deserialize_model(row['model_data'])
            optimizer = None
            if row['optimizer_data']:
                optimizer = self._deserialize_optimizer(row['optimizer_data'])
            
            training_state = None
            if row['training_state']:
                training_state = pickle.loads(gzip.decompress(row['training_state']))
            
            metadata = None
            if row['metadata']:
                metadata = json.loads(row['metadata'])
            
            result = {
                'model': model,
                'optimizer': optimizer,
                'training_state': training_state,
                'timestep': row['timestep'],
                'episode': row['episode'],
                'reward': row['reward'],
                'metadata': metadata,
                'checkpoint_id': row['id']
            }
            
            logging.info(f"📂 Checkpoint loaded from SQLite: timestep={row['timestep']}, id={row['id']}")
            return result
            
        except Exception as e:
            logging.error(f"Failed to load checkpoint: {e}", exc_info=True)
            return None
    
    def save_training_stats(
        self,
        timestep: int,
        iteration: Optional[int] = None,
        mean_reward: Optional[float] = None,
        mean_episode_length: Optional[float] = None,
        learning_rate: Optional[float] = None,
        value_loss: Optional[float] = None,
        policy_loss: Optional[float] = None,
        entropy_loss: Optional[float] = None,
        fps: Optional[float] = None,
        metadata: Optional[Dict] = None
    ):
        """Save training statistics (thread-safe)"""
        conn = None
        try:
            # Get thread-safe connection
            conn = self._get_connection()
            
            metadata_json = json.dumps(metadata) if metadata else None
            
            cursor = conn.cursor()
            cursor.execute('''
                INSERT INTO training_stats 
                (timestep, iteration, mean_reward, mean_episode_length, learning_rate,
                 value_loss, policy_loss, entropy_loss, fps, metadata)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                timestep, iteration, mean_reward, mean_episode_length, learning_rate,
                value_loss, policy_loss, entropy_loss, fps, metadata_json
            ))
            conn.commit()
        except Exception as e:
            logging.error(f"Failed to save training stats: {e}", exc_info=True)
            if conn:
                try:
                    conn.rollback()
                except:
                    pass
    
    def save_episode(
        self,
        episode_num: int,
        timestep_start: Optional[int] = None,
        timestep_end: Optional[int] = None,
        reward: Optional[float] = None,
        length: Optional[int] = None,
        collision: Optional[bool] = None,
        goal_reached: Optional[bool] = None,
        distance_to_goal: Optional[float] = None,
        metadata: Optional[Dict] = None
    ):
        """Save episode statistics"""
        try:
            metadata_json = json.dumps(metadata) if metadata else None
            
            cursor = self.conn.cursor()
            cursor.execute('''
                INSERT INTO episodes 
                (episode_num, timestep_start, timestep_end, reward, length,
                 collision, goal_reached, distance_to_goal, metadata)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                episode_num, timestep_start, timestep_end, reward, length,
                collision, goal_reached, distance_to_goal, metadata_json
            ))
            self.conn.commit()
        except Exception as e:
            logging.error(f"Failed to save episode: {e}", exc_info=True)
            self.conn.rollback()
    
    def get_latest_checkpoint_info(self) -> Optional[Dict]:
        """
        Get information about latest checkpoint (highest timestep).
        
        ⚠️ IMPORTANT: For RL training, we use LAST checkpoint (highest timestep),
        NOT best checkpoint by reward. This is because RL algorithms continuously
        improve and the latest checkpoint contains the most recent learning progress.
        """
        try:
            cursor = self.conn.cursor()
            cursor.execute('''
                SELECT timestep, episode, reward, created_at
                FROM checkpoints
                ORDER BY timestep DESC
                LIMIT 1
            ''')
            row = cursor.fetchone()
            if row:
                return dict(row)
            return None
        except Exception as e:
            logging.error(f"Failed to get latest checkpoint info: {e}")
            return None
    
    def get_best_checkpoint_info(self, min_reward: float = 0.0, limit: int = 1) -> Optional[List[Dict]]:
        """
        Get best checkpoint(s) by reward (for use as base model).
        
        Args:
            min_reward: Minimum reward threshold (default: 0.0 for positive rewards)
            limit: Number of best checkpoints to return (default: 1)
        
        Returns:
            List of checkpoint info dicts, sorted by reward DESC
        """
        try:
            cursor = self.conn.cursor()
            cursor.execute('''
                SELECT timestep, episode, reward, created_at
                FROM checkpoints
                WHERE reward IS NOT NULL AND reward >= ?
                ORDER BY reward DESC, timestep DESC
                LIMIT ?
            ''', (min_reward, limit))
            rows = cursor.fetchall()
            if rows:
                return [dict(row) for row in rows]
            return None
        except Exception as e:
            logging.error(f"Failed to get best checkpoint info: {e}")
            return None
    
    def get_training_history(
        self,
        limit: int = 100,
        start_timestep: Optional[int] = None
    ) -> List[Dict]:
        """Get training statistics history"""
        try:
            cursor = self.conn.cursor()
            if start_timestep:
                cursor.execute('''
                    SELECT * FROM training_stats
                    WHERE timestep >= ?
                    ORDER BY timestep DESC
                    LIMIT ?
                ''', (start_timestep, limit))
            else:
                cursor.execute('''
                    SELECT * FROM training_stats
                    ORDER BY timestep DESC
                    LIMIT ?
                ''', (limit,))
            
            return [dict(row) for row in cursor.fetchall()]
        except Exception as e:
            logging.error(f"Failed to get training history: {e}")
            return []
    
    def _serialize_model(self, model: Any) -> bytes:
        """Serialize model to compressed bytes"""
        # Save model to temporary file, then read and compress
        import tempfile
        import shutil
        
        with tempfile.NamedTemporaryFile(delete=False, suffix='.zip') as tmp_file:
            tmp_path = tmp_file.name
        
        try:
            # Save model to temp file
            model.save(tmp_path)
            
            # Read and compress
            with open(tmp_path, 'rb') as f:
                data = f.read()
            compressed = gzip.compress(data)
            
            return compressed
        finally:
            # Clean up temp file
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
    
    def _deserialize_model(self, data: bytes) -> bytes:
        """Deserialize model from compressed bytes"""
        # Decompress and return bytes (for external loading with PPO.load())
        decompressed = gzip.decompress(data)
        return decompressed
    
    def _serialize_optimizer(self, optimizer: Any) -> bytes:
        """Serialize optimizer state"""
        state_dict = optimizer.state_dict()
        return gzip.compress(pickle.dumps(state_dict))
    
    def _deserialize_optimizer(self, data: bytes) -> Any:
        """Deserialize optimizer state"""
        return pickle.loads(gzip.decompress(data))
    
    def close(self):
        """Close database connection (all thread connections)"""
        # Close main connection
        if self.conn:
            self.conn.close()
        
        # Close all thread connections
        if hasattr(self, '_thread_connections'):
            for thread_id, conn in self._thread_connections.items():
                try:
                    conn.close()
                except:
                    pass
            self._thread_connections.clear()
        
        logging.info("SQLite checkpoint manager closed")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

