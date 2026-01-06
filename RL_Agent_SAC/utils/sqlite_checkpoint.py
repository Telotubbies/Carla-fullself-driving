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
    
    def __init__(self, db_path: str, enable_wal: bool = True):
        
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        import threading
        self._main_thread_id = threading.get_ident()
        self._thread_connections = {}
        self._connection_lock = threading.Lock()
        self.conn = sqlite3.connect(str(self.db_path), timeout=30.0, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        if enable_wal:
            self.conn.execute('PRAGMA journal_mode=WAL')
            self.conn.execute('PRAGMA synchronous=NORMAL')
            self.conn.execute('PRAGMA cache_size=-64000')
            self.conn.execute('PRAGMA temp_store=MEMORY')
        self._create_tables()
        logging.info(f"✅ SQLite checkpoint manager initialized: {self.db_path} (main thread: {self._main_thread_id})")
    def clear_database(self):
        
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
        cursor = self.conn.cursor()
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
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
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
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_checkpoints_timestep ON checkpoints(timestep)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_stats_timestep ON training_stats(timestep)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_episodes_num ON episodes(episode_num)')
        self.conn.commit()
    def _get_connection(self):
        
        import threading
        current_thread_id = threading.get_ident()
        if current_thread_id == self._main_thread_id:
            return self.conn
        with self._connection_lock:
            if current_thread_id not in self._thread_connections:
                conn = sqlite3.connect(str(self.db_path), timeout=30.0, check_same_thread=False)
                conn.row_factory = sqlite3.Row
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
        
        import time
        start_time = time.time()
        logging.info(f"💾 Starting checkpoint save for timestep {timestep}...")
        conn = None
        try:
            logging.info(f"💾 Getting database connection...")
            conn = self._get_connection()
            logging.info(f"💾 Serializing model (timestep {timestep})...")
            model_buffer = self._serialize_model(model)
            logging.info(f"💾 Model serialized: {len(model_buffer) / (1024*1024):.2f} MB")
            optimizer_buffer = None
            if hasattr(model, 'policy') and hasattr(model.policy, 'optimizer'):
                logging.info(f"💾 Serializing optimizer...")
                optimizer_buffer = self._serialize_optimizer(model.policy.optimizer)
                logging.info(f"💾 Optimizer serialized: {len(optimizer_buffer) / (1024*1024):.2f} MB")
            logging.info(f"💾 Creating training state...")
            training_state = {
                'n_steps': getattr(model, 'n_steps', 0),
                'num_timesteps': getattr(model, 'num_timesteps', timestep),
                'learning_rate': getattr(model, 'lr_schedule', lambda _: 0.0)(1.0) if hasattr(model, 'lr_schedule') else None
            }
            training_state_buffer = gzip.compress(pickle.dumps(training_state))
            metadata_json = json.dumps(metadata) if metadata else None
            logging.info(f"💾 Writing to database...")
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
            logging.info(f"💾 Committing transaction...")
            conn.commit()
            elapsed = time.time() - start_time
            logging.info(f"💾 Checkpoint saved to SQLite: timestep={timestep}, id={checkpoint_id}, elapsed={elapsed:.2f}s")
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
        
        try:
            cursor = self.conn.cursor()
            if timestep is not None:
                cursor.execute('SELECT * FROM checkpoints WHERE timestep = ? ORDER BY id DESC LIMIT 1', (timestep,))
            else:
                cursor.execute('SELECT * FROM checkpoints ORDER BY timestep DESC LIMIT 1')
            row = cursor.fetchone()
            if row is None:
                logging.warning("No checkpoint found in database")
                return None
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
        
        conn = None
        try:
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
    def extract_checkpoint_to_temp(self, checkpoint_id: int) -> str:
        
        import tempfile
        try:
            cursor = self.conn.cursor()
            cursor.execute('SELECT model_data FROM checkpoints WHERE id = ?', (checkpoint_id,))
            row = cursor.fetchone()
            if not row or not row['model_data']:
                raise ValueError(f"Checkpoint {checkpoint_id} not found or has no model data")
            model_data = row['model_data']
            temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.zip')
            temp_file.write(model_data)
            temp_file.close()
            logging.info(f"✅ Extracted checkpoint {checkpoint_id} to temp file: {temp_file.name}")
            return temp_file.name
        except Exception as e:
            logging.error(f"Failed to extract checkpoint {checkpoint_id}: {e}", exc_info=True)
            raise
    def get_latest_checkpoint_info(self) -> Optional[Dict]:
        
        try:
            cursor = self.conn.cursor()
            cursor.execute('SELECT * FROM checkpoints ORDER BY timestep DESC LIMIT 1')
            row = cursor.fetchone()
            if row:
                return dict(row)
            return None
        except Exception as e:
            logging.error(f"Failed to get latest checkpoint info: {e}")
            return None
    def get_best_checkpoint_info(self, min_reward: float = 0.0, limit: int = 1) -> Optional[List[Dict]]:
        
        try:
            cursor = self.conn.cursor()
            cursor.execute('SELECT * FROM checkpoints WHERE reward >= ? ORDER BY reward DESC LIMIT ?', (min_reward, limit))
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
        
        try:
            cursor = self.conn.cursor()
            if start_timestep:
                cursor.execute('SELECT * FROM training_stats WHERE timestep >= ? ORDER BY timestep ASC LIMIT ?', (start_timestep, limit))
            else:
                cursor.execute('SELECT * FROM training_stats ORDER BY timestep DESC LIMIT ?', (limit,))
            return [dict(row) for row in cursor.fetchall()]
        except Exception as e:
            logging.error(f"Failed to get training history: {e}")
            return []
    def _serialize_model(self, model: Any) -> bytes:
        
        import tempfile
        import shutil
        logging.info(f"💾 Starting model serialization...")
        with tempfile.NamedTemporaryFile(delete=False, suffix='.zip') as tmp_file:
            tmp_path = tmp_file.name
        try:
            logging.info(f"💾 Saving model to temp file: {tmp_path}")
            model.save(tmp_path)
            logging.info(f"💾 Model saved, reading file...")
            with open(tmp_path, 'rb') as f:
                data = f.read()
            logging.info(f"💾 Model data size: {len(data) / (1024*1024):.2f} MB")
            logging.info(f"💾 Compressing model data...")
            compressed = gzip.compress(data)
            logging.info(f"💾 Compressed size: {len(compressed) / (1024*1024):.2f} MB")
            return compressed
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
    def _deserialize_model(self, data: bytes) -> bytes:
        
        decompressed = gzip.decompress(data)
        return decompressed
    def _serialize_optimizer(self, optimizer: Any) -> bytes:
        
        state_dict = optimizer.state_dict()
        return gzip.compress(pickle.dumps(state_dict))
    def _deserialize_optimizer(self, data: bytes) -> Any:
        
        return pickle.loads(gzip.decompress(data))
    def close(self):
        
        if self.conn:
            self.conn.close()
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