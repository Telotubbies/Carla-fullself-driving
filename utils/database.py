"""
PostgreSQL database integration for storing training data.

Optional module - only used if database is enabled in config.
"""

import logging
from typing import Optional, Dict, Any, List
from datetime import datetime
import json

logger = logging.getLogger(__name__)

# Try to import database libraries
try:
    import psycopg2
    from psycopg2.extras import execute_values
    from sqlalchemy import create_engine, Column, Integer, Float, String, DateTime, Text, JSON
    try:
        from sqlalchemy.orm import declarative_base
    except ImportError:
        from sqlalchemy.ext.declarative import declarative_base
    from sqlalchemy.orm import sessionmaker
    DB_AVAILABLE = True
    Base = declarative_base()
except ImportError:
    DB_AVAILABLE = False
    Base = None
    logger.warning("PostgreSQL libraries not available. Install: pip install psycopg2-binary SQLAlchemy")


# Only define TrainingData if database is available
if DB_AVAILABLE and Base is not None:
    class TrainingData(Base):
        """Database model for training data."""
        __tablename__ = 'training_data'
        
        id = Column(Integer, primary_key=True, autoincrement=True)
        timestamp = Column(DateTime, default=datetime.utcnow)
        step = Column(Integer)
        image_path = Column(String(500))
        x = Column(Float)
        y = Column(Float)
        yaw = Column(Float)
        velocity = Column(Float)
        steering = Column(Float)
        throttle = Column(Float)
        brake = Column(Float)
        features = Column(JSON)  # Store feature vector as JSON
        metadata = Column(JSON)  # Additional metadata
else:
    # Dummy class if database not available
    class TrainingData:
        pass


class DatabaseManager:
    """Manages PostgreSQL database connection and operations."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize database manager.
        
        Args:
            config: Database configuration dictionary
        """
        self.config = config
        self.enabled = config.get('enabled', False)
        self.engine = None
        self.Session = None
        
        if not self.enabled:
            logger.info("Database disabled in config")
            return
        
        if not DB_AVAILABLE:
            logger.error("Database libraries not available. Disabling database.")
            self.enabled = False
            return
        
        try:
            self._connect()
            self._create_tables()
            logger.info("✅ Database connection established")
        except Exception as e:
            logger.error(f"❌ Failed to connect to database: {e}")
            logger.warning("Continuing without database...")
            self.enabled = False
    
    def _connect(self) -> None:
        """Connect to PostgreSQL database."""
        db_config = self.config.get('postgresql', {})
        host = db_config.get('host', 'localhost')
        port = db_config.get('port', 5432)
        database = db_config.get('database', 'carla_training')
        user = db_config.get('user', 'postgres')
        password = db_config.get('password', 'postgres')
        
        connection_string = f"postgresql://{user}:{password}@{host}:{port}/{database}"
        self.engine = create_engine(connection_string, pool_pre_ping=True)
        self.Session = sessionmaker(bind=self.engine)
    
    def _create_tables(self) -> None:
        """Create database tables if they don't exist."""
        if Base is not None:
            Base.metadata.create_all(self.engine)
            logger.info("✅ Database tables created/verified")
    
    def save_data(
        self,
        step: int,
        image_path: Optional[str],
        vehicle_state: Dict[str, float],
        control: tuple,
        features: Optional[list] = None,
        metadata: Optional[Dict] = None
    ) -> bool:
        """
        Save training data to database.
        
        Args:
            step: Step number
            image_path: Path to image file
            vehicle_state: Vehicle state dictionary
            control: Control tuple (steering, throttle, brake)
            features: Optional feature vector
            metadata: Optional metadata dictionary
            
        Returns:
            True if successful
        """
        if not self.enabled:
            return False
        
        try:
            session = self.Session()
            data = TrainingData(
                step=step,
                image_path=image_path,
                x=vehicle_state.get('x', 0.0),
                y=vehicle_state.get('y', 0.0),
                yaw=vehicle_state.get('yaw', 0.0),
                velocity=vehicle_state.get('velocity', 0.0),
                steering=control[0],
                throttle=control[1],
                brake=control[2],
                features=features,
                metadata=metadata
            )
            session.add(data)
            session.commit()
            session.close()
            return True
        except Exception as e:
            logger.error(f"Error saving to database: {e}")
            return False
    
    def batch_save(self, data_list: List[Dict[str, Any]]) -> bool:
        """
        Batch save multiple records.
        
        Args:
            data_list: List of data dictionaries
            
        Returns:
            True if successful
        """
        if not self.enabled:
            return False
        
        try:
            session = self.Session()
            records = []
            for data in data_list:
                record = TrainingData(
                    step=data.get('step', 0),
                    image_path=data.get('image_path'),
                    x=data.get('x', 0.0),
                    y=data.get('y', 0.0),
                    yaw=data.get('yaw', 0.0),
                    velocity=data.get('velocity', 0.0),
                    steering=data.get('steering', 0.0),
                    throttle=data.get('throttle', 0.0),
                    brake=data.get('brake', 0.0),
                    features=data.get('features'),
                    metadata=data.get('metadata')
                )
                records.append(record)
            
            session.bulk_save_objects(records)
            session.commit()
            session.close()
            return True
        except Exception as e:
            logger.error(f"Error batch saving to database: {e}")
            return False
    
    def close(self) -> None:
        """Close database connection."""
        if self.engine is not None:
            self.engine.dispose()
            logger.info("✅ Database connection closed")
