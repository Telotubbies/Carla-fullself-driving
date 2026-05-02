import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def test_sac_config_import():
    """Test SAC configuration import."""
    from src.sac_trainer.config import get_sac_config, get_training_config
    
    config = get_sac_config()
    assert config is not None
    
    training_config = get_training_config()
    assert training_config is not None
    assert 'num_iterations' in training_config


def test_callbacks_import():
    """Test callbacks import."""
    from src.sac_trainer.callbacks import CarlaCallbacks
    assert CarlaCallbacks is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
