import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def test_ros2_imports():
    """Test ROS2 module imports."""
    try:
        import rclpy
        from sensor_msgs.msg import PointCloud2, Image
        from geometry_msgs.msg import TwistStamped
        assert True
    except ImportError as e:
        pytest.skip(f"ROS2 not installed: {e}")


def test_sensor_publisher_import():
    """Test sensor publisher import."""
    try:
        from src.ros2_bridge.sensor_publisher import SensorPublisher
        assert SensorPublisher is not None
    except ImportError as e:
        pytest.skip(f"ROS2 dependencies not available: {e}")


def test_control_subscriber_import():
    """Test control subscriber import."""
    try:
        from src.ros2_bridge.control_subscriber import ControlSubscriber
        assert ControlSubscriber is not None
    except ImportError as e:
        pytest.skip(f"ROS2 dependencies not available: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
