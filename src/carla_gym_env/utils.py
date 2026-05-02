import numpy as np
import carla
from typing import Tuple


def get_speed(velocity: carla.Vector3D) -> float:
    """Calculate speed from velocity vector in m/s."""
    return np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


def get_transform_matrix(transform: carla.Transform) -> np.ndarray:
    """Convert CARLA transform to 4x4 transformation matrix."""
    rotation = transform.rotation
    location = transform.location
    
    # Convert to radians
    pitch = np.radians(rotation.pitch)
    yaw = np.radians(rotation.yaw)
    roll = np.radians(rotation.roll)
    
    # Rotation matrices
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    # Combined rotation matrix (yaw * pitch * roll)
    matrix = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, location.x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, location.y],
        [-sp, cp * sr, cp * cr, location.z],
        [0, 0, 0, 1]
    ])
    
    return matrix


def world_to_sensor(point: np.ndarray, sensor_transform: carla.Transform) -> np.ndarray:
    """Transform world coordinates to sensor coordinates."""
    sensor_matrix = get_transform_matrix(sensor_transform)
    sensor_matrix_inv = np.linalg.inv(sensor_matrix)
    
    # Convert point to homogeneous coordinates
    point_homo = np.append(point, 1)
    
    # Transform
    point_sensor = sensor_matrix_inv @ point_homo
    
    return point_sensor[:3]


def calculate_distance(loc1: carla.Location, loc2: carla.Location) -> float:
    """Calculate Euclidean distance between two locations."""
    return np.sqrt(
        (loc1.x - loc2.x)**2 + 
        (loc1.y - loc2.y)**2 + 
        (loc1.z - loc2.z)**2
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def get_forward_vector(rotation: carla.Rotation) -> np.ndarray:
    """Get forward vector from rotation."""
    yaw = np.radians(rotation.yaw)
    return np.array([np.cos(yaw), np.sin(yaw), 0.0])


def get_right_vector(rotation: carla.Rotation) -> np.ndarray:
    """Get right vector from rotation."""
    yaw = np.radians(rotation.yaw)
    return np.array([np.sin(yaw), -np.cos(yaw), 0.0])


def project_point_to_line(point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Project a point onto a line segment.
    
    Returns:
        projected_point: The projected point on the line
        distance: Distance from point to projected point
    """
    line_vec = line_end - line_start
    point_vec = point - line_start
    
    line_len = np.linalg.norm(line_vec)
    if line_len < 1e-6:
        return line_start, np.linalg.norm(point_vec)
    
    line_unitvec = line_vec / line_len
    
    # Project point onto line
    projection_length = np.dot(point_vec, line_unitvec)
    projection_length = np.clip(projection_length, 0, line_len)
    
    projected_point = line_start + line_unitvec * projection_length
    distance = np.linalg.norm(point - projected_point)
    
    return projected_point, distance


def kmh_to_ms(speed_kmh: float) -> float:
    """Convert km/h to m/s."""
    return speed_kmh / 3.6


def ms_to_kmh(speed_ms: float) -> float:
    """Convert m/s to km/h."""
    return speed_ms * 3.6
