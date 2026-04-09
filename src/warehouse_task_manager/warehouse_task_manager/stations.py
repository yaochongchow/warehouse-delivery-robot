import yaml
import os
from geometry_msgs.msg import PoseStamped
from math import sin, cos


def load_stations(yaml_path: str) -> dict:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    stations = {}
    for name, coords in data['waypoints'].items():
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(coords['x'])
        pose.pose.position.y = float(coords['y'])
        pose.pose.position.z = 0.0
        yaw = float(coords['yaw'])
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        stations[name] = pose

    return stations


# Station name aliases
STATION_ALIASES = {
    'A': 'pickup_A',
    'B': 'pickup_B',
    'D1': 'delivery_D1',
    'D2': 'delivery_D2',
    'charging': 'charging',
}


def resolve_station_name(name: str) -> str:
    return STATION_ALIASES.get(name, name)
