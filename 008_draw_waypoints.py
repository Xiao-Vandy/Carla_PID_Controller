import os
import carla
import sys
import pygame
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import math
import xml.etree.ElementTree as ET
carla_root = '/home-local/lix88/Downloads/Hybrid_project/carla/XiaoProj/carla'
sys.path.append(carla_root)
from agents.navigation.global_route_planner import GlobalRoutePlanner #type: ignore
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO #type: ignore
from vehicle_controller import VehiclePIDController


def plot_path(waypoints, ids, title):
    x = [waypoint.location.x for waypoint in waypoints]
    y = [waypoint.location.y for waypoint in waypoints]
    
    plt.figure(figsize=(10, 10))
    plt.scatter(x, y, c=ids, cmap='viridis')
    plt.scatter(x[0], y[0], c='red', marker='*', s=100)
    plt.scatter(x[-1], y[-1], c='green', marker='*', s=100)
    
    # Connect waypoints with lines
    plt.plot(x, y, color='blue', linewidth=1)

    for x, y, id in zip(x, y, ids):
        plt.text(x, y, str(id), fontsize=12, ha='center', va='bottom')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(f'Path for {title}')
    plt.savefig(f'{title}_path.png')
    print(f'{title}_path.png saved')
    plt.show()


def get_required_waypoints(Town_name):
    waypoints_file_mapping = {
        "Town01_required_waypoints.txt": "Town01_waypoints_xml_format.txt",
        "Town02_required_waypoints.txt": "Town02_waypoints_xml_format.txt",
        "Town07_required_waypoints.txt": "Town07_waypoints_xml_format.txt",
    }
    id_file_name = f"{Town_name}_required_waypoints.txt"
    xml_file_name = waypoints_file_mapping.get(id_file_name)
    if not xml_file_name:
        print(f"Waypoints file for {Town_name} not found")
        return None, None
    return id_file_name, xml_file_name  

def load_waypoints_from_xml(id_file_name, xml_file_name):
    id_file_path = f"./Town_required_waypoints/{id_file_name}"
    if not os.path.exists(id_file_path):
        raise FileNotFoundError(f"ID file not found: {id_file_path}")
    
    with open(id_file_path, 'r') as f:
        ids = eval(f.read())

    xml_file_path = f"./Town_required_waypoints/{xml_file_name.replace('required', 'xml_format')}"
    if not os.path.exists(xml_file_path):
        raise FileNotFoundError(f"XML file not found: {xml_file_path}")
    
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    waypoints = []
    
    for i, waypoint in enumerate(root.findall('waypoint')):
        x = float(waypoint.get('x'))
        y = float(waypoint.get('y'))
        z = float(waypoint.get('z'))
        yaw = float(waypoint.get('yaw'))
        id = ids[i] if i < len(ids) else None
        waypoint_loc = carla.Location(x=x, y=y, z=z)
        waypoint_rot = carla.Rotation(yaw=yaw)
        
        # Create a Transform object
        waypoint_transform = carla.Transform(location=waypoint_loc, rotation=waypoint_rot)
        waypoints.append((waypoint_transform, id))
        
    return waypoints

class RoutePlanner:
    def __init__(self,map,waypoints_list):
        self.map = map
        self.waypoints_list = waypoints_list
        self.safe_waypoints = []
        self.dao = GlobalRoutePlannerDAO(map, sampling_resolution = 2.0)
        self.global_route_planner = GlobalRoutePlanner(self.dao)
        self.global_route_planner.setup()

    def get_safe_waypoints(self):
        start_location = self.waypoints_list[0]
        for i in range(1, len(self.waypoints_list)):
            end_location = self.waypoints_list[i]

            start_waypoint = self.map.get_waypoint(start_location.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            end_waypoint = self.map.get_waypoint(end_location.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            route = self.global_route_planner.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)

            start_location = end_location
            for way_point, _ in route:
                self.safe_waypoints.append(way_point.transform)

def main():
    map_name = 'Town07'
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    world = client.load_world(map_name)
    weather = carla.WeatherParameters(
        cloudiness=20.0,
        sun_altitude_angle=75.0
    )
    world.set_weather(weather)
    blueprint_library = world.get_blueprint_library()
    map = world.get_map()
    id_file_name, waypoints_file_name = get_required_waypoints(map_name)
    
    if id_file_name is None or waypoints_file_name is None:
        return
    
    waypoints = load_waypoints_from_xml(id_file_name, waypoints_file_name)
    if not waypoints:
        raise ValueError("Waypoints list is empty. Check the waypoints file.")
    print(f"len(waypoints): {len(waypoints)}")
    print()
    plot_path(waypoints=[wp[0] for wp in waypoints], ids=[wp[1] for wp in waypoints], title=map_name)


if __name__ == '__main__':
    main()
