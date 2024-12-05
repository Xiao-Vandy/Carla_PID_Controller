import carla
import matplotlib.pyplot as plt
import numpy as np
import pathlib

towns_list = ['Town01', 'Town02','Town07']
#towns_list = ['Town02']

args = {}

for town in towns_list:

    args['map'] = town

    # Read the indices from the provided file
    file_path = f"./Town_required_waypoints/{args['map']}_required_waypoints.txt"
    with open(file_path, 'r') as file:
        indices_str = file.read().strip()
        specific_indices = list(map(int, indices_str.strip('[]').split(',')))

    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and the map
    world = client.get_world()
    if args['map'] is not None:
        print('load map %r.' % args['map'])
        world = client.load_world(args['map'])
    
    carla_map = world.get_map()

    # Retrieve all spawn points
    spawn_points = carla_map.get_spawn_points()

    # Format the waypoints in the requested format
    waypoints_output = []
    for i in specific_indices:
        point = spawn_points[i].location
        rotation = spawn_points[i].rotation
        waypoint = f'<waypoint pitch="0.0" roll="0.0" x="{point.x}" y="{point.y}" yaw="{rotation.yaw}" z="0.0" />'
        waypoints_output.append(waypoint)

    # Define the output file path
    output_file_path = f"./Town_required_waypoints/{args['map']}_waypoints_xml_format.txt"

    # Write the waypoints to a text file
    with open(output_file_path, 'w') as output_file:
        output_file.write('<waypoints>\n')
        for waypoint in waypoints_output:
            output_file.write(waypoint + '\n')
        output_file.write('</waypoints>')

    print(f"Waypoints for {town} have been saved to {output_file_path}")