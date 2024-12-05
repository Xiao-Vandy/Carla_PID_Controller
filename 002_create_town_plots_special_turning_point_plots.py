import carla
import matplotlib.pyplot as plt
import numpy as np
import pathlib

towns_list = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07']
#towns_list = ['Town01']

args = {}

for town in towns_list:
    args['map'] = town

    file_path = f"./Town_required_waypoints/{args['map']}_required_waypoints.txt"
    try:
        with open(file_path, 'r') as file:
            indices_str = file.read().strip()
            if indices_str:
                specific_indices = list(map(int, indices_str.strip('[]').split(',')))
            else:
                print(f"No valid indices found in file: {file_path}. Skipping {town}.")
                continue
    except FileNotFoundError:
        print(f"File not found: {file_path}. Skipping {town}.")
        continue

    print(specific_indices)
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    if args['map'] is not None:
        print('load map %r.' % args['map'])
        world = client.load_world(args['map'])

    carla_map = world.get_map()

    spawn_points = carla_map.get_spawn_points()

    x_coords = np.array([point.location.x for point in spawn_points])
    y_coords = np.array([point.location.y for point in spawn_points])

    topology = carla_map.get_topology()
    plt.figure(figsize=(12, 12))
    plt.scatter(x_coords, y_coords, c='blue', label='Spawn Points')
    for segment in topology:
        road_start = segment[0].transform.location
        road_end = segment[1].transform.location
        plt.plot([road_start.x, road_end.x], [road_start.y, road_end.y], color='gray', linewidth=2)

    specific_x_coords = [x_coords[i] for i in specific_indices]
    specific_y_coords = [y_coords[i] for i in specific_indices]
    plt.scatter(specific_x_coords, specific_y_coords, c='red', label='Specific Spawn Points')

    for i, (x, y) in zip(specific_indices, zip(specific_x_coords, specific_y_coords)):
        plt.text(x, y, str(i), fontsize=12, color='green')

    plt.title('Specific Spawn Points on CARLA Town Map')
    plt.xlabel('X Coordinates')
    plt.ylabel('Y Coordinates')
    plt.legend()
    plt.grid(False)
    output_dir = 'Town required Turning Points'
    pathlib.Path(f"./{output_dir}").mkdir(parents=True, exist_ok=True)
    plt.savefig(f"./{output_dir}/{town}_required_turning_points.png")