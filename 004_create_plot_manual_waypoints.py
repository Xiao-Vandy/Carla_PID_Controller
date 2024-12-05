import carla
import matplotlib.pyplot as plt
import numpy as np
import pathlib

# towns_list = ['Town01', 'Town02','Town03','Town04','Town05','Town06','Town07']#
towns_list = ['Town07']

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

    map = world.get_map()

    # Retrieve all spawn points
    spawn_points = map.get_spawn_points()

    # Extract the x, y coordinates of each spawn point
    x_coords = np.array([point.location.x for point in spawn_points])
    y_coords = np.array([point.location.y for point in spawn_points])

#    Get the topology to draw the map
    topology = map.get_topology()


    # Plot all spawn points (for reference)
    plt.figure(figsize=(12, 12))
    plt.scatter(x_coords, y_coords, c='blue', label='All Spawn Points')

    # Plot the map topology (roads)
    for segment in topology:
        road_start = segment[0].transform.location
        road_end = segment[1].transform.location
        plt.plot([road_start.x, road_end.x], [road_start.y, road_end.y], color='gray', linewidth=2)

    # Highlight the specific spawn points from the file
    specific_x_coords = [x_coords[i] for i in specific_indices]
    specific_y_coords = [y_coords[i] for i in specific_indices]
    plt.scatter(specific_x_coords, specific_y_coords, c='red', label='Specific Spawn Points')

    # Annotate each specific spawn point with its index
    for i, (x, y) in zip(specific_indices, zip(specific_x_coords, specific_y_coords)):
        plt.text(x, y, str(i), fontsize=12, color='green')

    # Add titles and labels
    plt.title('Specific Spawn Points on CARLA Town Map')
    plt.xlabel('X Coordinates')
    plt.ylabel('Y Coordinates')
    plt.legend()
    plt.grid(False)

    output_dir = 'Town_required_waypoints'
    pathlib.Path(f"./{output_dir}").mkdir(parents=True, exist_ok=True)
    plt.savefig(f"./{output_dir}/{args['map']}_required_waypoints.png")
