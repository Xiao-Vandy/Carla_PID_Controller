import carla
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import mplcursors

towns_list = ['Town01', 'Town02','Town03','Town04','Town05','Town06','Town07']
args = {}
for town in towns_list:

    args['map'] = town
    # create a client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    if args['map'] is not None:
        print('load map %r.' % args['map'])
        world = client.load_world(args['map'])
    
    map = world.get_map()

    spawn_points = map.get_spawn_points()

    x_coords = np.array([point.location.x for point in spawn_points])
    y_coords = np.array([point.location.y for point in spawn_points])

    print(x_coords)
    print(y_coords)

    topology = map.get_topology()
    plt.figure(figsize=(14, 10))  # Make the plot larger
    for segment in topology:
        road_start = segment[0].transform.location
        road_end = segment[1].transform.location
        plt.plot([road_start.x, road_end.x], [road_start.y, road_end.y], color='gray', linewidth=2)

    scatter = plt.scatter(x_coords, y_coords, c='blue', marker='o', label='Spawn Points')

    plt.title(f"carla town map with spawn points: {town}")
    plt.xlabel("x coordinates")
    plt.ylabel("y coordinates")
    plt.legend()
    plt.grid(False)

    # Assuming you want to calculate yaws for each spawn point
    yaws = [point.rotation.yaw for point in spawn_points]  # Extract yaw from each spawn point

    annotations = [f"Index: {i}\nYaw: {yaw:.2f}" for i, yaw in enumerate(yaws)]
    mplcursors.cursor(scatter, multiple=True).connect(
        "add", lambda sel: sel.annotation.set_text(annotations[sel.index])
    )

    output_dir = 'Town_spawn_points'
    pathlib.Path(f"./{output_dir}").mkdir(parents=True, exist_ok=True)

    plt.savefig(f"./{output_dir}/{town}_spawn_points_interactive.png")

    plt.show()
