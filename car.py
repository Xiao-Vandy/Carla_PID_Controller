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


def low_pass_filter(current_value, previous_value, alpha):
    return alpha * current_value + (1 - alpha) * previous_value

def get_speed(vehicle):
    # Unit: km/h
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def calculate_lateral_error(current_location, target_location, target_heading):
    # Calculate the vector from the target to the current location
    dx = current_location.x - target_location.x
    dy = current_location.y - target_location.y

    # Rotate the heading by 90 degrees to get the direction perpendicular to the path
    perpendicular_vector = np.array([-np.sin(target_heading), np.cos(target_heading)])

    # Project the error vector onto the perpendicular direction
    error_vector = np.array([dx, dy])
    lateral_error = np.dot(error_vector, perpendicular_vector)

    return lateral_error

def distance_vehicle(waypoint, vehicle_transform):
    
    if not isinstance(waypoint, carla.Transform):
        raise TypeError(f"waypoint is of type {type(waypoint)}, but carla.Transform is expected.")
    
    loc = vehicle_transform.location
    x = waypoint.location.x - loc.x
    y = waypoint.location.y - loc.y

    return math.sqrt(x * x + y * y)

def draw_waypoints(world, waypoints, z=0.5):
  
    for wpt in waypoints:
        # Directly use the location and rotation from the Transform object
        begin = wpt.location + carla.Location(z=z)
        angle = math.radians(wpt.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=20.0)

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
        waypoints.append(waypoint_transform)
        
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

class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0, 255, (height, width, 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))

def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:, :, :3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))

def render_text(display, text, x, y, font, color=(255, 255, 255)):
    # print(f"Rendering text: {text} at ({x}, {y})")
    text_surface = font.render(text, True, color)
    display.blit(text_surface, (x, y))

def main():
    map_name = 'Town01'
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

    route_planner = RoutePlanner(map, waypoints)
    route_planner.get_safe_waypoints()
    safe_waypoints = route_planner.safe_waypoints
    # draw_waypoints(world, safe_waypoints)

    T = 10000
    for i in range(len(safe_waypoints) - 1):
        start_waypoint = safe_waypoints[i]
        end_waypoint = safe_waypoints[i + 1]
        start_location = start_waypoint.location
        end_location = end_waypoint.location
        start_location.z += 0.5
        end_location.z += 0.5
        world.debug.draw_line(start_location, end_location, thickness=0.2, color=carla.Color(b=255), life_time=T)
        start_location.z += 0.1
        world.debug.draw_line(start_location, end_location, thickness=0.2, color=carla.Color(r=255), life_time=T)
    
    spawn_points = map.get_spawn_points()
    if len(spawn_points) == 0:
        raise RuntimeError("No spawn points found")


    # Initialize the results dictionary outside the loop
    results = {}

    # Define the list of target speeds
    target_speed = 30

    # Define a list of PID configurations
    pid_configs = [

       {
        'args_lateral_dict': {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07,
            'dt': 1.0 / 10.0,
        },
        'args_longitudinal_dict': {
            'K_P': 1.0,
            'K_D': 0.0,
            'K_I': 0.75,
            'dt': 1.0 / 10.0,
        },
        
    },
    {
        'args_lateral_dict': {
            'K_P': 1.8,
            'K_D': 0.3,
            'K_I': 0.05,
            'dt': 1.0 / 10.0,
        },
        'args_longitudinal_dict': {
            'K_P': 1.0,
            'K_D': 0.0,
            'K_I': 0.75,
            'dt': 1.0 / 10.0,
        },
    },
]

    # for target_speed in target_speeds:
    for config_id, pid_config in enumerate(pid_configs):
        args_lateral_dict = pid_config['args_lateral_dict']
        args_longitudinal_dict = pid_config['args_longitudinal_dict']
        # Re-initialize data lists for each trial
        throttle_data = []
        brake_data = []
        lateral_error_data = []
        route_completion_data = []

        # Re-initialize simulation variables
        i = 0
        alpha = 0.2  # Smoothing factor
        previous_steer = 0.0
        current_view_index = 0

        # Initialize the vehicle at the starting point
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_point = carla.Transform(
            carla.Location(
            x=safe_waypoints[0].location.x,
            y=safe_waypoints[0].location.y,
            z=safe_waypoints[0].location.z + 1.0
        ),
        carla.Rotation(yaw=safe_waypoints[0].rotation.yaw)
        )
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
            raise RuntimeError("Spawn failed due to collision at spawn position")
        
        pid_controller = VehiclePIDController(vehicle, args_lateral_dict, args_longitudinal_dict)

        next_waypoint = safe_waypoints[0]

   
        throttle_data = []
        brake_data = []
        lateral_error_data = []
        route_completion_data = []

    
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        renderObject = RenderObject(800, 600)
        camera.listen(lambda image: pygame_callback(image, renderObject))

        pygame.init()
        gameDisplay = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
        gameDisplay.fill((0,0,0))
        gameDisplay.blit(renderObject.surface, (0,0))
        pygame.display.flip()
        clock = pygame.time.Clock()
        pygame.font.init()
        font = pygame.font.Font(None, 36)

        views = [
            # carla.Transform(carla.Location(x=0.0, y=0.0, z=50.00), carla.Rotation(pitch=-90)),
            carla.Transform(carla.Location(x=-10.0, y=0.0, z=4.0), carla.Rotation(pitch=0)),
            carla.Transform(carla.Location(x=0.5, y=0.0, z=1.3), carla.Rotation(pitch=0)) 
        ]
 
        camera.set_transform(views[current_view_index])
        try:
            print("Press Tab to switch camera view, ESC to exit")
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                    elif event.key == pygame.K_TAB:
                        current_view_index = (current_view_index + 1) % len(views)
                        camera.set_transform(views[current_view_index])
        
                vehicle_transform = vehicle.get_transform()
                spectator = world.get_spectator()
                spectator.set_transform(carla.Transform(vehicle_transform.location+carla.Location(z=80), carla.Rotation(pitch=-90)))
                current_location = vehicle_transform.location
                # print(f"current_location: {current_location}")
                # current_waypoint = safe_waypoints[i % len(safe_waypoints)]
                #print(f"current_location: {current_waypoint.location}")
                next_waypoint = safe_waypoints[(i + 1) % len(safe_waypoints)]
                # print(f"Type of next_waypoint.location: {type(next_waypoint.location)}")
                # print(f"Type of next_waypoint.rotation: {type(next_waypoint.rotation)}")

                if not isinstance(next_waypoint, carla.Transform):
                    raise TypeError(f"next_waypoint is of type {type(next_waypoint)}, but carla.Transform is expected.")

                # print(f"next_waypoint: {next_waypoint.location}")
                # Get vehicle heading
                vehicle_yaw = vehicle_transform.rotation.yaw
                # Assuming next_waypoint is an instance of a waypoint class
                next_waypoint_yaw = next_waypoint.rotation.yaw  # Get the yaw from the rotation attribute
                # print(type(next_waypoint))  # Debug to see what `next_waypoint` is


                # Calculate lateral error
                lateral_error = calculate_lateral_error(current_location, next_waypoint.location, np.deg2rad(next_waypoint_yaw))

                control = pid_controller.run_step(target_speed, next_waypoint)
                vehicle.apply_control(control)
                control.steer = low_pass_filter(control.steer, previous_steer, alpha)
                previous_steer = control.steer
                
                # Display vehicle information
                speed = get_speed(vehicle)
                throttle = control.throttle
                brake = control.brake
                steer = control.steer

                location_text_x = f"x: {current_location.x:.2f}"
                location_text_y = f"y: {current_location.y:.2f}"

                # Update display
                gameDisplay.fill((0, 0, 0))
                gameDisplay.blit(renderObject.surface, (0, 0))

                # location_text_z = f"z: {current_location.z:.2f}"
                render_text(gameDisplay, f"Speed: {speed:.2f} km/h", 10, 10, font)
                render_text(gameDisplay, f"Throttle: {throttle:.2f}", 10, 50, font)
                render_text(gameDisplay, f"Brake: {brake:.2f}", 10, 90, font)
                render_text(gameDisplay, f"Steer: {steer:.2f}", 10, 130, font)
                render_text(gameDisplay, f"{location_text_x} ", 10, 170, font)
                render_text(gameDisplay, f"{location_text_y} ", 10, 210, font)
                # render_text(gameDisplay, f"{location_text_z} ", 10, 240, font)

                # Update data lists for current speed
                throttle_data.append(throttle)
                brake_data.append(brake)
                lateral_error_data.append(lateral_error)

                # Calculate route completion percentage
                completion = (i / len(safe_waypoints)) * 100
                route_completion_data.append(completion)
            
                pygame.display.flip()
                clock.tick(20)
                if i == (len(safe_waypoints) - 1):
                    control = pid_controller.run_step(0, safe_waypoints[-1])
                    vehicle.apply_control(control)
                    print("Arrived at the destination")
                    break
        
                vehicle_dist = distance_vehicle(next_waypoint, vehicle_transform)
        
                if vehicle_dist < 1.5:
                    i += 1
                    next_waypoint = safe_waypoints[i % len(safe_waypoints)]
       
            # Store results for this speed
            results[f"Config {config_id + 1}"] = {
                'throttle': throttle_data,
                'brake': brake_data,
                'lateral_error': lateral_error_data,
                'route_completion': route_completion_data,
            }

        finally:
            camera.destroy()
            vehicle.destroy()
            pygame.quit()
    
    print(123)
    # Plot comparison for all speeds after the simulations
    fig, axes = plt.subplots(2, 1, figsize=(12, 20), sharex=True)

    # Plot comparison for all configurations after the simulations
    # fig, axes = plt.subplots(4, 1, figsize=(12, 20), sharex=True)

    # Create comparison plots
    # for config_name, result in results.items():
    for config_name, result in results.items():
        time_steps = np.arange(len(result['throttle']))  # Assuming data was recorded per simulation step

        # # Throttle Comparison
        # axes[0].plot(time_steps, result['throttle'], label=target_speed)
        # axes[0].set_ylabel("Throttle")
        # axes[0].legend()
        # axes[0].set_title("Throttle Comparison for Different Configurations")

        # # Brake Comparison
        # axes[1].plot(time_steps, result['brake'], label=target_speed)
        # axes[1].set_ylabel("Brake")
        # axes[1].legend()
        # axes[1].set_title("Brake Comparison for Different Configurations")

        # Lateral Error Comparison
        axes[0].plot(time_steps, result['lateral_error'], label=config_name)
        axes[0].set_ylabel("Lateral Error")
        axes[0].legend()
        axes[0].set_title("Lateral Error Comparison for Different Configurations")

        # Route Completion Comparison
        axes[1].plot(time_steps, result['route_completion'], label=config_name)
        axes[1].set_ylabel("Route Completion (%)")
        axes[1].legend()
        axes[1].set_title("Route Completion for Different Configurations")

    # Label the x-axis
    axes[1].set_xlabel("Time Step")

    # Tight layout for better appearance
    plt.tight_layout()

    plt.savefig(f"./output/{map_name}_comparison_plot_configs_target_speed={target_speed}.png")
    # plt.savefig(f"./output/{map_name}_target_speed={target_speed}.png")



if __name__ == "__main__":
    main()