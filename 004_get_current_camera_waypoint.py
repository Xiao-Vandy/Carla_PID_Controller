import carla

def print_camera_waypoint(world, map):
    # Get the spectator (camera) object
    spectator = world.get_spectator()
    
    # Get the camera's transform (location and rotation)
    camera_transform = spectator.get_transform()
    camera_location = camera_transform.location
    
    # Print the camera's current location
    print(f"Camera location: x={camera_location.x}, y={camera_location.y}, z={camera_location.z}")
    
    # Get the closest waypoint to the camera's location
    waypoint = map.get_waypoint(camera_location, project_to_road=True)
    
    # Print the waypoint details
    print(f"Nearest waypoint to the camera: Road id={waypoint.road_id}, Lane id={waypoint.lane_id}")
    print(f"Waypoint location: x={waypoint.transform.location.x}, y={waypoint.transform.location.y}, z={waypoint.transform.location.z}")

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # Get the world and mapXiaoProj/Town_required_waypoints
    world = client.get_world()
    map = world.get_map()
    
    # Print the current camera (spectator) waypoint
    print_camera_waypoint(world, map)

if __name__ == "__main__":
    main()

