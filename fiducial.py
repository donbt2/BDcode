import time
import argparse
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.power import PowerClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import blocking_stand
from bosdyn.api import geometry_pb2

# Constants for default values
TARGET_DISTANCE = 1.0  # Default distance to maintain (meters)
MAX_SPEED = 0.5  # Maximum speed (meters per second)
MIN_SPEED = 0.1  # Minimum speed (meters per second), to ensure smooth stopping
TAG_ID = 500  # Default AprilTag ID to follow
TAG_LOSS_TIMEOUT = 10.0  # Timeout period (seconds) to declare the tag as lost

def get_tag_pose(world_object_client, robot_state_client, tag_id):
    """Fetches the pose of the specified AprilTag relative to the robot's body."""
    try:
        # Retrieve the list of detected world objects
        objects = world_object_client.list_world_objects()
        # Get the robot's current state
        robot_state = robot_state_client.get_robot_state()
        # Calculate the transformation from the robot's vision frame to its body frame
        vision_tform_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)

        # Search through the objects for the specified AprilTag ID
        for obj in objects.world_objects:
            if obj.apriltag_properties and obj.apriltag_properties.tag_id == tag_id:
                try:
                    vision_tform_tag = get_a_tform_b(obj.transforms_snapshot, VISION_FRAME_NAME, obj.apriltag_properties.frame_name_fiducial)
                    body_tform_tag = vision_tform_body.inverse() * vision_tform_tag
                    return body_tform_tag  # Return the tag's pose relative to the robot's body
                except Exception:
                    continue  # Skip if there's an error in transforming coordinates
    except Exception as e:
        print(f"Failed to get tag pose: {e}")
    return None  # Return None if the tag was not found

def is_obstacle_nearby(robot_state_client):
    """Determines if there is an obstacle within a predefined distance from Spot."""
    # Retrieve the current state of the robot
    robot_state = robot_state_client.get_robot_state()
    # Define a threshold distance (meters) to consider a detection as an obstacle
    obstacle_threshold = 0.5

    # Iterate through the transform snapshots to check for proximity to obstacles
    for frame_name, transform in robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.items():
        # Check for depth-related frames (or replace with appropriate frame names)
        if "depth" in frame_name.lower():
            distance = transform.parent_tform_child.position.x
            if distance < obstacle_threshold:
                print(f"Obstacle detected at distance: {distance} meters.")
                return True  # Return True if an obstacle is detected

    return False  # Return False if no obstacles are detected

def follow_tag(robot_command_client, world_object_client, robot_state_client, target_distance, max_speed, tag_id):
    """Instructs the robot to follow the specified AprilTag while avoiding obstacles."""
    print("Following AprilTag...")
    # Initialize the variable to track the last time the tag was seen
    last_seen_time = time.time()

    while True:
        # Check for nearby obstacles and stop if any are detected
        if is_obstacle_nearby(robot_state_client):
            print("Obstacle detected! Stopping.")
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())
            time.sleep(0.5)
            continue
        
        # Get the current pose of the tag
        tag_pose = get_tag_pose(world_object_client, robot_state_client, tag_id)
        if tag_pose is None:
            # If the tag is not found, calculate the time since it was last seen
            print("Tag not found. Scanning...")
            time_since_seen = time.time() - last_seen_time
            if time_since_seen > TAG_LOSS_TIMEOUT:
                print("Tag lost. Please reposition the robot or the tag.")
                # Stop the robot if the tag has been lost for too long
                robot_command_client.robot_command(RobotCommandBuilder.stop_command())
                time.sleep(0.5)
            continue
        else:
            # Update the last seen time if the tag is found
            last_seen_time = time.time()

        # Extract the distance in the 'x' axis and the lateral offset 'y' from the tag's position
        distance = tag_pose.position.x
        offset_y = tag_pose.position.y

        # Adjust speed dynamically based on distance from the target
        speed = max(min(max_speed, distance - target_distance), MIN_SPEED)

        # If the distance to the tag is greater than the target distance, calculate velocities
        if distance > target_distance + 0.2:
            vx = speed  # Forward velocity
            vy = offset_y * 0.5  # Lateral velocity for smooth steering
        else:
            vx, vy = 0.0, 0.0  # Stop if within target distance

        # Construct the robot command to move towards the tag
        cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=vx,
            v_y=vy,
            v_rot=0.0,  # No rotation
            body_frame_name=BODY_FRAME_NAME
        )

        # Send the command to the robot
        robot_command_client.robot_command(cmd)
        time.sleep(0.1)  # Sleep briefly to reduce command rate

def main():
    """Main function to set up and start the robot's following behavior."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Spot Follow QR Code Script")
    parser.add_argument('username', help='Username to authenticate with the robot')
    parser.add_argument('password', help='Password to authenticate with the robot')
    parser.add_argument('robot_ip', help='IP address of Spot robot')
    parser.add_argument('--tag-id', type=int, default=TAG_ID, help='AprilTag ID to follow')
    parser.add_argument('--target-distance', type=float, default=TARGET_DISTANCE, help='Target following distance (meters)')
    parser.add_argument('--max-speed', type=float, default=MAX_SPEED, help='Maximum approach speed (m/s)')
    args = parser.parse_args()

    # Create SDK and instantiate the robot
    sdk = create_standard_sdk('SpotFollowQR')
    robot = sdk.create_robot(args.robot_ip)
    # Authenticate with the robot using provided credentials
    robot.authenticate(args.username, args.password)

    # Synchronize time with the robot
    robot.time_sync.wait_for_sync()
    # Lease handling to ensure exclusive control over the robot
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease = lease_client.take()

    # Power on the robot
    power_client = robot.ensure_client(PowerClient.default_service_name)
    power_client.power_on(timeout_sec=20)

    # Create necessary clients to control the robot and retrieve state information
    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    # Command the robot to stand, initializing its posture
    blocking_stand(robot_command_client)
    # Begin following the specified tag while checking for obstacles
    follow_tag(robot_command_client, world_object_client, robot_state_client,
               args.target_distance, args.max_speed, args.tag_id)

# Run the main function when the script is executed
if __name__ == '__main__':
    main()