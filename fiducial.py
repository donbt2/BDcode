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
from bosdyn.client import math_helpers
import sys

TARGET_DISTANCE = 1.0  # Default distance to maintain (meters)
MAX_SPEED = 0.5  # Maximum speed (m/s)
MIN_SPEED = 0.1  # Minimum speed to ensure smooth stopping
TAG_ID = 500  # Default AprilTag ID
TAG_LOSS_TIMEOUT = 10.0  # Seconds to wait before declaring tag lost

def get_tag_pose(world_object_client, robot_state_client, tag_id):
    try:
        objects = world_object_client.list_world_objects()
        robot_state = robot_state_client.get_robot_state()
        vision_tform_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)

        for obj in objects.world_objects:
            if obj.apriltag_properties and obj.apriltag_properties.tag_id == tag_id:
                try:
                    vision_tform_tag = get_a_tform_b(obj.transforms_snapshot, VISION_FRAME_NAME, obj.apriltag_properties.frame_name_fiducial)
                    body_tform_tag = vision_tform_body.inverse() * vision_tform_tag
                    return body_tform_tag
                except Exception:
                    continue
    except Exception as e:
        print(f"Failed to get tag pose: {e}", file=sys.stderr)
    return None

def is_obstacle_nearby(robot_state_client):
    state = robot_state_client.get_robot_state()
    front_proximity = state.kinematic_state.foot_height_time_series

    # Insert logic to check if there's an obstacle nearby using Spot's proximal sensors
    # This is a placeholder example
    obstacle_threshold = 0.5
    return any(detection.range < obstacle_threshold for detection in front_proximity)

def follow_tag(robot_command_client, world_object_client, robot_state_client, target_distance, max_speed, tag_id):
    print("Following AprilTag...")
    last_seen_time = time.time()

    while True:
        if is_obstacle_nearby(robot_state_client):
            print("Obstacle detected! Stopping.")
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())
            time.sleep(0.5)
            continue
        
        tag_pose = get_tag_pose(world_object_client, robot_state_client, tag_id)
        if tag_pose is None:
            print("Tag not found. Scanning...")
            time_since_seen = time.time() - last_seen_time
            if time_since_seen > TAG_LOSS_TIMEOUT:
                print("Tag lost. Please reposition the robot or the tag.")
                robot_command_client.robot_command(RobotCommandBuilder.stop_command())
                time.sleep(0.5)
            continue
        else:
            last_seen_time = time.time()

        distance = tag_pose.position.x
        offset_y = tag_pose.position.y

        # Dynamic speed based on distance
        speed = max(min(max_speed, distance - target_distance), MIN_SPEED)

        if distance > target_distance + 0.2:
            vx = speed
            vy = offset_y * 0.5  # smooth steering
        else:
            vx, vy = 0.0, 0.0

        cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=vx,
            v_y=vy,
            v_rot=0.0,
            body_frame_name=BODY_FRAME_NAME
        )

        robot_command_client.robot_command(cmd)
        time.sleep(0.1)

def main():
    parser = argparse.ArgumentParser(description="Spot Follow QR Code Script")
    parser.add_argument('username', help='Username to authenticate with the robot')
    parser.add_argument('password', help='Password to authenticate with the robot')
    parser.add_argument('robot_ip', help='IP address of Spot robot')
    parser.add_argument('--tag-id', type=int, default=TAG_ID, help='AprilTag ID to follow')
    parser.add_argument('--target-distance', type=float, default=TARGET_DISTANCE, help='Target following distance (meters)')
    parser.add_argument('--max-speed', type=float, default=MAX_SPEED, help='Maximum approach speed (m/s)')
    args = parser.parse_args()

    sdk = create_standard_sdk('SpotFollowQR')
    robot = sdk.create_robot(args.robot_ip)
    robot.authenticate(args.username, args.password)

    robot.time_sync.wait_for_sync()
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease = lease_client.take()

    power_client = robot.ensure_client(PowerClient.default_service_name)
    power_client.power_on(timeout_sec=20)

    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    blocking_stand(robot_command_client)
    follow_tag(robot_command_client, world_object_client, robot_state_client,
               args.target_distance, args.max_speed, args.tag_id)


if __name__ == '__main__':
    main()