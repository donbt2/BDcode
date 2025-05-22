import time
import argparse
import numpy as np
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.power import PowerClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import blocking_stand

TARGET_DISTANCE = 1.0  # Default distance to maintain (meters)
SPEED = 0.5  # Default speed (m/s)
TAG_ID = 500  # Default AprilTag ID
OBSTACLE_RADIUS = 0.5  # Avoid obstacles within this radius (meters)


def get_tag_pose(world_object_client, robot_state_client, tag_id):
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
    return None


def detect_obstacles_and_avoidance_velocities(world_object_client):
    """
    Detect obstacles near the robot within OBSTACLE_RADIUS,
    and compute lateral velocity offset to avoid them.
    Returns: (obstacle_detected: bool, lateral_velocity_correction: float)
    """
    objects = world_object_client.list_world_objects()
    lateral_correction = 0.0
    detected = False

    for obj in objects.world_objects:
        if obj.has_geometry:
            pos = obj.pose.position
            dist = np.sqrt(pos.x ** 2 + pos.y ** 2)
            # Consider only obstacles in front (x > 0) and within detection radius
            if dist < OBSTACLE_RADIUS and pos.x > 0:
                detected = True
                # Steer right if obstacle is to the left (pos.y > 0), else steer left
                if pos.y > 0:
                    lateral_correction -= 0.3  # steer right
                else:
                    lateral_correction += 0.3  # steer left

    # Clamp lateral_correction to ±0.5 m/s
    lateral_correction = np.clip(lateral_correction, -0.5, 0.5)
    return detected, lateral_correction


def follow_tag(robot_command_client, world_object_client, robot_state_client, target_distance, speed, tag_id):
    print("Following AprilTag with dynamic obstacle avoidance...")

    while True:
        tag_pose = get_tag_pose(world_object_client, robot_state_client, tag_id)
        if tag_pose is None:
            print("Tag not found.")
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())
            time.sleep(0.2)
            continue

        distance = tag_pose.position.x
        offset_y = tag_pose.position.y

        obstacle_detected, lateral_vel_corr = detect_obstacles_and_avoidance_velocities(world_object_client)

        if obstacle_detected:
            print("Obstacle detected! Adjusting path...")
            vx = max(0.0, min(speed * 0.5, distance - target_distance))  # slow forward velocity
            vy = lateral_vel_corr + offset_y * 0.3                    # combine avoidance and tag offset
            vy = np.clip(vy, -speed, speed)
            v_rot = 0.0
        else:
            if distance > target_distance + 0.2:
                vx = min(speed, distance - target_distance)
                vy = offset_y * 0.5
                v_rot = 0.0
            else:
                robot_command_client.robot_command(RobotCommandBuilder.stop_command())
                time.sleep(0.1)
                continue

        cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=vx,
            v_y=vy,
            v_rot=v_rot,
            body_frame_name=BODY_FRAME_NAME
        )

        robot_command_client.robot_command(cmd)
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="Spot Follow AprilTag with Obstacle Avoidance")
    parser.add_argument('username', help='Username to authenticate with the robot')
    parser.add_argument('password', help='Password to authenticate with the robot')
    parser.add_argument('robot_ip', help='IP address of Spot robot')
    parser.add_argument('--tag-id', type=int, default=TAG_ID, help='AprilTag ID to follow')
    parser.add_argument('--target-distance', type=float, default=TARGET_DISTANCE, help='Target following distance (meters)')
    parser.add_argument('--speed', type=float, default=SPEED, help='Maximum approach speed (m/s)')
    args = parser.parse_args()

    sdk = create_standard_sdk('SpotFollowAprilTag')
    robot = sdk.create_robot(args.robot_ip)
    robot.authenticate(args.username, args.password)
    robot.time_sync.wait_for_sync()

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease = lease_client.take()
    lease_keep_alive = LeaseKeepAlive(lease_client)

    power_client = robot.ensure_client(PowerClient.default_service_name)
    power_client.power_on(timeout_sec=20)

    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    blocking_stand(robot_command_client)
    follow_tag(robot_command_client, world_object_client, robot_state_client,
               args.target_distance, args.speed, args.tag_id)


if __name__ == '__main__':
    main()