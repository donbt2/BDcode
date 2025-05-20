import time
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.power import PowerClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import blocking_stand

TARGET_DISTANCE = 1.0  # meters
SPEED = 0.5  # m/s
TAG_ID = 500  # Replace with your tag's ID (as recognized by Spot)

def get_tag_pose(world_object_client, robot_state_client):
    objects = world_object_client.list_world_objects()
    robot_state = robot_state_client.get_robot_state()
    vision_tform_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)

    for obj in objects.world_objects:
        if obj.apriltag_properties and obj.apriltag_properties.tag_id == TAG_ID:
            try:
                vision_tform_tag = get_a_tform_b(obj.transforms_snapshot, VISION_FRAME_NAME, obj.apriltag_properties.frame_name_fiducial)
                body_tform_tag = vision_tform_body.inverse() * vision_tform_tag
                return body_tform_tag
            except Exception as e:
                continue
    return None

def follow_tag(robot_command_client, world_object_client, robot_state_client):
    print("Following AprilTag...")
    while True:
        tag_pose = get_tag_pose(world_object_client, robot_state_client)
        if tag_pose is None:
            print("Tag not found.")
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())
            time.sleep(0.2)
            continue

        distance = tag_pose.position.x
        offset_y = tag_pose.position.y

        if distance > TARGET_DISTANCE + 0.2:
            vx = min(SPEED, distance - TARGET_DISTANCE)
            vy = offset_y * 0.5  # smooth steering
            cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=vx,
                v_y=vy,
                v_rot=0.0,
                body_frame_name=BODY_FRAME_NAME
            )
        else:
            cmd = RobotCommandBuilder.stop_command()

        robot_command_client.robot_command(cmd)
        time.sleep(0.1)

def main():
    sdk = create_standard_sdk('SpotFollowQR')
    robot = sdk.create_robot('192.168.80.3')  # Replace with Spot's IP
    robot.authenticate('admin')  # Replace with your credentials

    robot.time_sync.wait_for_sync()
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease = lease_client.take()

    power_client = robot.ensure_client(PowerClient.default_service_name)
    power_client.power_on(timeout_sec=20)

    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    blocking_stand(robot_command_client)
    follow_tag(robot_command_client, world_object_client, robot_state_client)

if __name__ == '__main__':
    main()
