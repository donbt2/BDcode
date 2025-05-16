import time
import sys
from bosdyn.client import create_standard_sdk
from bosdyn.client.robot import Robot
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.util import authenticate
from bosdyn.client.estop import EstopClient, EstopKeepAlive, EstopEndpoint

def main():
    if len(sys.argv) != 3:
        print("Usage: python spot_basic_script.py <ROBOT_IP> <USERNAME:PASSWORD>")
        return

    robot_ip = sys.argv[1]
    username, password = sys.argv[2].split(":")

    # Create SDK and robot object
    sdk = create_standard_sdk("SpotBasicClient")
    robot = sdk.create_robot(robot_ip)

    # Authenticate and time sync
    authenticate(robot)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()

    # Clients
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    power_client = robot.ensure_client(PowerClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    # Set up e-stop
    endpoint = EstopEndpoint(client=estop_client, name="SpotBasicEndpoint", timeout=9.0)
    endpoint.force_simple_setup()
    estop_keep_alive = EstopKeepAlive(endpoint)

    # Acquire lease
    lease = lease_client.acquire()

    # Power on
    print("Powering on Spot...")
    power_client.power_on(timeout_sec=20)
    while not robot.is_powered_on():
        time.sleep(1)

    print("Spot is powered on. Sending stand command...")
    command_client.robot_command(RobotCommandBuilder.synchro_stand_command())
    time.sleep(10)

    # Power off
    print("Command complete. Powering off Spot...")
    power_client.safe_power_off()

    # Return lease and stop e-stop
    lease_client.return_lease()
    estop_keep_alive.shutdown()
    print("Done.")

if __name__ == '__main__':
    main()
