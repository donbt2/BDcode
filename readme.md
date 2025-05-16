# 🐶 Boston Dynamics Spot – SDK Setup & Basic Control Script

This repository provides:

1. A Python script to interact with Boston Dynamics' Spot robot (`spot_basic_script.py`) that performs a short demo sequence.
2. Step-by-step instructions to set up your development environment using the official Spot SDK

> This guide assumes you have access to a Spot robot and the corresponding credentials

---

## Contents

* `spot_basic_script.py` – Authenticates, powers on Spot, makes it stand, spins, changes height, and powers it off.
* Spot SDK setup instructions – Based on [Boston Dynamics' quickstart guide](https://dev.bostondynamics.com/docs/python/quickstart).

---

## 1. Spot SDK Environment Setup

 **Note:** Follow the official SDK documentation to install Python dependencies and verify your setup.

### Step-by-Step Instructions

```bash
# Clone the Spot SDK repository
git clone https://github.com/boston-dynamics/spot-sdk.git
cd spot-sdk

# Create and activate a virtual environment
python3 -m pip install virtualenv
python3 -m virtualenv --python=/usr/bin/python3 spotenv
source spotenv/bin/activate

# Install required Python packages
python3 -m pip install bosdyn-client bosdyn-mission
```

To confirm your setup is working, run:

```bash
python3
>>> import bosdyn.client
>>> help(bosdyn.client)
>>> exit()
```

---

## 🤖 2. Set Up Estop (Emergency Stop)

Spot requires an Estop system to be engaged before executing any robot commands.

To set up a basic Estop server without a GUI:

```bash
cd python/examples/estop
python3 -m pip install -r requirements.txt
python3 estop_nogui.py <ROBOT_IP>
```

Replace `<ROBOT_IP>` with your Spot's IP address.

Leave this terminal running while executing control scripts.

---

## 3. Custom Script – `spot_basic_script.py`

### Description

This script connects to Spot, authenticates, powers it on, makes it stand, spins twice, moves up and down, then safely powers off.

### Script Features

* Authenticates with Spot
* Syncs time
* Acquires a lease
* Sets up a simple Estop endpoint
* Powers on Spot
* Commands Spot to:

  * Stand up
  * Spin in place twice
  * Move body up and down
* Powers off and cleans up

### Usage

```bash
python spot_basic_script.py <ROBOT_IP> <USERNAME:PASSWORD>
```

### Example

```bash
python spot_basic_script.py 192.168.80.3 admin:admin123
```

> Ensure you’ve set up the Estop before running this, or the command will fail.

---

## File Overview

### `spot_basic_script.py`

```python
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

    sdk = create_standard_sdk("SpotBasicClient")
    robot = sdk.create_robot(robot_ip)

    authenticate(robot)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    power_client = robot.ensure_client(PowerClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    endpoint = EstopEndpoint(client=estop_client, name="SpotBasicEndpoint", timeout=9.0)
    endpoint.force_simple_setup()
    estop_keep_alive = EstopKeepAlive(endpoint)

    lease = lease_client.acquire()

    print("Powering on Spot...")
    power_client.power_on(timeout_sec=20)
    while not robot.is_powered_on():
        time.sleep(1)

    print("Spot is powered on. Sending stand command...")
    command_client.robot_command(RobotCommandBuilder.synchro_stand_command())
    time.sleep(3)

    print("Spinning Spot twice...")
    for _ in range(2):
        spin_command = RobotCommandBuilder.synchro_velocity_command(0.0, 0.0, 1.0)
        command_client.robot_command(spin_command)
        time.sleep(3)
        command_client.robot_command(RobotCommandBuilder.synchro_velocity_command(0.0, 0.0, 0.0))
        time.sleep(1)

    print("Moving Spot up and down...")
    for _ in range(2):
        up_command = RobotCommandBuilder.synchro_stand_command(body_height=0.2)
        command_client.robot_command(up_command)
        time.sleep(2)
        down_command = RobotCommandBuilder.synchro_stand_command(body_height=-0.2)
        command_client.robot_command(down_command)
        time.sleep(2)

    command_client.robot_command(RobotCommandBuilder.synchro_stand_command())
    time.sleep(2)

    print("Command sequence complete. Powering off Spot...")
    power_client.safe_power_off()

    lease_client.return_lease()
    estop_keep_alive.shutdown()
    print("Done.")

if __name__ == '__main__':
    main()
```

---

## Notes

* Make sure Spot is in a safe and open environment before running any movement commands.
* Always use a properly configured Estop during testing.
* The SDK and API are subject to Boston Dynamics’ license agreement.

---

## License

This script and guide are provided for educational and internal development purposes.
Usage of the Spot SDK is governed by Boston Dynamics’ licensing terms.

For more information, visit: [https://www.bostondynamics.com](https://www.bostondynamics.com)

---
