# Boston Dynamics Spot – SDK Setup & Basic Control Script

This repository provides:

1. A basic Python script to interact with Boston Dynamics' Spot robot (`spot_basic_script.py`)
2. Step-by-step instructions to set up your development environment using the official Spot SDK

> ⚠️ This guide assumes you have access to a Spot robot and the login credentials for said robot

---

## Contents

* `spot_basic_script.py` – Authenticates, powers on Spot, makes it stand, spin in a circle, move vertically up and down,  and power off.
* Spot SDK setup instructions – Based on [Boston Dynamics' quickstart guide](https://dev.bostondynamics.com/docs/python/quickstart).

---

## 🔧 1. Spot SDK Environment Setup

> 📝 **Note:** These commands are meant to be run step-by-step in your terminal, not as a shell script.

### Clone the SDK Repository

```bash
git clone https://github.com/boston-dynamics/spot-sdk.git
```

### Set Up a Virtual Environment

```bash
python3 -m pip install virtualenv
python3 -m virtualenv --python=/usr/bin/python3 helloboston
source helloboston/bin/activate
```

### Install SDK Packages

```bash
python3 -m pip uninstall bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
python3 -m pip install bosdyn-client bosdyn-mission
```

### Verify SDK Installation

```bash
python3
>>> import bosdyn.client
>>> help(bosdyn.client)
>>> exit()
```

### Ping Your Spot Robot

```bash
ping <spot-ip>
python3 -m bosdyn.client <spot-ip> id
```

---

## 2. Run Official "Hello Spot" Demo

### Navigate to the Example

```bash
cd ~/spot-sdk/python/examples/hello_spot
python3 -m pip install -r requirements.txt
```

### Set Your Environment Variables

```bash
export BOSDYN_CLIENT_USERNAME=<your-username>
export BOSDYN_CLIENT_PASSWORD=<your-password>
```

### Run the Script

```bash
python3 hello_spot.py <spot-ip>
```

> 🚩 This WILL fail due to E-Stop errors. This is intentional. Proceed to the next step.

---

## 🚨 3. Set Up an E-Stop (Emergency Stop)

```bash
cd ~/spot-sdk/python/examples/estop
python3 -m pip install -r requirements.txt
python3 estop_nogui.py <spot-ip>
```

Now try the Hello Spot script again:

```bash
cd ~/spot-sdk/python/examples/hello_spot
python3 hello_spot.py <spot-ip>
```

> Spot will perform some poses, take a photo, and sit. Use the E-Stop GUI to safely end the session.

---

## 🧠 4. Custom Script – `spot_basic_script.py`

### Description

This script connects to Spot, authenticates, powers it on, sends a stand command, waits 10 seconds, then safely powers off.

### Script Features

* Authenticates with Spot
* Syncs time
* Acquires a lease
* Sets up a simple Estop endpoint
* Powers on Spot
* Commands Spot to stand
* Waits 10 seconds
* Powers off and cleans up

### Usage

```bash
python spot_basic_script.py <ROBOT_IP> <USERNAME:PASSWORD>
```

### Example

```bash
python spot_basic_script.py 192.168.80.3 admin:admin123
```

> ⚠️ Ensure you’ve set up the Estop before running this, or the command will fail.

---

## 📁 File Overview

### `spot_basic_script.py`

```python
# This script connects to Spot, authenticates, powers on,
# sends a stand command, and then powers off.

# Usage: python spot_basic_script.py <ROBOT_IP> <USERNAME:PASSWORD>
```

---

## 📝 Notes

* Make sure Spot is in a safe and open environment before running any movement commands.
* Always use a properly configured Estop during testing.
* The SDK and API are subject to Boston Dynamics’ license agreement.

---

## 📜 License

This script and guide are provided for educational and internal development purposes.
Usage of the Spot SDK is governed by Boston Dynamics’ licensing terms.

For more information, visit: [https://www.bostondynamics.com](https://www.bostondynamics.com)

---
