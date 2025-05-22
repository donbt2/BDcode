Boston Dynamics Spot – SDK Setup, Fiducial Detection & Basic Control Script
This repository provides:

A Python script (spot_basic_script.py) that connects to Spot, performs a demo sequence involving standing, spinning, moving vertically, and powering off.
Step-by-step instructions to set up the development environment and establish communication with Spot using the official Spot SDK.
An advanced fiducial detection module that detects and tracks AprilTags or fiducials using Spot's perception system or image processing.
Contents
spot_basic_script.py – Basic Spot control sequence
Fiducial detection and following module
SDK environment setup instructions
Estop (Emergency Stop) configuration
Usage examples and notes
1. Spot SDK Environment Setup
Note: Follow these instructions carefully. It is recommended to use a virtual environment to isolate dependencies.

Step-by-Step Instructions
# Clone the Spot SDK repository
git clone https://github.com/boston-dynamics/spot-sdk.git
cd spot-sdk

# Create and activate a Python virtual environment
python3 -m pip install virtualenv
python3 -m virtualenv --python=/usr/bin/python3 spotenv
source spotenv/bin/activate

# Install required Python dependencies
pip install bosdyn-client bosdyn-mission
Verify your setup
python3 -c "import bosdyn.client; help(bosdyn.client)"
2. Set Up Estop (Emergency Stop)
Spot requires an active Estop connection before issuing movement commands.

Run the Estop server locally (no GUI):

cd python/examples/estop
python3 -m pip install -r requirements.txt
python3 estop_nogui.py <ROBOT_IP>
Replace <ROBOT_IP> with your Spot's IP address.

Keep this terminal running while executing control scripts to maintain the Estop connection.

3. Basic Control Script – spot_basic_script.py
Description
This script connects to Spot, authenticates, powers it on, makes it stand, spins twice, moves vertically, then powers off. It ensures safety and proper cleanup.

Usage
python3 spot_basic_script.py <ROBOT_IP> <USERNAME:PASSWORD>
Example:

python3 spot_basic_script.py 192.168.80.3 admin:admin123
Note: Ensure the Estop system is engaged before executing the script, or the commands will fail.

Script Features
Authenticates with Spot
Synchronizes robot time
Acquires a lease
Sets up Estop endpoint
Powers Spot on and makes it stand
Performs a spin in place twice
Moves Spot's body up and down
Powers off safely
4. Fiducial Detection and Following Module
Overview
This module detects fiducial markers (AprilTags) in images from Spot’s cameras or through the perception system, computes their poses in the world coordinate frame, and commands Spot to approach or follow the detected fiducials.

Main capabilities
Detect fiducials using Spot's perception system or via AprilTag image processing.
Transform fiducial pose from camera frame to world frame.
Command Spot to navigate towards fiducials with offset adjustments.
Supports multiple camera sources and options for obstacle avoidance.
How it works
Detection: Uses Spot's WorldObject perception data or AprilTag detection via OpenCV and an AprilTag library.
Pose Estimation: Uses camera intrinsics and cv2.solvePnP to estimate fiducial position.
Navigation: Commands Spot to move to an offset position near the fiducial, adjusting heading based on fiducial location.
Usage
Integrate or run as part of your personal scripts to detect and follow fiducials.

5. Additional Notes & Best Practices
Always operate Spot in a safe, open environment during movements.
Use the Estop system correctly to avoid accidents.
Confirm your SDK, firmware, and perception system are compatible and up to date.
Respect licensing agreements; usage of SDK and API is governed by Boston Dynamics’ license.
6. License
This code and documentation are provided for educational and internal development purposes. Use of the Spot SDK is subject to Boston Dynamics’ licensing terms:

https://www.bostondynamics.com

7. Summary
This repository offers a complete setup guide for Spot SDK development, a basic control script for demo purposes, and a sophisticated fiducial detection/following system. Proper setup and safety measures are critical for successful operation.

8. Sample Fiducial Detection Script Snippet
Below is an outline of how fiducial detection and navigation work within the script:

import numpy as np
import cv2
from apriltag import apriltag

# Detect fiducials in an image
def detect_fiducial_in_image(image, dim, source_name):
    # Convert image bytes to numpy array
    image_grey = np.array(Image.frombytes('P', (int(dim[0]), int(dim[1])), data=image.data, decoder_name='raw'))
    # Rotate images for consistent orientation
    image_grey = rotate_image(image_grey, source_name)

    # Detect AprilTags
    detector = apriltag(family='tag36h11')
    detections = detector.detect(image_grey)

    bboxes = []
    for det in detections:
        bbox = det['lb-rb-rt-lt']
        # Draw bounding box (for visualization)
        cv2.polylines(image_grey, [np.int32(bbox)], True, (0, 0, 0), 2)
        bboxes.append(bbox)
    return bboxes

# Convert bounding box to object and image points for pose estimation
def bbox_to_image_object_pts(bbox):
    fiducial_size_mm = 146  # AprilTag side length in mm
    obj_pts = np.array([
        [0, 0],
        [fiducial_size_mm, 0],
        [0, fiducial_size_mm],
        [fiducial_size_mm, fiducial_size_mm]
    ], dtype=np.float32)

    img_pts = np.array([
        [bbox[3][0], bbox[3][1]],
        [bbox[2][0], bbox[2][1]],
        [bbox[0][0], bbox[0][1]],
        [bbox[1][0], bbox[1][1]]
    ], dtype=np.float32)
    return obj_pts, img_pts

# Pose estimation from image points
def pixel_coords_to_camera_coords(bboxes, intrinsics, source_name):
    camera_matrix = make_camera_matrix(intrinsics)
    best_pose = None
    min_dist = float('inf')
    for bbox in bboxes:
        obj_points, img_points = bbox_to_image_object_pts(bbox)
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            np.zeros((5, 1)),
            previous_guess=..., # optionally provide previous solution
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if success:
            dist = np.linalg.norm(tvec)
            if dist < min_dist:
                min_dist = dist
                best_pose = (rvec, tvec)
    return best_pose
Note: Replace '...' with your previous estimates or initial guesses if available. Adjust for your specific camera intrinsics.

Final Remarks
This setup provides a comprehensive foundation for interacting with Boston Dynamics Spot, performing safe control, and implementing fiducial-based navigation. Adapt the code and instructions to your specific environment and objectives.
