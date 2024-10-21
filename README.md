# Steel Detection ROS Package README

## Overview

This README provides a step-by-step guide for setting up and running the Steel Detection ROS package within a Robot Operating System (ROS) environment. The package processes data from a camera and outputs detection results.

## Dataset

You can download the required dataset from the following link:

- [MulSIR-252](https://drive.google.com/drive/folders/1F2h8GPhrtqMsQicXzB6tLd1090A1p7v_?usp=drive_link)

## Prerequisites

- ROS installed (e.g., ROS Noetic)
- Catkin workspace created and initialized
- Compatible camera setup with the required topics published

## Setup Instructions

### Step 1: Build and Run the Node

1. **Navigate to the Robot Workspace:**

   ```
   cd robot_ws
   ```

2. **Build the Workspace:**

   ```
   catkin_make
   ```

3. **Source the Workspace:**

   ```
   source devel/setup.bash
   ```

4. **Start ROS Master:**

   ```
   roscore
   ```

5. **Run the Steel Detection Node:**

   ```
   rosrun steel_detection steel_detect
   ```

### Step 2: Play the ROS Bag

1. Play the ROS Bag File:

   Ensure that the bag file contains the required topics:

   - `/camera/color/image_raw`
   - `/camera/aligned_depth_to_color/image_raw`

   ```
   rosbag play xxx.bag
   ```

### Step 3: View the Topic Format

1. **Source the Workspace Again:**

   ```
   source devel/setup.bash
   ```

2. **Echo the Detection Topic:** To view the output of the steel detection process:

   ```
   rostopic echo /steel_detection
   ```

## Required Topics

Make sure that the ROS bag being played contains the following topics:

- `/camera/color/image_raw`: RGB camera feed.
- `/camera/aligned_depth_to_color/image_raw`: Depth camera feed aligned with the RGB camera.

## Troubleshooting

- If the steel detection node fails to start, ensure that all dependencies are correctly installed and that the workspace is sourced properly.
- If the required topics are not present in the ROS bag, check your camera setup and recording settings.

## Conclusion

This guide outlines the basic steps for running the Steel Detection ROS package. For further modifications or enhancements, refer to the package documentation or ROS wiki for advanced configurations.

