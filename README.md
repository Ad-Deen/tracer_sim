# PX4 & ROS 2 Drone Pursuit (Chaos Car Challenge)

This project implements a GPS-denied autonomous drone pursuit system using a Raspberry Pi 5 (or Simulation) and a Pixhawk 6C. The drone utilizes **Image-Based Visual Servoing (IBVS)** and a **Trapezoidal Safe Zone** logic to track a moving red car via a 20° downward-tilted camera.

## 🚀 Key Features

- **Dual Mode Control:** Toggle between Manual teleop and Autonomous pursuit using the `m` key.
- **GPS-Denied Navigation:** Optimized for Offboard mode using Optical Flow and Altitude Hold logic.
- **Perspective-Aware Detection:** A custom red car detector with a trapezoidal "sweet spot" to compensate for camera tilt and perspective distortion.
- **Robust Re-acquisition:** Logic to "coast" for 1.0 second after losing a target before declaring a lost lock.

---

## 🛠 Prerequisites & Setup

### 1. Download Required Meshes

Before building the package, you must download the custom vehicle mesh used in the simulation.

- **Download Link:** [Chaos Car Mesh File](https://drive.google.com/file/d/1YeTOY_UW7MulZmMu2iMyZAabGPCXxCoq/view?usp=sharing)
- **Instructions:** Download the file and place it in the directory where your URDF/SDF files expect it.

### 2. ⚠️ Important: Fixing File Paths

Currently, the **URDF files and Launch files** in this repository use **Absolute/Pure path references** (e.g., `/home/user/ros2_ws/...`) rather than ROS 2 package-relative paths.

**To run the simulation effortlessly, you must:**

1. Open the `.urdf` and `.launch.py` files.
2. Search for mesh `<mesh filename="...">` tags or file paths.
3. Update them to match the local paths on your machine, or ideally, convert them to use:Python
    
    `# Example of fixed path in launch files using get_package_share_directory
    from ament_index_python.packages import get_package_share_directory`
    

---

## 🏗 Installation & Build

1. **Clone the repository:**Bash
    
    `cd ~/ros2_ws/src
    git clone <your-repo-link>`
    
2. **Install dependencies:**Bash
    
    `pip install pynput opencv-python
    sudo apt install ros-humble-cv-bridge`
    
3. **Build the workspace:**Bash
    
    `cd ~/ros2_ws
    colcon build --packages-select tracer
    source install/setup.bash`
    

---

## 🎮 Running the Project

### 1. Start the Simulation & Detector

Run the Red Car Detector node to process camera frames and publish errors:

Bash

`ros2 run tracer detector`

### 2. Start the Teleop & Autonomous Agent

Run the control node to fly the drone:

Bash

`ros2 run tracer teleop`

### 3. Controls

- **'m' Key:** Toggle between **MANUAL** and **AUTO**.
- **Manual Mode:** Use `w, a, s, d, x, r, f, q, e` to fly.
- **Auto Mode:** The drone will automatically adjust `Vx` and `Yaw` to center the red car within the trapezoidal safe zone.

---

## 📊 Logic Overview

### The Trapezoidal Safe Zone

Unlike a standard rectangle, our safe zone is wider at the bottom than the top. This accounts for the **20° downward tilt** of the camera, ensuring that the drone's "ideal position" is physically consistent with the ground plane perspective.

### Re-acquisition Logic

If the car is occluded (e.g., drives under a bridge), the detector will:

1. Keep the `target_found (z)` flag set to `1.0` for **1.0 second**.
2. Zero out the errors to allow the drone to "coast" or hover.
3. If the car is not found within 1 second, the flag drops to `0.0`.

---

## 📝 Roadmap

- [x]  IBVS Error calculation with Trapezoidal constraints.
- [x]  Manual/Auto mode switching.
- [x]  Target re-acquisition grace period.
- [ ]  Integration of NMPC (Nonlinear Model Predictive Control) for aggressive tracking.
- [ ]  Migration from absolute paths to `ament_index` package paths.