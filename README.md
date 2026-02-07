# PX4 & ROS 2 Drone Pursuit (Tracer Project)

This project enables a drone (Raspberry Pi 5 + Pixhawk 6C) to autonomously track and pursue a "Chaos Car" using **Image-Based Visual Servoing (IBVS)**. It is designed for GPS-denied environments, relying on optical flow and altitude hold logic.

## 🚀 Key Features

- **Dual-Mode Control:** Seamlessly toggle between Manual Teleop and Autonomous Pursuit via keyboard.
- **Perception-Aware IBVS:** A custom trapezoidal "Safe Zone" accounts for the 20° camera tilt, preventing control "hunting."
- **GPS-Denied Offboard Control:** Commands the drone via Body-Frame Velocity/Attitude setpoints.
- **Target Re-acquisition:** Includes a 1-second "coasting" logic to maintain lock during temporary occlusions.

---

## 📂 Package Structure

Plaintext

`tracer/
├── launch/             # Python launch files for Gazebo & RViz
├── meshes/             # STL and XML model files for drone & camera
├── urdf/               # Robot description files (tracer.urdf)
├── world/              # Gazebo environment (tracer.sdf)
├── tracer/             # Python source code (Detector, Controller, Driver)
├── rviz/               # Pre-configured RViz2 layout
├── package.xml         # Dependencies and metadata
└── setup.py            # Package installation and data_file mapping`

---

## 🛠 Setup & Installation

### 1. External Mesh Download

The simulation requires a specific vehicle mesh that is too large for the repository. **You must download this before building.**

- **Download Mesh:** [Google Drive Link](https://drive.google.com/file/d/1YeTOY_UW7MulZmMu2iMyZAabGPCXxCoq/view?usp=sharing)
- **Installation:** Place the downloaded files into the `meshes/` directory.

### 2. Fixing File Paths (Crucial)

The current URDF and Launch files use absolute path references specific to the original environment. To run the simulation on your machine, you must:

1. Open `urdf/tracer.urdf` and `launch/gazebo.launch.py`.
2. Search for hardcoded paths (e.g., `/home/deen/...`).
3. **Correction:** The package is configured to use `ament_index`. Replace absolute paths with the package share directory logic or use the `package://tracer/` URI in the URDF.

### 3. Build the Package

Bash

`cd ~/ros2_ws
colcon build --packages-select tracer --symlink-install
source install/setup.bash`

---

## 🎮 How to Run

### 1. Launch the Simulation

Bash

`ros2 launch tracer gazebo.launch.py`

### 2. Run the Pursuit Stack

In separate terminals:

Bash

`# Start the Vision Detector
ros2 run tracer detector

# Start the Controller (Manual/Auto Agent)
ros2 run tracer drone_controller`

---

## 🕹 Controls & Logic

### Mode Selection

- **`m` Key:** Toggles between **MANUAL** and **AUTO**.

### Manual Mode

- **`w / x`**: Forward / Backward
- **`a / d`**: Left / Right
- **`r / f`**: Up / Down
- **`q / e`**: Yaw Left / Right
- **`s`**: Emergency Stop (Zero Velocities)

### Auto Mode (Pursuit Logic)

The `detector.py` node calculates error based on a **Trapezoidal Safe Zone**.

- **Horizontal Error ($Yaw$):** Scaled -1.0 to 1.0 based on screen center.
- **Vertical Error ($V_x$):** Scaled -1.0 to 1.0 (Top = Far, Bottom = Near).
- **Coasting:** If the target is lost, the system stays in "Target Found" state ($z=1.0$) for **1 second** using the last known position to prevent disengagement.

---

## ⚠️ Known Issues

- **Absolute Paths:** Ensure all mesh references in `tracer.urdf` are updated to your local path or converted to package-relative paths.
- **Pi 5 Performance:** If running on hardware, ensure `cv2.imshow` is disabled in `detector.py` to save CPU cycles.