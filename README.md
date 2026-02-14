## Step 1: Start Simulation Backend (Terminal 1)
This launches **Gazebo**, the **PX4 Autopilot**, the **ROS-Gazebo Bridge**, and **MAVROS**.

### Action
1. Open a new terminal.
2. Navigate to Home and run the start script.

```bash
cd ~
./start_sim.sh
```

> [!IMPORTANT]
> **Check:** Wait until you see `"SIMULATION READY"` or the Gazebo window appears before proceeding.

---

## Step 2: Start QGroundControl (Terminal 2)
Launch the ground control station to arm the drone, monitor status, and control flight modes.

### Action
1. Open a new terminal.
2. Navigate to the location of your AppImage (usually the Downloads folder).
3. Run the application.

```bash
cd ~/Downloads
./QGroundControl-x86_64.AppImage
```

---

## Step 3: Start Localization (Terminal 3)
This launches **RTAB-Map** in "Localization Mode" (read-only). It loads the existing map into RAM and attempts to localize the drone within it.

### Action
1. Open a new terminal.
2. Navigate to your workspace and source the environment.
3. Launch the localization node.

```bash
cd ~/Documents/mint_Drone/rtabmap
source install/setup.bash
ros2 launch midcone_navigation localization.launch.py
```

* **Note:** Wait for the log message confirming the database has been loaded. The **RViz** window should appear automatically.

---

## Step 4: Monitor Position (Terminal 4)
Use this terminal to view the drone's real-time coordinates relative to the map.

### Option A: Absolute Map Coordinates (Recommended)
Shows the exact **X, Y, Z** of the `base_link` relative to the map origin. This includes map corrections.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

### Option B: Localization Events
Shows position updates only when RTAB-Map confirms a specific localization match.

```bash
ros2 topic echo /localization_pose
```