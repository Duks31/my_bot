## My Bot

https://github.com/user-attachments/assets/5554a42e-961d-457e-9b44-4f33979c5dec

Diff Drive Robot Using ROS2, Gazebo, Rviz2 and Fusion 360

### Description
This repository contains the ROS2 implementation for a differential drive robot. The package includes nodes for controlling the robot, simulating its motion. 

### System Requirements
- Ubuntu 22.04
- \>= ROS2 Humble 
- Python 3.XX

### Repository Structure
```bash
my_bot
├── CMakeLists.txt
├── LICENSE.md
├── README.md
├── config
│   ├── drive_bot.rviz
│   ├── empty.yaml
│   ├── my_controllers.yaml
│   └── view_bot.rviz
├── description
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── robot.urdf.xacro
│   ├── robot_core.xacro
│   └── ros2_control.xacro
├── launch
│   ├── launch_sim.launch.py
│   └── rsp.launch.py
├── log
│   ├── COLCON_IGNORE
│   ├── latest -> latest_list
│   ├── latest_list -> list_2024-11-25_17-22-41
│   └── list_2024-11-25_17-22-41
│       └── logger_all.log
├── package.xml
└── worlds
    ├── empty.world
    └── obstacles.world
```

### Installation and Build Instructions
1. Make a ros2 workspace and initialize it
```bash
   mkdir -p ~/my_bot_ws/src
   cd ~/my_bot_ws
   colcon build
   source install/setup.bash
```

2. Clone the repository
```bash
    cd ~/my_bot_ws/src
    git clone https://github.com/Duks31/my_bot
```

3. Navigate to the workspace and build the package
```bash
    cd ~/my_bot_ws
    colcon build
```

4. Install dependencies and build the workspace
```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    source install/setup.bash
```

5. Run the simulation
```bash
    ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
```

6. Remap and run the teleop node
```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

7. Visualize the robot in RViz2 (Optional)
    - Open a new terminal and run the following command
    ```bash
    Rviz2
    ```
    - In RViz2, set the Fixed Frame to `odom` and add the RobotModel to visualize the robot.
    - Add tf and RobotModel to visualize the robot in RViz2.

### Robot Description (URDF)

![diff_drive v4](https://github.com/user-attachments/assets/4ef3cf61-0392-4404-9eaf-3be592056547)

The robot's URDF (Unified Robot Description Format) file was initially designed using **Autodesk Fusion 360**. The 3D model of the robot was created in Fusion 360, which provided an accurate representation of the robot's physical dimensions, joints, and links. 

1. **Modeling in Fusion 360**:
   - The robot's physical design, including the chassis, wheels was created in Fusion 360.
   - The 3D model was exported as a URDF file for integration into ROS.

2. **Conversion to URDF**:
   - The exported model was processed using [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) into a URDF file.
   - Additional features like collision and visual properties were added to make the robot compatible with Gazebo simulation and ROS visualization tools (e.g., RViz2).

### Nodes and Launch Files

#### Nodes
1. **`robot_state_publisher`**
   - Publishes the robot's URDF to the `/robot_description` topic.
   - **Key Parameters**:
     - `robot_description`: Processed URDF description of the robot.
     - `use_sim_time`: Enables simulated time for Gazebo.

2. **`spawn_entity.py`**
   - Spawns the robot into the Gazebo simulation.
   - **Arguments**:
     - `-topic robot_description`: Specifies the URDF topic.
     - `-entity my_bot`: Unique name for the robot in the simulation.

3. **`diff_drive_controller/DiffDriveController`**
   - Controls the differential drive system by translating velocity commands to wheel motions.
   - **Key Parameters**:
     - `wheel_separation`: 0.35 m
     - `wheel_radius`: 0.05 m

4. **`joint_state_broadcaster/JointStateBroadcaster`**
   - Publishes joint states (position, velocity) to the `/joint_states` topic.

---

#### Launch Files

1. **`launch_sim.launch.py`**
   - Launches the simulation with the following components:
     - **Robot State Publisher**: Publishes the robot description.
     - **Gazebo**: Starts the simulation environment.
     - **Robot Spawner**: Adds the robot to the simulation.
     - **Controller Spawners**: Launches the `diff_drive_controller` and `joint_state_broadcaster`.
   - **Command**:
     ```bash
     ros2 launch my_bot launch_sim.launch.py
     ```

2. **`rsp.launch.py`**
   - Configures and launches the `robot_state_publisher` node.
   - Processes the `robot.urdf.xacro` file.
   - **Command**:
     ```bash
     ros2 launch my_bot rsp.launch.py use_sim_time:=true
     ```

---

### Worlds
- **obstacles.world**: This world file is used to simulate the robot in a world with obstacles.

### Acknowledgments
- Thanks to https://www.youtube.com/@ArticulatedRobotics
