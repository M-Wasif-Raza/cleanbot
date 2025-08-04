# CleanBot
**CleanBot – An Autonomous Indoor Cleaning Robot**  

This repository contains the ROS2 Humble implementation of CleanBot: a modular and fully autonomous indoor cleaning robot capable of mapping unknown environments, navigating to specified rooms, and performing coverage path planning on selected area.

📊 For complete overview please visit [here](https://www.canva.com/design/DAGnozmqtOQ/OSGvIO3hyjnvcSo0nQEhfw/watch?utm_content=DAGnozmqtOQ&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hb44c85889c)


---

## 📦 Dependencies

CleanBot uses the following ROS2 packages:

- **[Navigation2 (Nav2)](https://navigation.ros.org)** – For autonomous navigation, path planning, and control.  
- **[Open Navigation's Nav2 Complete Coverage](https://github.com/open-navigation/opennav_coverage)** – For coverage path planning (Boustrophedon and Dubins paths).  
- **[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)** – For 2D mapping using LiDAR.  
- **[SLAM-Frontier-Exploration](https://github.com/gjcliff/SLAM-Frontier-Exploration)** – For exploration of unknown environments in autonomous mapping mode.  

---

## 🚀 Installation

Make sure ROS2 Humble is already installed and sourced in your system before proceeding. You can check the official [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

### 1️⃣ Install ROS2 dependencies from apt

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-turtlebot3-simulations
```

### 2️⃣ Set up your ROS2 workspace
Create a new workspace for CleanBot:

```bash
mkdir -p ~/cleanbot_ws/src
cd ~/cleanbot_ws/src
```

### 4️⃣ Clone the required repositories

```bash
# Clone the main CleanBot repository
git clone https://github.com/M-Wasif-Raza/cleanbot.git

# Clone Nav2 Complete Coverage
git clone -b humble https://github.com/open-navigation/opennav_coverage.git

# Clone SLAM Frontier Exploration
git clone https://github.com/gjcliff/SLAM-Frontier-Exploration.git
```


### 5️⃣ Install Fields2Cover (v1.2.1)

Clone the Fields2Cover repository (required by **[opennav_coverage](https://github.com/OpenNavigation/opennav_coverage)**) and checkout the specific tag:

```bash
git clone https://github.com/Fields2Cover/Fields2Cover.git
cd Fields2Cover
git fetch --tags
git checkout tags/v1.2.1
```

### 6️⃣ Install rosdep dependencies
Use rosdep to install any missing dependencies:

```bash
cd ~/cleanbot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 7️⃣ Build & source the workspace

```bash
colcon build --symlink-install
```

### 8️⃣ Source the workspace
```bash
source ~/cleanbot_ws/install/setup.bash
```
---

## 🚀 Quick Start for Gazebo Classic Simulation

1. **Export the Gazebo Models Path:**
   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models 
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/cleanbot_ws/src/mts_department/models
   ```
2. **Launch the Coverage Planning Simulation:**
   ```bash
   ros2 launch cleanbot coverage_planning.launch.py 
   ```

### 🗺️ For Autonomous Mapping Simulation:

   ```bash
   ros2 launch cleanbot automapping.launch.py 
   ```

---

## Package: `mts_department`

The `mts_department` package provides a **Gazebo world** representing the **Department of Mechatronics Engineering, NUST CEME**.  

This custom environment allows CleanBot to perform autonomous navigation, mapping, and area-wise cleaning in a realistic indoor setting inspired by the actual department layout.  

---

### 📸 Gazebo World Overview

<p align="center">
  <img src="media/mts_model.gif" alt="Gazebo World" width="600"/>
</p>

<!-- ## 📚 Documentation

For detailed documentation on usage & configuration, please refer to the [CleanBot Wiki](https://github.com/M-Wasif-Raza/cleanbot.git/wiki). -->

## 👥 Acknowledgements
This project is a result of collective efforts of the Final Year Project (FYP) team:  

- [**Muhammad Wasif Raza**](https://www.linkedin.com/in/muhammad-wasif-raza-a86b4b24b/?originalSubdomain=pk)  
- [**Rameel Ahmed**](https://pk.linkedin.com/in/rameelahmed)  
- [**Noman Shabbir**](https://pk.linkedin.com/in/engr-nomanshabbir)  
- [**Abdul Moiz**](https://pk.linkedin.com/in/abdul-moiz-43062b256)

