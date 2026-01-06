# Unitree Go2 Native 2D SLAM & Nav2 Navigation (ROS 2 Jazzy) 🤖


[中文说明 (README_zh.md)](./README_Zh.md)

## 1. 🎯 Project Purpose

This repository provides a practical workflow to run **2D SLAM mapping** and **Nav2 navigation** on a **native Unitree Go2 EDU** robot using:

- Ubuntu 24.04 
- ROS 2 Jazzy
- Unitree_ROS2 (official Unitree ROS 2 interface)
- slam-toolbox (2D SLAM)
- Nav2 (Navigation2 stack)

It also includes keyboard teleoperation and Go2 pose/attitude visualization.


## 2. 🧰 Hardware&Environment

- Robot: **Unitree Go2 EDU**
- OS: **Ubuntu 24.04**
- ROS 2: **Jazzy**



## 3. 🧩 Software Stack

- Unitree_ROS2
- slam-toolbox
- Nav2 (nav2 / nav2_bringup)



## 4. ✅ Implemented Features

1. Keyboard teleoperation for Go2
2. Go2 pose/attitude visualization (TF / robot state visualization)
3. 2D mapping with **slam-toolbox**
4. Navigation with **Nav2**


## 5. 🛠️ Setup

### 5.1 📦 Install Dependencies

> Package names below assume Ubuntu 24.04 + ROS 2 Jazzy installed via apt.
> If you already installed ROS 2 Jazzy Desktop, you can skip installing it again.

**Required tools**
- git
- rosdep
- colcon build extensions

**SLAM + Navigation**
- slam-toolbox
- navigation2
- nav2-bringup

**Teleop**
- teleop-twist-keyboard

**Install:**
```bash
sudo apt update
sudo apt install -y \
  git \
  python3-rosdep \
  python3-colcon-common-extensions

sudo rosdep init || true
rosdep update

# Recommended full desktop install (optional if already installed)
sudo apt install -y ros-jazzy-desktop

# SLAM + Nav2 + teleop
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard
```

### 5.2 📥 Clone This Repository

```bash
cd ~
git clone https://github.com/Jumbo213/go2_slam_nav2.git
```


### 5.3 🏗️ Build (colcon)

```bash
cd ~/go2_slam_nav2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```


## 6. 🚀 Usage

### 6.1 ⚠️ Important: Source Environments in Every Terminal

For **each terminal** you open, run:

```bash
cd ~/go2_slam_nav2
source ~/unitree_ros2/setup.sh
source ~/go2_slam_nav2/install/setup.bash
```

> Notes: 📝
>
> * `~/unitree_ros2/setup.sh` is assumed to be the Unitree_ROS2 environment script path.
> * If your Unitree_ROS2 path differs, update it accordingly.


### 6.2 Terminal 1 — Keyboard Control Go2

> Key mappings follow the official `teleop_twist_keyboard` prompts.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 6.3 Terminal 2 — Start Go2 Bringup + Pose Visualization + SLAM Integration

Open a new terminal, source environments (Section 6.1), then run:

```bash
ros2 launch go2_core go2_start.launch.py
```

### 6.4 Terminal 3 — Start Nav2 Navigation

Open another new terminal, source environments (Section 6.1), then run:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/go2_slam_nav2/config/nav2_params.yaml
```


## 7. ⚙️ Configuration

* Nav2 parameters file:

  * `~/go2_slam_nav2/config/nav2_params.yaml`

Ensure the following match your real robot setup:

* Laser scan topic name
* Frame IDs (e.g., `base_link`, `odom`, `map`, and laser frame)
* TF tree is valid for navigation (commonly `map -> odom -> base_link`)


## 8. 🙏 Acknowledgements

This project is based on and inspired by:

* [https://github.com/FishPlusDragon/unitree-go2-slam-toolbox](https://github.com/FishPlusDragon/unitree-go2-slam-toolbox) 🌟

Many thanks to the original author and contributors for providing a solid baseline integration and a helpful reference implementation🤝. This repository reorganizes the workflow and extends it into a practical **slam-toolbox + Nav2** pipeline on **ROS 2 Jazzy** for **Unitree Go2 EDU**.
