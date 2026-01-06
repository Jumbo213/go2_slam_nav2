# Unitree Go2 原生 2D SLAM 与 Nav2 导航（ROS 2 Jazzy）🤖

[English Version (README.md)](./README.md)

## 1. 🎯 项目作用

本仓库提供一套实用流程，用于在**原生 Unitree Go2 EDU** 机器狗上实现 **2D SLAM 建图**与 **Nav2 导航**，运行环境与组件如下：

- Ubuntu 24.04
- ROS 2 Jazzy
- Unitree_ROS2（Unitree 官方 ROS 2 接口）
- slam-toolbox（2D SLAM）
- Nav2（Navigation2 导航栈）

同时包含键盘遥控与 Go2 姿态/位姿可视化功能，便于调试与使用。

---

## 2. 🧰 硬件与环境

- 机器人：**Unitree Go2 EDU**
- 系统：**Ubuntu 24.04**
- ROS 2：**Jazzy**

---

## 3. 🧩 软件栈

- Unitree_ROS2
- slam-toolbox
- Nav2（nav2 / nav2_bringup）

---

## 4. ✅ 已实现功能

1. GO2 键盘遥控
2. GO2 姿态/位姿可视化（TF / robot state 可视化）
3. 基于 **slam-toolbox** 的 2D 建图
4. 基于 **Nav2** 的导航

---

## 5. 🛠️ 安装与编译

### 5.1 📦 安装依赖

> 以下包名假设你在 Ubuntu 24.04 上通过 apt 安装 ROS 2 Jazzy。
> 如果你已经安装了 ROS 2 Jazzy Desktop，可跳过重复安装。

**必需工具**
- git
- rosdep
- colcon 编译扩展

**SLAM + 导航**
- slam-toolbox
- navigation2
- nav2-bringup

**遥控**
- teleop-twist-keyboard

**安装命令：**
```bash
sudo apt update
sudo apt install -y \
  git \
  python3-rosdep \
  python3-colcon-common-extensions

sudo rosdep init || true
rosdep update

# 推荐安装完整桌面版（如果你尚未安装）
sudo apt install -y ros-jazzy-desktop

# SLAM + Nav2 + teleop
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard
```
### 5.2 📥 克隆仓库

```bash
cd ~
git clone https://github.com/Jumbo213/go2_slam_nav2.git
```

### 5.3 🏗️ 编译（colcon）

```bash
cd ~/go2_slam_nav2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

---

## 6. 🚀 使用方法

### 6.1 ⚠️ 重要：每个终端都需要加载环境变量

对于你打开的**每一个终端**，都执行：

```bash
cd ~/go2_slam_nav2
source ~/unitree_ros2/setup.sh
source ~/go2_slam_nav2/install/setup.bash
```

> 说明：
>
> * `~/unitree_ros2/setup.sh` 默认是 Unitree_ROS2 的环境脚本路径。
> * 如果你的 Unitree_ROS2 安装路径不同，请自行修改为实际路径。

### 6.2 终端 1 — 键盘控制 Go2

> 按键映射参考 `teleop_twist_keyboard` 的官方提示。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 6.3 终端 2 — 启动 Go2 Bringup + 姿态可视化 + SLAM 集成

另开一个终端，按 6.1 加载环境变量后执行：

```bash
ros2 launch go2_core go2_start.launch.py
```

### 6.4 终端 3 — 启动 Nav2 导航

再开一个终端，按 6.1 加载环境变量后执行：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/go2_slam_nav2/config/nav2_params.yaml
```

---

## 7. ⚙️ 配置

* Nav2 参数文件：

  * `~/go2_slam_nav2/config/nav2_params.yaml`

请确保以下内容与你的真实机器人配置一致：

* 激光雷达 scan 话题名称
* 各 Frame ID（例如 `base_link`、`odom`、`map`、以及雷达 frame）
* TF 树满足导航要求（常见为 `map -> odom -> base_link`）

---

## 8. 🙏 致谢

本项目基于并参考了以下开源项目：

* [https://github.com/FishPlusDragon/unitree-go2-slam-toolbox](https://github.com/FishPlusDragon/unitree-go2-slam-toolbox) 🌟

感谢原作者及所有贡献者提供了扎实的基础集成与清晰的参考实现🤝。本仓库在此基础上对流程进行了整理，并扩展为可直接使用的 **slam-toolbox + Nav2** 工作流，适用于 **ROS 2 Jazzy** 与 **Unitree Go2 EDU**。

