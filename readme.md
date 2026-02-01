# Cartpole Simulation (ROS 2 + Gazebo)

A lightweight cartpole simulation for ROS 2. This project demonstrates how to simulate a robot in Gazebo and interact with it by applying external forces directly to specific links.

## Prerequisites

* **OS:** Ubuntu 24.04 (Noble) or 22.04 (Jammy)
* **ROS 2:** Jazzy Jalisco or Humble Hawksbill
* **Gazebo:** Harmonic or Fortress
* **Dependencies:** `ros-gz` bridge and interfaces

### Install Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-ros-gz
sudo apt-get install ros-$ROS_DISTRO-ros-gz-interfaces

```

## Build Instructions

1. **Navigate to your workspace:**
```bash
cd ~/cartpole_ws

```


2. **Build the package:**
```bash
colcon build --symlink-install

```


3. **Source the environment:**
```bash
source install/setup.bash

```



## How to Run the Simulation

To run this simulation correctly, we must follow a specific order.

### Step 1: Open the Custom World

**Command:**

```bash
gz sim -r $(ros2 pkg prefix cartpole_sim)/share/cartpole_sim/urdf/world_with_force.sdf

```

**Why do we do this?**
We cannot just launch an empty Gazebo world. We must launch our custom `world_with_force.sdf` because it contains the **`ApplyLinkWrench` system plugin**. This plugin is what allows us to apply "magical" forces to the robot from the terminal later. If you skip this step, the force commands will simply fail.

### Step 2: Spawn the Robot (URDF)

Now that the world is ready, we inject our robot into it.

**Command:**

```bash
ros2 run ros_gz_sim create -name cartpole -file $(ros2 pkg prefix cartpole_sim)/share/cartpole_sim/urdf/cartpole.urdf -z 1.0

```

* **Note:** We use `-z 1.0` to spawn the robot 1 meter in the air. This aligns with the "fixed base" we defined in the URDF, preventing the pole from clipping through the floor.

## How to Interact (Apply Force)

Since we are not using a GUI script right now, we can interact with the robot by sending direct messages to the Gazebo physics engine.

**Push the Cart (Force X = -5000)**
Run this command in a new terminal to give the cart a strong shove:

```bash
gz topic -t "/world/force_demo_world/wrench" -m gz.msgs.EntityWrench -p 'entity { name: "cartpole::cart" type: LINK } wrench { force { x: -5000 } }'

```

**Push the Other Way (Force X = 5000)**

```bash
gz topic -t "/world/force_demo_world/wrench" -m gz.msgs.EntityWrench -p 'entity { name: "cartpole::cart" type: LINK } wrench { force { x: 5000 } }'

```

### Troubleshooting

* **Robot doesn't move?** Check if the simulation is **Paused** (look for the Play button in the bottom-left).
* **Error `Entity not found`?** Ensure you spawned the robot *after* the world loaded.