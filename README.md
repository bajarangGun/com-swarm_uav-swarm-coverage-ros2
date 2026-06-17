# Multi-UAV CoM-Based Coverage Planning in ROS 2

This repository contains a centralized, mathematically optimized coverage planning algorithm for UAV swarms, built for **ROS 2** and tested via **ArduPilot SITL**. 

## Academic Attribution & Novel Contributions
This project builds upon the foundational Grid-based Coverage Planning theories presented in the paper: 
**[Online Concurrent Multi-Robot Coverage Path Planning (Ratijit Mitra and Indranil Saha)](https://arxiv.org/abs/2403.10460)**.

While the core Hungarian assignment and BFS routing mechanics are derived from the original paper, **this repository introduces the following novel contributions and engineering integrations:**
* **Center of Mass (CoM) Elastic Tethering:** A mathematical penalty system that prevents trajectories from stretching the swarm's RF communication threshold beyond physical hardware limits.
* **Smart Network Recovery:** Autonomously parks leading drones to allow trailing agents to catch up, dynamically healing the network topology.
* **Deadlock Aversion (Cohesive Retreat):** Identifies geometric local minima (swarm paralysis) caused by hard constraints and forcefully injects 1-step cohesive retreat trajectories toward the swarm CoM to break the deadlock.
* **Full ROS 2 & ArduPilot Integration:** Ported the theoretical framework into a real-time, asynchronous C++ ROS 2 node architecture bridged with ArduPilot SITL flight physics.

---

## Simulation Results

Extensive stress-testing was conducted on an Ubuntu 24.04 environment using 3 simulated UAVs. The system demonstrated exceptional algorithmic efficiency, consistently maintaining less than **0.15% computational overhead**.

### 1. Network Tension Boundary Test
*Tested on a 15x15 Grid. Proves the Elastic Tether correctly aborts the mission via Cohesive Retreat Livelock when physical RF constraints are mathematically unachievable.*

| Comm Range (Tether) | Mission Status | Failure Mode |
| :--- | :--- | :--- |
| **6.0 Cells** | **100% Success** | N/A |
| **5.0 / 4.0 / 3.0 Cells** | **Mission Abort** | Algorithmic Livelock (Boundary Reached) |

### 2. Spatial Scaling (Baseline)
*Tested with a constant 6.0 communication range and standard corner-cluster starting position.*

| Grid Size | Total Area | Coverage | Total Comp Time (TCT) | Mission Time (MT) | Algorithmic Overhead |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 10 x 10 | 100 Cells | 100% | 0.035 sec | 72.09 sec | 0.05% |
| 15 x 15 | 225 Cells | 100% | 0.101 sec | 150.02 sec | 0.06% |
| 20 x 20 | 400 Cells | 100% | 0.293 sec | 288.00 sec | 0.10% |
| 25 x 25 | 625 Cells | 100% | 0.542 sec | 432.04 sec | 0.13% |

### 3. Initial Position Dynamics & Self-Healing
*Tested on a 20x20 grid. The "Disconnected Start" placed drones at opposite corners of the map, massively violating the 6.0 tether. The algorithm successfully suspended coverage, forced the drones to fly inward to establish a network connection, and then executed the sweep.*

| Starting Configuration | Coordinates | TCT | Mission Time (MT) | Note |
| :--- | :--- | :--- | :--- | :--- |
| **Corner Cluster** | `0,0; 1,0; 2,0` | 0.293 sec | 288.00 sec | Baseline sweep |
| **Disconnected Start** | `0,0; 0,19; 19,0` | 0.436 sec | 321.04 sec | Includes ~33s Network Healing Tax |
| **Center drop** | `10,10; 10,11; 11,10` | N/A | N/A | **Mission Abort:** Algorithmic Livelock (Boundary Reached) |

---

## System Setup & Installation Guide

### Prerequisites
To run this simulation, your system must have the following installed:
* **OS:** Ubuntu 24.04
* **Middleware:** ROS 2
* **Flight Stack:** ArduPilot SITL & MAVROS
* **Build Tools:** `colcon`

### Step 1: Create the Workspace & Clone
Open your terminal and create a dedicated ROS 2 workspace:
```bash
mkdir -p ~/concpp_ws/src
cd ~/concpp_ws/src
git clone [https://github.com/bajarangGun/com-swarm_uav-swarm-coverage-ros2.git](https://github.com/bajarangGun/com-swarm_uav-swarm-coverage-ros2.git) con_plan_exec_pkg
```

### Step 2: Build the Package
Navigate to the root of your workspace and compile the package:
```bash
cd ~/concpp_ws
colcon build --packages-select con_plan_exec_pkg --symlink-install
```
### Step 3: Initialize ArduPilot SITL & MAVROS
Before launching the central brain, you must have your simulated drone environment running.

Initialize three separate ArduPilot SITL vehicle instances (representing drone0, drone1, and drone2).

Ensure your ROS 2 bridge (e.g., MAVROS) is running and actively connecting the SITL instances to the ROS 2 network so the central planner can publish trajectory commands.

### Launching the Simulation & Visualization
Source your workspace and launch the swarm central brain. This command configures a 20x20 grid with 3 drones and a 6.0 cell communication threshold:
```bash
source ~/concpp_ws/install/setup.bash
ros2 launch con_plan_exec_pkg swarm.launch.py rc:=3 ws_x:=20 ws_y:=20 drone_locs:="0,0; 1,0; 2,0" comm_range:=6.0 cell_size:=5.0 takeoff_alt:=10.0
```

###  Visualize in RViz2
To watch the algorithm dynamically assign paths and monitor the drones in real-time, open a new terminal tab, source the workspace again, and launch RViz:
```bash
source ~/concpp_ws/install/setup.bash
rviz2
```
## Authors

**Course Project — UAV Communications (EE-798T)** **Professor:** Prof. Ketan Rajawat

**Group Members**   

1. Md Tahseen Aslam (251010069)
2. Divyansh singh (251010064) 

---

