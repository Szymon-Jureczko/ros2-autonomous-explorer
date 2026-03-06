# Autonomous Frontier Explorer

A fully autonomous frontier-based exploration system built with **ROS 2 Jazzy**, **Gazebo Harmonic**, **Nav2**, and **SLAM Toolbox**. The robot autonomously maps an unknown environment by detecting and navigating to frontiers — the boundaries between explored and unexplored space — until the entire reachable area is mapped.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Docker](https://img.shields.io/badge/Docker-containerized-2496ED?logo=docker&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

## Overview

https://github.com/user-attachments/assets/02ae595e-0e4e-4193-83b2-f0638b869e9c

The system drops a differential-drive robot into a **20×15 m five-room apartment** in Gazebo and lets it explore completely on its own. No teleoperation, no pre-built map, no waypoints — just `docker compose up` and watch.

The entire stack — ROS 2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox, and a full desktop GUI — runs inside a **Docker container**. A built-in noVNC server makes Gazebo and RViz2 accessible from any web browser with zero local installation. This makes the project fully portable: it runs identically on any Linux, macOS, or Windows machine that has Docker.

**Key capabilities:**
- Zero-install dev environment via Docker + noVNC browser desktop
- BFS-based frontier detection on the live SLAM occupancy grid
- Frontier scoring by size-to-distance ratio (prefers large, nearby frontiers)
- 60-second navigation timeout with automatic goal cancellation
- Goal blacklisting to avoid repeatedly targeting unreachable areas
- Recovery spins when no frontiers are immediately visible
- Initial 360° spin to seed the SLAM map before exploration begins

## Architecture

```
┌──────────────┐    /scan     ┌──────────────┐   /map    ┌──────────────────┐
│   Gazebo     │─────────────▶│ SLAM Toolbox │─────────▶│ Frontier Explorer│
│  Harmonic    │    /odom     │ (online async)│          │   (Python node)  │
│              │──┐  /tf      └──────────────┘          └────────┬─────────┘
└──────────────┘  │                                              │
       ▲          │                                    NavigateToPose
       │          │           ┌──────────────┐                   │
  /cmd_vel        └──────────▶│    Nav2       │◀──────────────────┘
       │                      │  (7 nodes)   │
       │                      └──────┬───────┘
       │                             │
       │    ┌────────────────────────┘
       │    │  cmd_vel chain:
       │    │  controller_server → /cmd_vel_nav
       │    │  → velocity_smoother → /cmd_vel_smoothed
       │    │  → collision_monitor → /cmd_vel
       └────┘
```

**Launch sequence with timed delays:**

| Delay | Component | Purpose |
|-------|-----------|---------|
| 0s | Gazebo | Start simulation |
| 0s | Static TF publishers | chassis→lidar, base_link→chassis, base_link→base_footprint |
| 5s | ros_gz_bridge | Bridge /cmd_vel, /scan, /odom, /tf, /clock |
| 10s | SLAM Toolbox | Online async mapping (needs /scan + /tf) |
| 20s | Nav2 (7 nodes) | Navigation stack (needs map→odom TF from SLAM) |
| 25s | RViz2 | Visualization |
| 40s | Frontier Explorer | Autonomous exploration (needs Nav2 active) |

## Technologies

| Component | Technology | Details |
|-----------|-----------|---------|
| **Middleware** | ROS 2 Jazzy | Latest LTS release |
| **Simulator** | Gazebo Harmonic (gz-sim-8) | ogre2 rendering, gpu_lidar sensor |
| **SLAM** | SLAM Toolbox | Online async mode, 0.05 m resolution |
| **Navigation** | Nav2 | MPPI controller, NavFn planner (allow_unknown: true) |
| **Language** | Python 3 | Single frontier explorer node (~460 lines) |
| **Bridge** | ros_gz_bridge | Bidirectional Gazebo ↔ ROS 2 transport |
| **Container** | Docker + docker-compose | Dev and CI services |
| **Desktop** | TigerVNC + noVNC + Fluxbox | Browser-accessible GUI on port 6080 |

## Containerization

The project is fully containerized — **no local ROS installation is required**. The `Dockerfile` builds on `osrf/ros:jazzy-desktop` and layers in:

- All Nav2, Gazebo, SLAM Toolbox and bridge packages
- A **TigerVNC** server (port `5901`) for a remote desktop
- A **noVNC** web proxy (port `6080`) so the desktop is accessible from any browser
- An **XFCE4** window manager
- A `start-vnc.sh` startup script that wires everything together

`docker-compose.yml` defines two services:

| Service | Purpose | GUI |
|---------|---------|-----|
| `dev` | Interactive development — workspace mounted as a volume, VNC desktop started automatically | noVNC on port 6080 |
| `ci` | Headless CI/testing — builds the workspace and launches in `headless:=true rviz:=false` mode | None |

```
 Host machine
 └─── Docker container (ros2_dev)
      ├── XFCE4 desktop (DISPLAY :1)
      │    ├── Gazebo Harmonic window
      │    └── RViz2 window
      ├── TigerVNC server :5901  ──────── native VNC client
      └── noVNC websocket :6080  ──────── http://localhost:6080/vnc.html
```

Because Gazebo's ogre2 renderer needs OpenGL, the launch file applies Mesa software-rendering environment variables scoped only to RViz2, while Gazebo uses Mesa's ogre2 backend directly — keeping them compatible without a GPU.

---

**Nav2 nodes (minimal set — no unused servers):**
- `controller_server` (MPPI)
- `planner_server` (NavFn)
- `behavior_server` (spin, backup, wait)
- `smoother_server`
- `velocity_smoother`
- `collision_monitor`
- `bt_navigator`

## Running

There are two ways to run this project: **inside the provided dev container** (recommended — no install required) or **natively on Ubuntu 24.04**.

---

### Option 1 — Dev Container + noVNC (recommended)

The repo ships a `Dockerfile` and `docker-compose.yml` that provide a complete ROS 2 Jazzy + Gazebo + Nav2 environment with a **noVNC desktop** for GUI access. No local ROS installation needed.

#### Start the container

```bash
git clone https://github.com/Szymon-Jureczko/ros2-autonomous-explorer.git
cd ros2-autonomous-explorer
docker compose build dev
docker compose up -d dev
```

> **Note:** Use `docker compose up -d dev` (detached mode). Running without `-d` launches
> an interactive TUI that captures keypresses and prevents typing in the terminal.

This starts:
- A **TigerVNC** server on port `5901` (display `:1`)
- A **noVNC** web client on port `6080`
- An **XFCE4** desktop environment

#### Open the desktop

Open your browser and go to:

```
http://localhost:6080/vnc.html
```

Click **Connect**. When prompted, enter the VNC password.

> **VNC password:** `password`
>
> You can also connect a native VNC client to `localhost:5901`.

> **Port conflict:** If port `6080` is already in use (e.g. by a VS Code dev container),
> edit `docker-compose.yml` and change `"6080:6080"` to e.g. `"7080:6080"`, then access
> noVNC at `http://localhost:7080/vnc.html`.

#### Build and launch

The recommended way is `docker exec` from your host terminal — you get full copy/paste
and don't need to use the noVNC desktop terminal:

```bash
docker exec -it ros2_dev bash
```

Then inside the container:

```bash
cd /workspaces/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select autonomous_explorer
source install/setup.bash
ros2 launch autonomous_explorer auto_explore.launch.py
```

Gazebo and RViz2 windows will appear on the noVNC desktop.

#### Headless CI mode (no GUI)

```bash
docker compose up ci
```

This builds and launches with `headless:=true rviz:=false` automatically.

---

### Option 2 — Native Ubuntu 24.04

#### Prerequisites

- **ROS 2 Jazzy** (desktop install)
- **Gazebo Harmonic**, **Nav2**, **SLAM Toolbox**, **ros_gz_bridge**

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-tf2-ros
```

#### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Szymon-Jureczko/ros2-autonomous-explorer.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select autonomous_explorer
source install/setup.bash
```

#### Launch

**Full stack (Gazebo GUI + RViz2):**
```bash
ros2 launch autonomous_explorer auto_explore.launch.py
```

**Headless (no Gazebo GUI, keep RViz2):**
```bash
ros2 launch autonomous_explorer auto_explore.launch.py headless:=true
```

**Fully headless (CI / SSH):**
```bash
ros2 launch autonomous_explorer auto_explore.launch.py headless:=true rviz:=false
```

---

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | Run Gazebo in server-only mode (no GUI) |
| `rviz` | `true` | Launch RViz2 for visualization |

### What to Expect

1. Gazebo opens with the apartment world
2. After a few seconds the robot begins a 360° spin to seed the SLAM map
3. The frontier explorer starts detecting and navigating to frontiers
4. The map progressively fills in as the robot visits each room
5. When no frontiers remain for 5 consecutive ticks, exploration is declared complete

## Project Structure

```
autonomous_explorer/
├── Dockerfile                        # Multi-stage image (base → dev → production)
├── docker-compose.yml                # dev (noVNC GUI) and ci (headless) services
├── CMakeLists.txt                    # Build config (Python-only install)
├── package.xml                       # ROS 2 package manifest
├── README.md
├── config/
│   ├── nav2_explore_params.yaml      # Nav2 parameters (MPPI, NavFn, costmaps)
│   ├── slam_params.yaml              # SLAM Toolbox online async config
│   └── explore_rviz.rviz             # RViz2 display configuration
├── launch/
│   └── auto_explore.launch.py        # Full-stack launch with timed delays
├── src/
│   └── frontier_explorer.py          # Autonomous frontier exploration node
└── worlds/
    └── robot_world.sdf               # 20×15m apartment + diff-drive robot
```

## How the Frontier Explorer Works

1. **Clock health check** — Waits for `/clock` messages to confirm sim-time is active before connecting to Nav2
2. **Initial spin** — Sends a 360° `Spin` action to Nav2's behavior server, seeding the SLAM map with a full LIDAR sweep
3. **Frontier detection** — Every `replan_interval` seconds (default 5s):
   - Reads the SLAM occupancy grid
   - Finds all FREE cells adjacent to UNKNOWN cells (frontier cells)
   - Clusters frontier cells via BFS into connected components
   - Filters out clusters smaller than `min_frontier_size` (default 5 cells)
4. **Frontier selection** — Scores each frontier by `cluster_size / distance_to_centroid`, picks the highest. Skips blacklisted goals
5. **Navigation** — Sends a `NavigateToPose` goal oriented toward the frontier
6. **Timeout & blacklisting** — If navigation takes >60 seconds, the goal is cancelled and blacklisted (up to 50 entries)
7. **Recovery** — If no frontiers are found, a 180° recovery spin is triggered. After 5 consecutive ticks with no frontiers (and at least 2 goals completed), exploration is declared complete

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_frontier_size` | `5` | Minimum cells in a frontier cluster to be considered |
| `replan_interval` | `5.0` | Seconds between exploration ticks |
| `blacklist_radius` | `1.0` | Radius (m) around blacklisted goals to reject |

## The Simulation World

The Gazebo world is a **20×15 m apartment** with 5 interconnected rooms:

- Two **bedrooms** (top-left, top-center)
- **Kitchen** (top-right)
- **Living room** (bottom-left, large open area)
- **Bathroom** (bottom-right)

All rooms are connected by doorways. The robot starts in the living room at `(-5, -3.5)` facing north.

**Robot specs:**
- Differential drive (wheel separation: 0.4 m, wheel radius: 0.1 m)
- 360° LIDAR (gpu_lidar): 360 samples, 8 m range, 10 Hz, min range 0.20 m
- LIDAR mounted at z=0.3 m on the chassis
- Robot radius: 0.22 m (for Nav2 footprint)

## Notes

- **Software rendering:** The launch file sets Mesa/OpenGL environment variables on RViz2 for dev containers without a GPU. If you have a GPU, these are harmless but unnecessary.
- **Map reset:** To reset the SLAM map without killing everything: `ros2 service call /slam_toolbox/reset std_srvs/srv/Empty`
- **Velocity chain:** The command velocity passes through three stages — controller → velocity_smoother → collision_monitor — before reaching the robot, ensuring smooth and safe motion.

## License

MIT
