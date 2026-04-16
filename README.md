# TIAGo Mobile Manipulator — ROS 2 Simulation Workspace

A custom ROS 2 Jazzy simulation workspace for the **TIAGo mobile manipulator** from PAL Robotics. Combines PAL's official robot description packages (as git submodules) with custom packages for Gazebo Harmonic simulation, Nav2 navigation, SLAM, and a **MoveIt-free arm IK solver** with full-body coordination.

The robot navigates to a target, adjusts its torso height, solves inverse kinematics, checks for 3D collisions, and reaches with its arm — all controlled through ROS 2 action servers.

---

## System Environment

| Component | Details |
|-----------|---------|
| **OS** | Ubuntu 24.04.4 LTS (Noble Numbat) |
| **Kernel** | 6.17.0-20-generic |
| **Processor** | 12th Gen Intel Core i5-1240P (16 threads) |
| **RAM** | 8 GB DDR4 |
| **GPU** | NVIDIA GeForce RTX 3050 Mobile + Intel Alder Lake-P iGPU **(not used)** |
| **ROS 2** | Jazzy Jalisco |
| **Gazebo** | Gazebo Harmonic (gz-sim 8.10.0) |
| **Python** | 3.12.3 |

---

## Architecture Overview

The workspace is organised into three layers: **PAL submodules** (upstream robot description), a **simulation layer** (Gazebo world, controllers, sensor bridges), and **custom control packages** (IK, torso, orchestrator, collision detection).

```
                          ┌──────────────────────────────────┐
                          │        reach_target (orchestrator)│
                          │   /reach_target action server     │
                          └──────┬──────────┬──────────┬─────┘
                                 │          │          │
                    ┌────────────▼──┐  ┌────▼─────┐  ┌─▼──────────────┐
                    │  Nav2 Stack   │  │  Torso   │  │ Arm Controller │
                    │ /navigate_to_ │  │ /torso_  │  │ /arm_reach     │
                    │    pose       │  │  adjust  │  │ action server  │
                    └──────┬────────┘  └──────────┘  └───────┬────────┘
                           │                                 │
                           │                    ┌────────────▼────────────┐
                           │                    │  Collision Detector     │
                           │                    │  /check_collision srv   │
                           │                    │  (PointCloud2 → spheres)│
                           │                    │  (Not used now)         │
                           │                    └─────────────────────────┘
                           │
              ┌────────────▼─────────────────────────────────┐
              │              Gazebo Harmonic                 │
              │  ┌─────────┐  ┌───────────┐  ┌────────────┐  │
              │  │ PMB2    │  │ 7-DOF Arm │  │ RGB-D Cam  │  │
              │  │ Base    │  │ + Hey5    │  │ + Laser    │  │
              │  └─────────┘  └───────────┘  └────────────┘  │
              └──────────────────────────────────────────────┘
```

---

## Package Architecture

### Layer 1 — PAL Robotics Submodules (upstream, not edited)

| Package | Purpose |
|---------|---------|
| `src/tiago_robot/` | Robot URDF/xacro description, bringup launch, controller configs |
| `src/pmb2_robot/` | PMB2 differential-drive mobile base description |
| `src/pal_hey5/` | 5-fingered hand model (end effector) |
| `src/pal_gripper/` | Parallel gripper model |
| `src/pal_urdf_utils/` | Xacro helper macros (deg2rad, materials, FT sensors) |
| `src/launch_pal/` | PAL launch utilities and argument handling |
| `src/play_motion2/` | Pre-recorded motion sequence action server |
| `src/twist_mux/` | Velocity command priority multiplexer |

### Layer 2 — Simulation Package (`src/tiago_robot_simulation/`)

Orchestrates the full simulation environment. This is where all the Gazebo-specific customisation lives, keeping it isolated from PAL's upstream descriptions.

**Key files:**

| File | Purpose |
|------|---------|
| `launch/gazebo_simulation.launch.py` | Top-level launch: xacro, URDF resolution, Gazebo, spawn, controllers, bridges, SLAM |
| `robots/tiago.urdf.xacro` | Parametric robot model (arm type, end effector, laser, camera configurable) **(from PAL Robotics)** |
| `scripts/resolve_urdf.py` | Converts `package://` URIs to `file://` absolute paths for Gazebo spawning |
| `config/controllers.yaml` | ros2_control parameters for all joint trajectory controllers |
| `ros2_control/ros2_control.urdf.xacro` | Hardware interface definitions (GazeboSimSystem plugin) |
| `worlds/test.sdf` | Rectangular walled room with a table in the centre |

**Configurable robot options** (xacro arguments):

| Argument | Options | Default |
|----------|---------|---------|
| `arm_type` | `tiago-arm`, `no-arm` | `tiago-arm` |
| `end_effector` | `pal-hey5`, `pal-gripper`, `robotiq-2f-85`, `no-end-effector` | `pal-hey5` |
| `laser_model` | `sick-571`, `sick-561`, `hokuyo`, `no-laser` | `sick-571` |
| `camera_model` | `orbbec-astra`, `orbbec-astra-pro`, `no-camera` | `orbbec-astra` |
| `base_type` | `pmb2`, `omni_base` | `pmb2` |

**Launch pipeline (event-driven controller spawning):**

```
xacro + resolve_urdf.py → /tmp/tiago_resolved.urdf
    ↓
Gazebo (load world) + robot_state_publisher + ros_gz_bridge
    ↓
ros_gz_sim create (spawn robot at x=4, y=4)
    ↓ on_exit
joint_state_broadcaster
    ↓ on_exit
arm_controller, torso_controller, head_controller
    ↓ on_exit (last controller)
diffdrive_controller + Cartographer SLAM
```

**Sensor bridges** (Gazebo to ROS 2 via `ros_gz_bridge`):

| Gazebo Topic | ROS 2 Topic | Type |
|-------------|-------------|------|
| `/scan_raw` | `/scan` | `LaserScan` |
| `/base_imu` | `/imu` | `Imu` |
| `/head_front_camera/image` | `/head_front_camera/image` | `Image` |
| `/head_front_camera/depth_image` | `/head_front_camera/depth_image` | `Image` |
| `/head_front_camera/points` | `/points2` | `PointCloud2` |
| `/head_front_camera/camera_info` | `/head_front_camera/camera_info` | `CameraInfo` |
| `/world/rectangular_walls_world/model/tiago/joint_state` | `/joint_states` | `JointState` |
| `/clock` | `/clock` | `Clock` |

**ros2_control controllers:**

| Controller | Type | Joints |
|-----------|------|--------|
| `joint_state_broadcaster` | Broadcaster | All joints |
| `diffdrive_controller` | DiffDriveController | `wheel_left_joint`, `wheel_right_joint` |
| `arm_controller` | JointTrajectoryController | `arm_1_joint` ... `arm_7_joint` |
| `torso_controller` | JointTrajectoryController | `torso_lift_joint` |
| `head_controller` | JointTrajectoryController | `head_1_joint`, `head_2_joint` |
**Need to modify the `gripper` in urdf.xacro to have a controller if using the gripper end effector**

### Layer 3 — Custom Control Packages

#### 3a. Navigation Stack (`src/navigation_/`)

Custom Nav2 configuration using:
- **SLAM:** Cartographer (launched from gazebo_simulation.launch.py)
- **Localisation:** AMCL (500-2000 particles, likelihood field model)
- **Global planner:** A* (GridBased)
- **Local controller:** DWB (Dynamic Window approach, 40 Hz)
- **Velocity smoother:** with cmd_vel remapping to `/diffdrive_controller/cmd_vel`

#### 3b. Arm Controller (`src/arm_controller/`)

MoveIt-free arm manipulation using PyKDL:

| Module | Purpose |
|--------|---------|
| `tiago_kdl_ik.py` | KDL Newton-Raphson IK solver with joint limits, multi-seed search (30 random + 6 orientations), and geometry-based reachability pre-check |
| `arm_reach_action_server.py` | ROS 2 Action Server on `/arm_reach`. Pipeline: transform target to torso frame → reachability check → multi-seed IK → collision check → send trajectory |
| `tiago_tf_utils.py` | TF2 utility for transforming any frame into `torso_lift_link` before IK |
| `arm_reach_bt_node.py` | Nav2 Behavior Tree plugin wrapping the action for BT-based task sequencing |

**IK solver details:**
- Kinematic chain: `torso_lift_link` → `arm_tool_link` (7 DOF)
- Solver: `ChainIkSolverPos_NR_JL` (Newton-Raphson with joint limits)
- Multi-seed strategy: current joints, neutral pose, 30 random seeds within joint limits
- Multi-orientation: user-requested + pointing down/forward/up + rotated roll + 45-degree pitch
- Workspace bounds in torso frame: X [-0.10, 0.85], Y [-0.70, 0.70], Z [-0.50, 0.90], max reach 0.90m
- Inline URDF-to-KDL tree builder (replaces missing `kdl_parser_py` binary for Jazzy)

#### 3c. Torso Controller (`src/torso_controller/`)

Computes optimal `torso_lift_joint` height to maximise arm reachability:

| Module | Purpose |
|--------|---------|
| `torso_height_calculator.py` | Computes: `h = target_z - 0.790 - 0.1`, clamped to [0.0, 0.35] m |
| `torso_adjust_action_server.py` | ROS 2 Action Server on `/torso_adjust` |

The torso base sits at Z=0.790m above `base_footprint`. The arm workspace is centred ~0.1m above the torso link. By adjusting the prismatic joint (0 to 0.35m travel), the effective arm reach envelope shifts vertically.

#### 3d. High-Level Orchestrator (`src/tiago_controller/`)

Sequences the full reach pipeline with recovery:

| Module | Purpose |
|--------|---------|
| `reach_target_action_server.py` | ROS 2 Action Server on `/reach_target`. Coordinates: Navigate → Torso → Arm, with reposition-and-retry on arm failure |
| `navigation_helper.py` | Computes standoff navigation goals, angular approach candidates, map boundary validation |

**Reposition-and-retry logic:** When the arm fails because the target is geometrically outside the workspace, the orchestrator:
1. Transforms the target to `base_footprint` to measure the offset from the arm's ideal reach (0.45m forward)
2. Computes a correction vector, rotates it into the map frame
3. Navigates to the corrected position
4. Falls back to 8 angular approach candidates if the primary position is outside the map
5. Retries torso + arm (max 2 reposition attempts)

#### 3e. Collision Detector (`src/collision_detector/`)

3D collision detection using PointCloud2 data:

| Module | Purpose |
|--------|---------|
| `point_cloud_processor.py` | PointCloud2 → numpy conversion, voxel downsampling, self-occlusion filtering |
| `arm_collision_model.py` | Sphere-chain representation of the arm (1-3 spheres per link segment via FK) |
| `collision_checker.py` | Sphere vs point cloud distance checking using scipy KD-tree |
| `collision_detector_node.py` | Main node: `/check_collision` service, continuous 5 Hz monitor with emergency stop, RViz marker publishing |

#### 3f. Message Packages

| Package | Definitions |
|---------|-------------|
| `arm_controller_msgs` | `ArmReach.action` (target_pose, dry_run → success, is_reachable, joint_angles) |
| `torso_controller_msgs` | `TorsoAdjust.action` (target_pose → success, torso_height) |
| `tiago_controller_msgs` | `ReachTarget.action` (target_pose/frame, skip flags → per-phase success, joint solution) |
| `collision_detector_msgs` | `CheckCollision.srv` (joint_angles → collision_free, min_distance, closest_link) |

<!-- ---

## TF Frame Hierarchy

```
map
  └─ odom
       └─ base_footprint
            └─ base_link
                 ├─ wheel_left_link, wheel_right_link
                 ├─ base_laser_link (SICK laser)
                 └─ torso_fixed_link
                      └─ torso_lift_link  ← IK base frame
                           ├─ arm_1_link → arm_2_link → ... → arm_7_link → arm_tool_link
                           │                                                    └─ hand links (pal-hey5)
                           └─ head_1_link → head_2_link
                                └─ head_front_camera_link (RGB-D)
``` -->

---

## Key ROS 2 Topics and Actions

| Topic/Action | Type | Source |
|-------------|------|--------|
| `/scan` | `LaserScan` | Gazebo bridge (SICK laser) |
| `/points2` | `PointCloud2` | Gazebo bridge (RGB-D depth) |
| `/joint_states` | `JointState` | joint_state_broadcaster |
| `/mobile_base_controller/cmd_vel` | `Twist` | Nav2 → diffdrive |
| `/arm_reach` | Action | arm_controller |
| `/torso_adjust` | Action | torso_controller |
| `/reach_target` | Action | tiago_controller (orchestrator) |
| `/navigate_to_pose` | Action | Nav2 bt_navigator |
| `/check_collision` | Service | collision_detector |

---

## Build & Run

```bash
# Build all packages
cd /home/pika/Projects/tiago
colcon build --symlink-install
source install/setup.bash

# Full simulation (Gazebo + controllers + SLAM + RViz)
ros2 launch tiago_robot_simulation gazebo_simulation.launch.py rviz:=true slam_cartographer:=true

# Nav2 stack (separate terminal)
ros2 launch navigation_ navigation_launch.py use_sim_time:=true

# Controller stack — torso + arm + collision detector + orchestrator (separate terminal)
ros2 launch tiago_controller reach_target.launch.py

# Test full pipeline — navigate to target, adjust torso, reach with arm
ros2 run tiago_controller test_reach_target --ros-args -p x:=2.0 -p y:=0.0 -p z:=1.0

# Skip navigation (close target)
ros2 run tiago_controller test_reach_target --ros-args \
    -p x:=0.5 -p y:=0.0 -p z:=1.0 -p frame_id:=base_footprint -p skip_navigation:=true

# Dry run (no movement)
ros2 run tiago_controller test_reach_target --ros-args -p x:=2.0 -p y:=0.0 -p z:=1.0 -p dry_run:=true
```

---

## Challenges Solved

### 1. MoveIt 2 Unavailable for Jazzy + Gazebo Harmonic

MoveIt 2 packages were not installable on ROS 2 Jazzy with the Gazebo Harmonic stack. Binary packages had unresolvable dependency conflicts, and building from source pulled in a chain of additional missing dependencies (warehouse_ros, moveit_msgs, etc.) that also didn't have Jazzy binaries.

**Solution:** Built a complete MoveIt-free arm manipulation pipeline from scratch:
- Wrote a custom KDL-based inverse kinematics solver (`tiago_kdl_ik.py`) using PyKDL directly
- Implemented an inline URDF-to-KDL tree builder since `ros-jazzy-kdl-parser-py` didn't exist as a binary package either
- Added multi-seed and multi-orientation search to compensate for KDL Newton-Raphson's sensitivity to initial conditions
- Created a separate collision detection package using PointCloud2 data to replace MoveIt's planning scene

### 2. TIAGo Robot Description Not Fully Ported to Jazzy

PAL Robotics' official `tiago_robot` packages were designed for older ROS 2 distros and Gazebo Classic. Many xacro files referenced Gazebo Classic plugins (`<gazebo>` blocks with `libgazebo_ros_*`), which don't work in Gazebo Harmonic (Ignition-based). The simulation infrastructure (ros2_control hardware interface, sensor plugins) needed to be rewritten for the new Gazebo.

**Solution:**
- Copied the required URDF/xacro files from the PAL repository and created a separate `tiago_robot_simulation` package
- Rewrote the ros2_control hardware interface to use `gz_ros2_control/GazeboSimSystem` instead of the Classic Gazebo plugin
- Replaced all Classic Gazebo sensor plugins with Gazebo Harmonic equivalents (SDF-level `<sensor>` elements + `ros_gz_bridge` parameter bridges)
- Wrote a custom `resolve_urdf.py` script to convert `package://` URIs to absolute `file://` paths since Gazebo Harmonic's SDF spawner doesn't resolve ROS package paths natively
- Built the event-driven controller spawning cascade (`OnProcessExit` chains) to handle the timing dependencies between Gazebo model spawning, joint state broadcasting, and controller activation

### 3. Creating the Gazebo World

Built a custom SDF world (`test.sdf`) with 4 textured walls forming a bounded rectangular room and an interactive table at the centre. The walls use mesh geometry with collision, and the table is a dynamic body so it responds to robot contact. This required understanding Gazebo Harmonic's SDF format differences from Classic.

### 4. Inverse Kinematics Convergence

KDL's `ChainIkSolverPos_NR_JL` (Newton-Raphson) is extremely sensitive to the initial seed joint configuration. A single solve attempt fails on the majority of reachable targets with "max iterations exceeded" or "no convergence."

**Solution:** Implemented a multi-seed, multi-orientation search strategy:
- 6 end-effector orientations (user-requested, pointing down, forward, up, 180-degree roll, 45-degree pitch)
- 32+ seed states (current joints, neutral pose, 30 random seeds within joint limits)
- Fast geometric pre-check (bounding box + max-reach sphere) to skip impossible targets immediately
- FK verification after each successful solve to catch numerical drift

This brings the solve rate for geometrically reachable targets from ~15% (single seed) to ~95%+.

### 5. Torso–Arm Coordination

The arm's workspace is fixed relative to `torso_lift_link`, but the torso is a prismatic joint with 0.35m of travel. A target at Z=1.2m in `base_footprint` is unreachable with the torso at 0, but reachable with the torso raised. The existing TIAGo software had no coordination between the two.

**Solution:** Built a torso height optimiser (`torso_height_calculator.py`) that computes the torso position that places the target at the centre of the arm's vertical workspace, then integrated it into the action server pipeline so the torso adjusts before every IK solve.

### 6. Navigation Goal Computation with Growing SLAM Map

When using Cartographer SLAM, the map grows as the robot explores. Navigation goals computed for the full target position may be outside the known map. The robot would get stuck trying to navigate to an unmapped location.

**Solution:** Added map boundary validation (`goal_fits_in_map`) that checks goals against the current costmap extent. When the target standoff position falls outside the map, the system clamps the goal to the map boundary and retries as the map grows. The navigation phase loops until the robot reaches within standoff distance of the target.

### 7. Arm Failure Recovery (Reposition and Retry)

The initial design was linear: navigate → torso → arm. If the arm couldn't reach (target slightly too far forward, or off to the side), the whole pipeline failed. There was no recovery.

**Solution:** Added a reposition-and-retry loop. When the arm fails with a geometric reachability error, the orchestrator:
- Transforms the target to `base_footprint` to see exactly where it is relative to the robot
- Computes a correction vector to centre the target at the arm's optimal reach position
- Rotates that correction into the map frame using the robot's current yaw
- Navigates to the corrected position and retries
- Falls back to 8 angular approach candidates if the primary correction is outside the map
- Gives up after 2 reposition attempts to avoid infinite loops

---

## Submodule Management

```bash
# Initialise after cloning
git submodule update --init --recursive

# Update a specific submodule
git submodule update --remote src/tiago_robot
```

---

## License

Custom packages are licensed under Apache-2.0. PAL Robotics submodules retain their original licenses.
