# mad_icp_core

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![License](https://img.shields.io/badge/license-BSD--3--Clause-green)
![C++](https://img.shields.io/badge/C++-17-blue)

A ROS-agnostic pure C++ library implementing the core algorithm of
[MAD-ICP: It Is All About Matching Data — Robust and Informed LiDAR Odometry](https://github.com/rvp-group/mad-icp)
(RA-L 2024, Ferrari et al., R(obots) V(ision) and P(erception) group).

`mad_icp_core` is refactored from the original implementation to separate
the algorithm logic from any ROS dependency, Python bindings, and other
non-essential components. It is designed to be consumed by downstream packages —
such as the companion [`mad_icp`](https://github.com/Chris7462/mad_icp) ROS2
package — or embedded into any C++ project independently.

Key changes from the original:
- ROS completely removed — pure C++ with no ROS headers
- Python bindings (pybind11) removed
- YAML-based runners removed — parameters are passed directly via constructor
- CMake modern targets (`mad_icp_core::mad_icp_core`) for clean downstream integration
- Built as a static library with `ament_cmake` for easy ROS2 workspace integration

---

## Algorithm

MAD-ICP (Matching And Data ICP) is a minimal, robust, and real-time LiDAR
odometry system that maintains a local keyframe-based map using a custom
spatial data structure called the **MAD-tree**.

**1. MAD-tree** — A custom kd-tree variant that organizes point clouds
into leaf nodes. Each leaf stores a local mean and normal, enabling
efficient nearest-neighbor lookup and planar patch matching without
explicit feature extraction.

**2. MAD-ICP Registration** — Point-to-plane ICP formulated with a Huber
robust kernel (`rho_ker`). Correspondences are established between incoming
scan leaves and the local map leaves. The transformation is solved iteratively
using Lie algebra updates on SE(3).

**3. Velocity Estimator** — A constant-velocity motion model that predicts
the initial guess for each frame, improving convergence speed and robustness
under fast motion.

**4. Pipeline** — Manages the full odometry loop: initialization, optional
motion deskewing, ICP registration, keyframe selection, and local map
maintenance. The map is kept as a sliding window of keyframes bounded by
`num_keyframes`. A keyframe is promoted based on a weight threshold (`p_th`)
that measures how well the current frame is registered.

---

## Dependencies

| Dependency | Version |
|---|---|
| CMake | ≥ 3.16 |
| C++ | 17 |
| Eigen3 | ≥ 3.3 |
| OpenMP | any |
| PCL | ≥ 1.12 (common only) |

Install system dependencies on Ubuntu:

```bash
sudo apt-get install libeigen3-dev libpcl-dev
```

OpenMP is typically included with GCC. If not:

```bash
sudo apt-get install libomp-dev
```

---

## Building

`mad_icp_core` uses `ament_cmake` and is built inside a ROS2 workspace with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Chris7462/mad_icp_core.git
cd ~/ros2_ws
colcon build --packages-select mad_icp_core
source install/setup.bash
```

---

## Related Packages

**[mad_icp](https://github.com/Chris7462/mad_icp)** — The companion ROS2 package
that wraps `mad_icp_core` with an `rclcpp` node for `lidar_odometry`.

---

## Reference

**[MAD-ICP: It Is All About Matching Data — Robust and Informed LiDAR Odometry](https://github.com/rvp-group/mad-icp)** —
The original implementation by Ferrari et al., from which `mad_icp_core` was refactored.
