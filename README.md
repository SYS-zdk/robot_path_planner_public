# robot_path_planner
## 1. Environment Requirements
<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-20.04-orange?logo=Ubuntu&Ubuntu-20.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-noetic-blue?logo=ROS&ROS=noetic" alt="ROS" />
</p>

- Ubuntu 20.04 LTS (Recommended)
- ROS Noetic 
- Dependencies:`ros-noetic-osqp`, `ros-noetic-gmapping`, `ros-noetic-map-server`, `ros-noetic-rviz`, `libeigen3-dev`

## 1.1 Build (Conan + catkin)

本项目的第三方库通过 Conan 管理（`3rd/`）。首次编译或换机器后需要先执行 Conan 安装生成 `3rd/conanbuildinfo.cmake` 等文件，否则会出现如 `find_package(osqp)` 找不到的问题。

```bash
cd /path/to/robot_path_planner
./scripts/build.sh
```

## 2. Project Overview

> Note: This repository is developed for experiments and secondary development on top of the open-source framework [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning). It is maintained in the open following community best practices and is still under active development; issues and PRs are welcome.

<p align="center">
  <img src="./images/video.gif" width="400" alt="Program Logo GIF" />
  <br />
  <em>Figure 1: Program Logo GIF</em>
</p>

This repository is primarily a personal learning-and-reproduction project in mobile robot / autonomous-driving motion planning. It studies and re-implements a collection of methods from open-source projects and the literature, and validates them end-to-end in a ROS1 navigation stack (costmap layers, planners, controllers, and simulation), with additional engineering refinements on implementation details, parameterization, and robustness.

In addition, based on my own understanding, I also designed and implemented several algorithms and system-level combinations, including HPCC (A* + RDP compression + corridor construction + convex optimization), HLP/HLPMPC(+Corridor), reachability-aware planning and control (reachability maps fused into A* and MPPI), and the Sunshine planner (sunshine ray sampling with layered optimization via conjugate gradient and iLQR).

Overall, the motivation of this repository is to provide an integrated, LEGO-like platform: a set of modular building blocks that helps people in the same field learn, reproduce, and compare motion-planning pipelines with less time spent on environment setup and resource hunting.

Below is a typical system architecture diagram used in this repository:

<p align="center">
  <img src="./images/second.jpg" width="400" alt="Organization" />
  <br />
  <em>Figure 2: Robot</em>
</p>

## 3. Core Innovations
### 3.1 Global Hierarchical Motion Planning for Ackermann Robots (HPCC)
A hierarchical global planning method combining path search, corridor construction, and convex optimization to generate kinodynamically feasible trajectories.

#### Key Steps
- Sparse Initial Path Generation: Fusion of A* algorithm with Ramer-Douglas-Peucker (RDP) path compression to eliminate redundant waypoints. The RDP operator (radial deviation threshold = 0.25) reduces waypoint count by 81.6% compared to raw A*, minimizing backend optimization load.
- Dynamic Corridor Inflation: Integrates Ackermann steering kinematics (minimum turning radius constraint) and convex decomposition to construct adaptive convex safety corridors, limiting trajectory randomness.
- Convex Optimization Formulation: Formulates trajectory generation as a convex optimization problem with safety corridor constraints (linear inequalities) and a cost function minimizing the second derivative of acceleration (snap).
- Curvature-Adaptive Time Redistribution: Dynamically adjusts time parameters based on trajectory curvature to ensure kinodynamic feasibility.

#### Core Formulas
Objective Function:

$$
\min \int_{0}^{T} \dddot{\mathbf{x}}(t)^2 \, dt
$$

where $\dddot{\mathbf{x}}(t)$ denotes the snap (third derivative of position).

Ackermann Kinematic Constraints:

$$
\begin{cases}
x_{i+1} = x_i + v_i \Delta T_i \cos(\theta_i) \\
 y_{i+1} = y_i + v_i \Delta T_i \sin(\theta_i) \\
 \theta_{i+1} = \theta_i + \omega_i \Delta T_i
\end{cases}
$$

with minimum turning radius

$$
R_{\text{min}} = \frac{L}{\tan(\delta_{\text{max}})}
$$

where $L$ is the wheelbase and $\delta_{\text{max}}$ the maximum steering angle.

Time Scaling Factor:

$$
\mu = \sqrt{\frac{k_{\text{max}}}{|k(s)|}}
$$

### 3.2 Hybrid Local Planner (HLP) Based on Dynamic State Graph
A sampling-based local planner designed for dynamic obstacle avoidance and real-time responsiveness, solving the "deadlock" issue in extreme orientation scenarios.

#### Key Steps
- Dynamic State Graph Construction: Builds a layered graph (20 layers × 60 nodes) with sampling based on robot kinematics (velocity/acceleration limits). Nodes encode position and orientation, with edges weighted by Euclidean distance and cost penalties.
- Layered Dijkstra Path Search: Evaluates node costs (obstacle cost + path alignment cost + goal proximity cost) to generate geometrically feasible local paths.
- Obstacle-Aware Velocity Allocation: Dynamically adjusts linear velocity based on obstacle density and kinodynamic constraints, avoiding over-acceleration.
- Hybrid A* Angular Velocity Optimization: Refines angular velocity in SE(2) space to ensure smooth steering and eliminate in-place oscillation.

#### Core Formulas
Node Cost Function:

$$
C_{\text{total}} = \alpha C_{\text{obs}} + \beta C_{\text{path}} + \gamma C_{\text{goal}}
$$

where $\alpha, \beta, \gamma$ are weighting factors for obstacle avoidance, path alignment, and goal proximity.

Allowed Linear Velocity:

$$
v_{\text{allow}} = v_{\text{max}} \cdot s,
\quad
s = \begin{cases}
1, & C_{\text{obs}} < C_{\text{thr}} \\
\\frac{C_{\text{max}} - C_{\text{obs}}}{C_{\text{max}} - C_{\text{thr}}}, & C_{\text{obs}} \geq C_{\text{thr}}
\end{cases}
$$

Layer Orientation Step:

$$
\Delta \theta_{\text{layer}} = \frac{2\theta_{\text{range}}}{N-1}
$$

Angular Velocity Calculation:

$$
\omega = \frac{\Delta \theta}{\Delta t / 2}
$$

### 3.3 HLPMPC: Hybrid Control with HLP + Short-Horizon MPC
An enhanced variant of HLP integrating model predictive control for smoother velocity profiles and improved tracking performance.

#### Key Steps
- Parallel Control Paths: Runs HLP (geometric feasibility) and MPC (dynamic smoothing) in parallel.
- Sparse QP Formulation: MPC optimizes velocity sequences (linear + angular) over a short horizon (N=5-10) using OSQP solver. The QP problem minimizes tracking error and velocity smoothness.
- Soft Blending Strategy: Uses a sigmoid function to compute continuous blending weights, avoiding abrupt control switches between HLP and MPC.
- Hierarchical Fallback Mechanism: Degrades to a simple lookahead proportional controller if MPC fails (infeasible solution/timeout).

#### Core Formulas
MPC Objective Function:

$$
J = \sum_{k=0}^{N-1} \left( q_v (v_k - v_{\text{ref}})^2 + q_\omega (\omega_k - \omega_{\text{ref}})^2 \right)
\\
\quad + \sum_{k=1}^{N-1} \left( r_v \Delta v_k^2 + r_\omega \Delta \omega_k^2 \right)
$$

Blending Weight Target:

$$
\alpha_{\text{target}} = \frac{1}{1 + \exp\bigl(-k\, (|v_{\text{MPC}} - v_{\text{HLP}}| - \tau)\bigr)}
$$

Filtered Blending Weight:

$$
\alpha_{\text{filtered}} = \alpha_{\text{prev}} + \gamma (\alpha_{\text{target}} - \alpha_{\text{prev}})
$$

### 3.4 HLPMPCCorridor: HLPMPC with Local Convex Safety Corridors
Integrates convex safety corridors into HLPMPC for enhanced geometric feasibility and collision avoidance.

#### Key Steps
- Corridor Construction: Decomposes the global path into a sequence of convex polygons via:
  - Initial polygon generation based on robot dimensions (width + safety margin).
  - Obstacle query using KD-tree for efficient neighborhood search.
  - Ellipse iteration to exclude collision points and refine corridor boundaries.
- Trajectory Feasibility Check: Rejects trajectories that exit the convex corridor (each sampled point must lie within at least one polygon).
- Integration with HLPMPC: Uses corridor constraints to prune unsafe candidate trajectories before blending.

#### Core Formulas
Corridor Boundary Constraint:

$$
\mathbf{n}_i^T (\mathbf{x}(t) - \mathbf{p}_i) \leq d_i, \quad \forall i
$$

Safety Corridor Inclusion:

$$
\mathbf{x}(t) \in \bigcup_{i=1}^{M} C_i \subseteq F_{\text{free}}
$$

### 3.5 Reachability-Aware Planning (Reachability Layer + A* + MPPI)
A reachability-aware navigation pipeline that augments traditional costmap-based planning with an explicit “passability / bottleneck” signal (reachability scores computed from costmaps). The reachability signal is fused into both global search (A*) and local control (MPPI) to prefer wider, safer corridors and avoid narrow passages.

#### Key Steps
- Global reachability map ($R_g$): compute a static environment score from the global master costmap using geometric cues such as obstacle clearance and (optionally) a Voronoi-skeleton-based corridor proxy.
- Global planning (A*): fuse costmap cost and reachability preference during node expansion; a hard threshold can be used to prune cells with very low reachability.
- Local reachability map ($R_\ell$): compute a robot-conditioned reachability field in the local window using widest-path (maximin) propagation from the robot cell.
- Local control (MPPI): treat the local costmap (inflation) as collision hard constraints, and incorporate reachability as both a hard constraint (`min_reachability`) and a soft critic (`reachability_scale`).

#### Core Formulas
Continuous compression used in reachability scoring:

$$
\mathrm{cont01}(\Delta;s)=\frac{\max(0,\Delta)}{\max(0,\Delta)+s}
$$

One typical global reachability aggregation (clearance/width/ridge factors):

$$
R_g(x)=\mathrm{clip}_{[0,1]}\Big( s_c(x)^{\alpha_c}\cdot s_w(x)^{\alpha_w}\cdot s_r(x)^{\alpha_r} \Big)
$$

Robot-conditioned local reachability via widest-path (maximin) propagation:

$$
R_\ell(x)=\max_{p\in\mathcal{P}(s\to x)}\ \min_{y\in p}\ r(y)
$$

MPPI weighting and update (numerically-stable form):

$$
w_k\propto \exp\left(-\frac{S_k-\min_j S_j}{\lambda}\right),\quad \sum_k w_k=1
$$

Reachability critic (soft preference) used in rollout evaluation:

$$
S\mathrel{+}=w_{\text{reach}}\cdot (1-\bar R_\ell)
$$

For full implementation-level details and parameter entry points, see `docs/reachability_planner.md`.

### 3.6 Sunshine Planner: Ray Sampling + Layered Optimization (Sunshine + CG + iLQR)
A hierarchical motion planning framework for grid maps that combines geometry-aware sampling with layered optimization: Sunshine ray sampling for fast global feasibility, conjugate-gradient smoothing for a real-time global reference, and iLQR for high-quality local optimal control.

#### Key Steps
- Sunshine ray sampling: perform 360° ray casting to measure free-space “sunlight” distances, and extract geometry-driven keypoints (e.g., corner / tangent-like candidates) to construct a sparse planning graph.
- Global search on samples: run graph search (A*) on the sampled graph to obtain an initial collision-free grid path $\pi_0$.
- Global path smoothing (CG): apply a lightweight conjugate-gradient optimizer to refine $\pi_0$ into a smoother reference path $\pi^*$ with low computation cost.
- Local optimization & control (iLQR): refine the trajectory under kinematic constraints and environment costs (e.g., corridor / ESDF), producing executable commands $(v,\omega)$; failures can trigger event-based replanning.

#### Core Formulas
Ray directions for a 360° scan:

$$
\phi_i = \frac{2\pi i}{N},\quad i=0,1,\dots,N-1
$$

Ray-cast free-space length (conceptual form):

$$
d(\phi_i)=\min\{r\ge 0\mid \mathrm{occupied}(x_0+r[\cos\phi_i,\sin\phi_i])=1\}
$$

Conjugate-gradient iterative update (standard form):

$$
\mathbf{p}_{k+1}=\mathbf{p}_k+\alpha_k\mathbf{d}_k,\quad
\mathbf{d}_{k+1}=-\nabla J(\mathbf{p}_{k+1})+\beta_k\mathbf{d}_k
$$

iLQR objective and dynamics (standard form):

$$
\min_{\{u_t\}}\ \sum_{t=0}^{T-1} \ell(x_t,u_t)+\ell_T(x_T),\quad x_{t+1}=f(x_t,u_t)
$$

For implementation mapping (code/config) and detailed derivations aligned with this repository, see `docs/sunshine_planner.md`.

## 4. Repository Structure (accurate and annotated)

```
robot_path_planner/
├── src/
│   ├── core/
│   │   ├── controller/
│   │   │   ├── hlpmpccorridor_local_planner/    # HLP + MPC + Safety Corridor
│   │   │   ├── reachability_controller/         # MPPI local controller (reachability-aware)
│   │   │   └── ilqr_controller/                 # iLQR local controller
│   │   ├── common/
│   │   │   └── safety_corridor/                 # ConvexSafetyCorridor implementation and utilities
│   │   ├── path_planner/
│   │   │   └── path_planner/                    # Global planners (HPCC-related code)
│   │   │       ├── src/graph_planner/           # graph-based global planner implementations
│   │   │       └── src/sample_planner/          # sampling-based global planners
│   │   └── trajectory_planner/
│   ├── plugins/
│   │   └── map_plugins/
│   │       ├── globalreachability_layer/        # global reachability costmap layer
│   │       ├── localreachability_layer/         # local reachability costmap layer
│   │       ├── social_layer/                    # pedestrian social cost layer
│   │       ├── distance_layer/                  # ESDF / distance utilities & layer
│   │       └── voronoi_layer/                   # Voronoi cost layer
│   ├── plugins/gazebo_plugins/.../pedsim_msgs/  # messages used by social/dynamic pedestrian modules
│   ├── sim_env/                                 # Simulation configs / launch / RViz
│   │   ├── config/controller/
│   │   └── launch/
├── scripts/                                      # helper scripts (build/run/stop)
├── assets/                                       # diagrams used in README
├── images/                                       # documentation images
├── build/  devel/                                # catkin build outputs (ignored)
├── LICENSE
└── README.md
```

Highlighted core implementation locations (quick reference):

| Area | Location (path) | Representative files / notes |
|---|---|---|
| Global hierarchical planner (HPCC) | `src/core/path_planner/path_planner/src/graph_planner/` | graph-based global planner sources; path compression utilities |
| Local planner (HLP + MPC + Corridor) | `src/core/controller/hlpmpccorridor_local_planner/` | main: `src/hybrid_planner_ros.cpp`, `src/hybrid_planner.cpp` |
| Reachability global/local layers | `src/plugins/map_plugins/{globalreachability_layer,localreachability_layer}/` | reachability cache + visualization topics |
| Reachability local controller (MPPI) | `src/core/controller/reachability_controller/` | MPPI rollout & critics (incl. reachability) |
| iLQR local controller | `src/core/controller/ilqr_controller/` | iLQR + corridor support |

The table highlights the four core innovation areas and points to specific package paths and representative source files for quick inspection.

## 5. Build & Run

### Build Examples
#### 5.1 Install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full *suggested*).

#### 5.2 Install git.
    sudo apt install git

#### 5.3 Install dependence

- OSQP
    ```
    git clone -b release-0.6.3 --recursive https://github.com/oxfordcontrol/osqp
    cd osqp && mkdir build && cd build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j6
    sudo make install
    sudo cp /usr/local/include/osqp/* /usr/local/include
    ```
- OSQP-Eigen
    ```
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen && mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```
- Other dependence.
    ```
    sudo apt install python-is-python3 \
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn
    ```

#### 5.4 Clone the reposity
```
git clone https://github.com/SYS-zdk/robot_path_planner_public.git
```

#### 5.5 Compile the code.

    cd robot_path_planner/
    catkin_make
    # or catkin build
    # you may need to install it by: sudo apt install python-catkin-tools
    # 由于存在许多第三方包，而不是通过系统安装，由人工导入，所以可能会存在编译顺序错误引起编译失败
    # 如social_layer包依赖pedsim_msgs
    # catkin重新构建时，没有优先编译pedsim_msgs，反而先编译了social_layer
    # 此时pedsim_msgs还未生成头文件（TrackedPersons.h）和编译产物，social_layer编译时自然找不到依赖，直接报错
    # catkin_make --cmake-args -DCATKIN_WHITELIST_PACKAGES="pedsim_msgs"
    # catkin_make -DCMAKE_BUILD_TYPE=Release或者./build.sh

### Run Examples

#### 5.6 Execute the code

    cd scripts/
    ./main.sh

    > NOTE: Modifying launch files may not have any effect, because they are regenerated by a Python script based on `src/user_config/user_config.yaml` when you run `main.sh`. Therefore, you should modify configurations in `user_config.yaml` instead of launch files.

#### 5.7 Use **2D Nav Goal** in RViz to select the goal.

#### 5.8  Moving!

#### 5.9  You can use the other script to shutdown them rapidly.
    ./killpro.sh


## 6. Parameters

### 6.1 Key Parameters
Refer to src/sim_env/config/controller/hlpmpccorridor_local_planner.yaml for critical parameters:
  - mpc_horizon: MPC prediction steps (default: 5)
  - mpc_dt: MPC time step (default: 0.1s)
  - mpc_q_v/mpc_q_w: MPC tracking weights (default: 1.0/1.0)
  - mpc_r_v/mpc_r_w: MPC smoothing weights (default: 0.5/0.5)
  - alpha_sigmoid_k: Blending sigmoid steepness (default: 5.0)
  - safety_corridor_range: Corridor expansion range (default: 0.5m)

### 6.2 Practical Tuning Tips
- For smoother velocities: Increase mpc_r_v/mpc_r_w or reduce mpc_horizon.
- For cluttered environments: Reduce alpha_sigmoid_k to rely more on HLP.
- For corridor rejection issues: Increase safety_corridor_range or trajectory sampling density.
- For kinodynamic compliance: Adjust max_vel_x, max_acc_x, max_omega to match your robot's specs.

## 7. References
- ros_motion_planning (upstream framework reference): https://github.com/ai-winter/ros_motion_planning
- OSQP: https://github.com/oxfordcontrol/osqp
- Ramer-Douglas-Peucker Algorithm: https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
- Hybrid A* Algorithm: https://github.com/hku-mars/hybrid_astar
- Convex Safety Corridor Decomposition: https://arxiv.org/abs/1907.08584
- Model Predictive Control for Mobile Robots: https://arxiv.org/abs/2003.09554

## 8. Citation
If you want to learn more about the detailed mathematical derivation of this project or if this project helps your research, please refer to the following citation formats:
- Global Planning Part: 张定坤，梁海朝。基于动态走廊膨胀与凸优化的移动机器人分层运动规划 [J/OL]. 系统仿真学报，1-21 [2026-01-13]. https://link.cnki.net/urlid/11.3092.V.20250916.1349.002.
- Local Planning Part: 张定坤，吴兴涛，梁海朝。基于动态状态图的自主移动机器人混合局部规划方法 [J]. 机器人技术与应用，2025, (05): 20-34.
- Environment Modeling Part: Zhang Dingkun. robot_path_planner_public. [EB/OL]. https://github.com/SYS-zdk/robot_path_planner_public, 2026.

## 9. License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0) — see the LICENSE file for details.
