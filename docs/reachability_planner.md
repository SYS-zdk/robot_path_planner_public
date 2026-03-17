
\newcommand{\R}{\mathbb{R}}

# 基于可达性图与 MPPI 的移动机器人运动规划方法（仓库实现对照版）

日期：2026-01-30

本文以本仓库 `robot_path_planner` 为实现依据，面向论文写作场景，系统阐述：

- 全局可达性地图与全局代价地图（global costmap）的关联；
- 全局可达性规划器（A*）如何融合可达性并生成全局路径；
- 局部可达性地图与局部代价地图（local costmap）的关联；
- 局部可达性规划器（MPPI 控制器）如何在代价/约束层面融合可达性并输出速度指令。

为避免"原理与实现不一致"导致论文出错，本文所有公式与叙述均以以下代码为准：

- 全局可达性层：`src/plugins/map_plugins/globalreachability_layer/src/globalreachability_layer.cpp`
- 局部可达性层：`src/plugins/map_plugins/localreachability_layer/src/localreachability_layer.cpp`
- 全局可达性规划器：`src/core/path_planner/path_planner/src/graph_planner/reachability_planner.cpp`
- 局部可达性规划器（MPPI 内核）：`src/core/controller/reachability_controller/src/reachability_mppi.cpp`
- 局部控制器主体：`src/core/controller/reachability_controller/src/reachability_controller.cpp`
- costmap 插件配置：
	- `src/sim_env/config/costmap/global_costmap_plugins.yaml`
	- `src/sim_env/config/costmap/local_costmap_plugins.yaml`

---

# 第一章 引言与整体框架

## 1.1 研究背景与问题定义

在 ROS1 `move_base` 框架中，传统导航通常依赖代价地图（costmap）表达障碍与膨胀代价，并通过全局规划器生成路径、局部规划器生成速度指令。然而，仅使用 costmap 往往难以稳定表达以下“通行性/可达性”信息：

- 走廊是否足够宽（对不同机器人尺寸/安全距离敏感）；
- 局部通道是否存在“瓶颈”（路径虽未碰撞但很难通过）；
- 在多条可行路线中，如何偏好“更宽、更居中、更安全”的通道；
- 局部控制中如何避免“贴墙”“钻缝”造成的抖动与停走。

为此，本仓库引入可达性图（Reachability Map），并分别在全局/局部层面构建：

- **全局可达性地图**：反映环境几何对通行性的影响，低频更新；
- **局部可达性地图**：反映“从机器人当前位置出发”的可达传播结果（机器人条件），中频更新。

在规划层面，对应实现：

- **全局可达性规划器**：在 A* 搜索代价中融合全局可达性；
- **局部可达性规划器**：采用 MPPI（Model Predictive Path Integral）在采样轨迹代价中融合局部可达性，并可通过阈值形成硬约束。

## 1.2 系统架构与数据流

### 1.2.1 ROS1/move_base 中的 costmap 与 layer

`move_base` 维护两张 costmap：

- `global_costmap`：全局规划使用；
- `local_costmap`：局部控制使用。

两者均由 `layered_costmap` 通过插件叠加得到“master costmap”。本仓库的插件配置（关键顺序）为：

- 全局：`static_map -> obstacle_layer -> globalreachability_layer -> inflation_layer -> voronoi_layer`
- 局部：`static_map -> obstacle_layer -> localreachability_layer -> inflation_layer`

### 1.2.2 可达性地图与代价地图的“关联”

本文强调一个非常关键的工程事实：

1) **可达性层在计算时依赖 costmap（master_grid）作为输入**，它读取障碍/未知等栅格信息来估计 clearance、骨架等；
2) **可达性层的输出默认不写回 costmap**（`write_to_master=false`），而是以缓存向规划器提供额外信息；
3) **规划器/控制器同时使用 costmap 与 reachability**：
	 - costmap（含 inflation）提供避障硬约束；
	 - reachability 提供“通行偏好/瓶颈约束/走廊中心偏好”。

在本仓库中：

- 全局可达性缓存名：`GlobalReachabilityLayer::cache_env_score_`（接口 `getEnvScoreCache()`）
- 局部可达性缓存名：`LocalReachabilityLayer::cache_reach_field_`（接口 `getReachabilityCache()`）

当 `write_to_master=true` 时，可达性会被映射为 costmap 的 `unsigned char` 代价写回 master_grid（只增不减），这会影响其它规划器；因此默认关闭。

## 1.3 统一符号与基本模型

### 1.3.1 可达性分数定义

在本文中，reachability 统一定义为 $R\in[0,1]$：

- $R=1$ 表示更安全/更宽阔/更“好走”；
- $R=0$ 表示不可达或极高风险。

### 1.3.2 机器人运动学（与 MPPI rollout 一致）

MPPI rollout 使用的是标准平面差分驱动（离散时间）模型：

$$
\begin{aligned}
x_{t+1} &= x_t + v_t\cos\theta_t\,\Delta t\\
y_{t+1} &= y_t + v_t\sin\theta_t\,\Delta t\\
	heta_{t+1} &= \mathrm{wrap}(\theta_t + \omega_t\,\Delta t)
\end{aligned}
$$

其中 $x_t,y_t$ 是机器人在全局坐标系中的位置（米），$\theta_t$ 为航向角（弧度）；$v_t$ 为线速度（m/s），$\omega_t$ 为角速度（rad/s）。

符号 $\mathrm{wrap}(\cdot)$ 表示角度归一化到 $(-\pi,\pi]$ 的操作，对应实现 `angles::normalize_angle()`；$\Delta t$ 为离散积分步长，对应代码参数 `mppi_dt`（默认会尝试从 `controller_frequency` 推导）。

---

## 1.4 从“代价地图”到“可达性缓存”的工程动机

在本仓库中，可达性层（global/local reachability layer）默认不写回 costmap（`write_to_master=false`）。这样做的核心原因是：costmap 的数值语义在 ROS1 导航链路里通常被理解为“碰撞风险/障碍惩罚”，而可达性更像“通行偏好/通道质量”。若将偏好直接写入 costmap，可能会把“偏好项”误当成“近似障碍项”，从而改变其它规划器/控制器的行为。

因此，本文所述系统采用“**costmap 负责避障硬约束，可达性以缓存形式提供软偏好与瓶颈约束**”的分工：

- **层（Layer）侧**：读取 master costmap，计算 reachability 并缓存；
- **规划器/控制器侧**：同时读取 costmap 与 reachability 缓存，在代价函数中融合。

---

# 第二章 全局部分：全局可达性地图与全局可达性规划器

## 2.1 全局可达性地图（GlobalReachabilityLayer）

### 2.1.1 输入、输出与更新策略

输入：全局 master costmap `master_grid`（已叠加 static/obstacle 等层）。

输出：全局可达性场 `env_score`，缓存为 `cache_env_score_`（类型 `std::vector<double>`，范围 $[0,1]$）。

更新策略：

- 通过 `update_interval_s` 控制低频重算；
- 若不需要重算，则直接复用缓存（避免可视化/写回出现周期性“有/无”的闪烁）。

### 2.1.2 清障距离（clearance）估计

全局层在 `updateCosts()` 中调用 `euclidean_distance_field::EuclideanDistanceField` 计算 2D EDF（Euclidean Distance Field）。该 EDF 内部以“栅格”为单位存储到最近障碍的距离平方（单位：cells$^2$），随后再换算为米。

$$
d_{\text{obs}}(x)=\sqrt{\max(0, d^2_{\text{cells}}(x))}\cdot \mathrm{res}
$$

其中 $\mathrm{res}$ 为地图分辨率（m/cell）。若某栅格的 EDF 值非有限（例如未正确计算），实现会将其视为 $+\infty$，并在后续评分中把该点的通行性“放行”为更保守的默认值（见 `continuous01()` 的 `!isfinite(delta)` 分支）。

### 2.1.3 骨架（Voronoi skeleton）与到骨架距离

为了表达“走廊中心线”偏好，全局层可选使用 `DynamicVoronoi` 在二值占据栅格上计算 Voronoi 骨架（近似 medial axis）。实现上会同时考虑 `vor.isVoronoiAlternative()` 与 `vor.isVoronoi()` 两种骨架判定，并对骨架 seed 做后处理，使其更适合后续的距离传播与可视化。

骨架生成后，代码使用 8 邻接 Dijkstra 传播，得到每个自由栅格到最近骨架 seed 的近似欧氏距离 $d_{\text{ridge}}(x)$（单位：米）。这一步的目的不是精确几何，而是提供一个稳定、光滑的“离中心线多远”的量。

为了改善骨架稀疏与孤立点问题，代码中还包含：

- 骨架 seed 膨胀（`skeleton_seed_dilate_radius_cells`）；
- 去除孤立骨架像素（邻域计数为 0 的点删除）；
- 若骨架为空，则退化为“仅 clearance 的走廊代理”。

### 2.1.4 corridor 半径代理（corridor thickness）

对任意栅格 $x$，实现定义一个“走廊半径（半宽）代理” $r_{\text{corr}}(x)$，其来源不是 $x$ 自己的 clearance，而是 **最近骨架 seed 处的 clearance**：

- 在 Dijkstra 初始化时，每个骨架 seed $s$ 记录自身 clearance $d_{\text{obs}}(s)$；
- 在距离传播过程中，`ridge_clearance_m[nidx] = ridge_clearance_m[cur.idx]`，即把“最近骨架 seed 的 clearance”沿最短路径传播到周围。

因此 $r_{\text{corr}}(x)$ 在代码中对应 `ridge_clearance_m[idx]`，可理解为“离 $x$ 最近的走廊中心线位置有多宽”。直观上：若中心线处更宽，则其影响区域内的 $r_{\text{corr}}$ 更大，从而让 A* 更倾向穿过更宽的走廊。

若 `enable_skeleton=false` 或骨架为空，代码退化为 `r_{\text{corr}}(x)=d_{\text{obs}}(x)`，并将 $d_{\text{ridge}}(x)$ 置为 0，使评分退化为“仅 clearance/局部宽度代理”的形式。

### 2.1.5 全局可达性评分函数（与实现一致）

全局可达性最终以乘性方式融合三类因子：

- clearance 因子（离障更远更好）；
- corridor 半径因子（走廊更宽更好）；
- ridge 吸引因子（靠近骨架更好）。

代码中的连续压缩函数为：

$$
\mathrm{cont01}(\Delta;s)=\frac{\max(0,\Delta)}{\max(0,\Delta)+s}
$$

定义机器人安全门槛：

$$
c_{\min}=r_{\text{robot}}+d_{\text{safety,min}}
$$

对应实现（`GlobalReachabilityLayer::updateCosts()`）的评分可写为：

$$
\begin{aligned}
s_c(x) &= \mathrm{cont01}(d_{\text{obs}}(x)-c_{\min};\ \texttt{risk\_clearance\_scale\_m})\\
s_w(x) &= \mathrm{cont01}(r_{\text{corr}}(x)-c_{\min};\ \texttt{risk\_width\_scale\_m})\\
s_r(x) &=
\begin{cases}
\exp\left(-\dfrac{d_{\text{ridge}}(x)}{\texttt{ridge\_attract\_scale\_m}}\right), & \texttt{enable\_skeleton}=\texttt{true}\\
1, & \texttt{enable\_skeleton}=\texttt{false}
\end{cases}\\
R_g(x) &= \mathrm{clip}_{[0,1]}\Big( s_c(x)^{\alpha_c}\cdot s_w(x)^{\alpha_w}\cdot s_r(x)^{\alpha_r} \Big)
\end{aligned}
$$

其中 $\alpha_c,\alpha_w,\alpha_r$ 分别对应参数 `alpha_clearance/alpha_width/alpha_ridge`，用于调节各因子的“陡峭程度”（指数越大，对低分更敏感）。

为避免符号歧义，式中各量的物理意义如下：

- $d_{\text{obs}}(x)$：栅格 $x$ 到最近障碍的欧氏距离（米），来自 EDF；
- $d_{\text{ridge}}(x)$：栅格 $x$ 到最近骨架 seed 的传播距离（米），来自 8 邻接 Dijkstra；
- $r_{\text{corr}}(x)$：走廊半宽代理（米），等于最近骨架 seed 处的 clearance（对应 `ridge_clearance_m`）；
- $c_{\min}=r_{\text{robot}}+d_{\text{safety,min}}$：机器人最小安全半径（米）；
- `risk_*_scale_m`：连续压缩函数的尺度（米），尺度越大，评分从 0 过渡到 1 越“缓”；
- `ridge_attract_scale_m`：骨架吸引衰减尺度（米），越大表示“离中心线远一点也还能接受”。

### 2.1.5（补充）参数物理意义速查（全局层）

下表给出论文写作中最常引用的参数（单位以实现为准）：

| 参数名 | 含义（物理意义） | 典型影响 |
|---|---|---|
| `robot_radius` | 机器人等效半径 $r_{\text{robot}}$（m） | 越大越保守，窄通道更易被判低可达 |
| `safety_distance_min` | 最小安全裕度 $d_{\text{safety,min}}$（m） | 提高后，贴墙区域更低分 |
| `treat_unknown_as_obstacle` | 是否把 `NO_INFORMATION` 当障碍 | 开启后未知区域可达性为 0，适合保守导航 |
| `occupied_cost_threshold` | 视为障碍的 cost 阈值（uchar） | 与 costmap 配置相关，影响 EDF/骨架 |
| `risk_clearance_scale_m` | clearance 因子过渡尺度（m） | 越大越“平滑”，评分上升更慢 |
| `risk_width_scale_m` | corridor 因子过渡尺度（m） | 越大越不敏感于走廊宽度差异 |
| `alpha_clearance/alpha_width/alpha_ridge` | 三因子指数 | 越大越强调该因素（低分惩罚更强） |
| `enable_skeleton` | 是否启用 Voronoi 骨架 | 关闭后不再偏好中心线，仅依赖 clearance/宽度代理 |
| `ridge_min_clearance_m` | 可作为骨架 seed 的最小 clearance（m） | 提高后骨架更“远离障碍”，但可能更稀疏 |
| `ridge_attract_scale_m` | 离骨架的吸引衰减尺度（m） | 越大越不严格居中 |
| `skeleton_seed_dilate_radius_cells` | 骨架 seed 膨胀半径（cells） | 增大可减少骨架稀疏导致的断裂 |
| `smooth_radius` | `env_score` 平滑半径（cells） | 增大可减轻棋盘格/噪点，但会模糊细节 |
| `update_interval_s` | 低频重算周期（s） | 越大越省算力，但环境变化反应更慢 |
| `max_cost` | 写回 costmap 的最大增量（0~252） | 仅 `write_to_master=true` 时生效，过大易“污染语义” |

### 2.1.6 与 global costmap 的耦合方式（写回/不写回）

全局可达性与 global costmap 的关联分为两层：

1) **计算耦合**：`R_g` 的计算以 global master costmap 为输入（障碍/未知决定 clearance 与骨架）。
2) **输出耦合**：
	 - 默认 `write_to_master=false`：只更新缓存，不改变 costmap；
	 - 若 `write_to_master=true`：把 $R_g$ 映射为代价并写回 costmap（只增不减）：

$$
c_{\text{reach}}(x)=\mathrm{round}\Big((1-R_g(x))\cdot \texttt{max\_cost}\Big)
$$

该映射与实现一致（`reachabilityToCost()`），且仅在 `new_cost > old_cost` 时写入。

## 2.2 全局可达性规划器（ReachabilityPathPlanner, A*）

### 2.2.1 规划器输入输出

输入：起点/终点栅格，global master costmap（用于障碍/代价），以及全局可达性缓存 $R_g$。

输出：一条全局路径（栅格序列），供局部控制器跟踪。

### 2.2.2 从 layer 直接读取 `env_score`（不依赖可视化 topic）

规划器在 `updateReachabilityLayer()` 中遍历 `layered_costmap->getPlugins()`，通过 `dynamic_pointer_cast<GlobalReachabilityLayer>` 找到全局可达性层，然后用 `getEnvScoreCache()` 获取 `env_score` 快照。

因此：

- 全局规划器**不订阅** `reachability_grid` / `reachability_cloud`；
- 即使 `write_to_master=false`，仍可完全工作。

### 2.2.3 A* 扩展与代价融合（与代码一致）

对 A* 的每一步扩展（motion），代码首先用 costmap 做可行性判定，再用 $R_g$ 做阈值过滤与代价惩罚。

1) **可行性（基于 costmap）**

代码读取 `c_new = costmap_->getCharMap()[node_new.id()]` 与 `c_cur`，并通过如下启发式跳过“更糟且接近致命”的栅格：当 `c_new >= LETHAL_OBSTACLE * factor_ && c_new >= c_cur` 时不扩展。写论文时可抽象为：

- 若栅格为障碍/致命代价，则不扩展。

2) **可达性阈值（hard gate）**

若 $R_g(x) < \texttt{min\_reachability}$，则该栅格不扩展（`continue`）。

实现细节补充：若规划器未能找到 `GlobalReachabilityLayer` 或缓存尚无效，则 `reachabilityScoreAtCell()` 会返回 $R_g=1$（相当于“暂不使用可达性约束/惩罚”），保证系统仍可退化为传统 A*。

3) **融合代价（soft bias）**

代码将 costmap 归一化代价与可达性惩罚叠加成 `penalty`：

$$
\mathrm{penalty}(x)=w_c\cdot \frac{c(x)}{255}+w_r\cdot (1-R_g(x))
$$

并把单步代价写为：

$$
\Delta g = \texttt{step\_base\_cost}\cdot g_{\text{motion}}\cdot\big(1+\mathrm{penalty}(x)\big)
$$

其中 $w_c$ 对应 `cost_weight`，$w_r$ 对应 `reachability_weight`，$g_{\text{motion}}$ 对应 motion 的几何步长（1 或 $\sqrt{2}$）。

### 2.2.4 全局规划器与 global costmap 的关系

全局规划器永远读取 global costmap（障碍/膨胀代价）作为可行性与代价的一部分；`R_g` 作为额外的“环境通行偏好”参与搜索：

- costmap 负责“不能撞、不要贴太近”；
- $R_g$ 负责“更宽的走廊更优、靠近骨架更优、减少瓶颈路线”。

### 2.2.5（补充）参数物理意义速查（全局规划器）

| 参数名 | 含义（物理意义） | 典型影响 |
|---|---|---|
| `min_reachability` | 全局可达性硬阈值（0~1） | 提高会更保守，可能导致无路可走 |
| `cost_weight` | costmap 归一化代价权重 $w_c$ | 提高会更“怕障碍/膨胀”，更远离障碍 |
| `reachability_weight` | 可达性惩罚权重 $w_r$（对 $1-R_g$） | 提高会更偏好宽走廊/居中 |
| `step_base_cost` | 每步基础代价缩放 | 影响 $g$ 增长尺度与启发式相对比重 |

---

# 第三章 局部部分：局部可达性地图与局部可达性规划器（MPPI）

## 3.1 局部可达性地图（LocalReachabilityLayer）

### 3.1.1 输入、输出与位姿依赖

输入：local master costmap `master_grid`（已叠加 static/obstacle 等层），以及机器人当前位姿（从 `updateBounds(robot_x, robot_y, ...)` 记录的 `last_robot_x_/y_`）。

输出：机器人条件的局部可达场 $R_\ell(x)$，缓存为 `cache_reach_field_`（接口 `getReachabilityCache()`）。

该层与全局层最大的差异在于：**它显式依赖机器人当前位置**，输出是“从机器人出发能否到达”的传播结果。

### 3.1.2 点通行性（local_score）计算

局部层先计算每个栅格的点通行性 $r(x)$（代码：`computeReachability()`）。该量只描述“站在这个栅格上是否好走/是否太窄”，并不包含“从机器人是否能到达”的连通性信息；连通性由后续传播（§3.1.3）引入。

点通行性主要由两类几何量构成：

- clearance：到最近障碍距离 $d_{\text{obs}}(x)$；
- passage width：局部通道宽度 $w(x)$（通过射线测距估计，可 axis-aligned 或 adaptive）。

其中 $w(x)$ 的实现与参数关系如下：

- `max_raytrace_range_m`：射线最大长度（米），决定“测宽”观察范围；
- `include_diagonal_rays`：axis-aligned 宽度估计时是否包含对角方向（提高鲁棒性）；
- `adaptive_width=true` 时，先用 EDF 梯度 $\nabla d_{\text{obs}}$ 估计“离障方向”，再取其法向（切向）作为走廊方向，在该方向及其反向各做一次 raycast 得到宽度；最后取 `min(adaptive, axis_aligned)` 作为保守宽度。

同样使用：

$$
\mathrm{cont01}(\Delta;s)=\frac{\max(0,\Delta)}{\max(0,\Delta)+s}
$$

定义局部安全门槛：

$$
c_0=r_{\text{robot}}+d_{\text{safety,min}},\quad w_0=\texttt{width\_min}>0?\ \texttt{width\_min}:2c_0
$$

则点通行性（与实现的乘性与指数一致）可写为：

$$
\begin{aligned}
s_c(x) &= \mathrm{cont01}(d_{\text{obs}}(x)-c_0;\ \texttt{risk\_clearance\_scale\_m})\\
s_w(x) &= \mathrm{cont01}(w(x)-w_0;\ \texttt{risk\_width\_scale\_m})\\
r(x) &= \mathrm{clip}_{[0,1]}\big(s_c(x)^{\alpha_c}\cdot s_w(x)^{\alpha_w}\big)
\end{aligned}
$$

若当前格为障碍/未知（按 `treat_unknown_as_obstacle` 配置）、或 clearance/width 未超过门槛，则 $r(x)=0$。

### 3.1.2（补充）参数物理意义速查（局部层）

| 参数名 | 含义（物理意义） | 典型影响 |
|---|---|---|
| `robot_radius` | 机器人等效半径（m） | 越大越保守 |
| `safety_distance_min` | 最小安全裕度（m） | 提高后贴墙区域更低分 |
| `width_min` | 最小可通过宽度阈值 $w_0$（m） | 设大可避免钻缝，但更易判 0 |
| `risk_clearance_scale_m` | clearance 评分过渡尺度（m） | 越大越平滑 |
| `risk_width_scale_m` | width 评分过渡尺度（m） | 越大越不敏感 |
| `alpha_clearance/alpha_width` | clearance/width 指数 | 越大越强调该因素 |
| `adaptive_width` | 是否用 EDF 梯度自适应测宽 | 开启后更贴合走廊方向，抑制栅格别名 |
| `include_diagonal_rays` | axis-aligned 测宽是否含对角 | 开启后宽度估计更保守/更稳定 |
| `max_raytrace_range_m` | 测宽射线最大范围（m） | 过小会低估宽度，过大增算力 |
| `update_interval_s` | 局部可达性刷新周期（s） | 越大越省算力但反应慢 |

### 3.1.3 widest-path（maximin）传播得到机器人条件可达性

局部层随后以机器人所在栅格 $s$ 为源点，将 $r(x)$ 传播为机器人条件可达性：

$$
R_\ell(x)=\max_{p\in\mathcal{P}(s\to x)}\ \min_{y\in p}\ r(y)
$$

实现方式为优先队列（按当前最优 score 取最大）更新规则：

$$
\mathrm{candidate}=\min\big(R_\ell(\text{cur}),\ r(\text{next})\big)
$$

若 `candidate` 优于当前 `R_\ell(next)` 则更新并入队（对应 `updateCosts()` 中的 `relax()` 逻辑）。

### 3.1.4 与 local costmap 的耦合方式（写回/不写回）

与全局层一致：

- 默认 `write_to_master=false`：只更新缓存 `cache_reach_field_`，不改变 local costmap；
- `write_to_master=true`：把 $R_\ell$ 映射为代价写回 local costmap（只增不减）：

$$
c_{\text{reach}}(x)=\mathrm{round}\Big((1-R_\ell(x))\cdot \texttt{max\_cost}\Big)
$$

本仓库默认关闭写回，避免“偏好项”变成“近似障碍项”导致局部规划过度保守。

## 3.2 局部可达性规划器：MPPI 控制器（ReachabilityController）

### 3.2.1 控制器接口与运行位置

局部规划器插件为 `rpp::controller::ReachabilityController`（`nav_core::BaseLocalPlanner`）。其内部调用 MPPI 核心类 `ReachabilityDWA::findBestPath()` 计算速度指令：

- 输出：`cmd_vel.linear.x` 与 `cmd_vel.angular.z`
- 输入：机器人当前位姿/速度、全局路径（local_plan）以及 local costmap

### 3.2.2 MPPI 的采样、加权与更新（与实现一致）

定义控制序列：

$$
\mathbf{u}=\{(v_t,\omega_t)\}_{t=0}^{T-1}
$$

代码中对每个采样 $k\in\{1,\dots,K\}$、每个时刻 $t$，采样噪声：

$$
\epsilon^v_{k,t}\sim\mathcal{N}(0,\sigma_v^2),\quad \epsilon^{\omega}_{k,t}\sim\mathcal{N}(0,\sigma_{\omega}^2)
$$

并采用 clamp 后的控制量：

$$
	ilde v_{k,t}=\mathrm{clamp}(u^v_t+\epsilon^v_{k,t},\ v_{\min},v_{\max}),\quad
	ilde \omega_{k,t}=\mathrm{clamp}(u^{\omega}_t+\epsilon^{\omega}_{k,t},\ \omega_{\min},\omega_{\max})
$$

其中角速度上下界在实现中取对称边界：$\omega_{\min}=-|\texttt{max\_vel\_theta}|,\ \omega_{\max}=|\texttt{max\_vel\_theta}|$。

对每条 rollout 得到总代价 $S_k$（代码变量 `total`）。权重采用数值稳定形式：

$$
w_k\propto \exp\left(-\frac{S_k-\min_j S_j}{\lambda}\right),\quad \sum_k w_k=1
$$

控制更新采用带步长 $\alpha\in[0,1]$ 的形式（对应参数 `mppi_update_alpha`）：

$$
u_t\leftarrow \mathrm{clamp}\left(u_t+\alpha\sum_k w_k\,\epsilon_{k,t}\right)
$$

为避免符号歧义，这里各量的物理意义与实现对应关系为：

- $T$：时域步数（`mppi_horizon_steps`），总预测时长约为 $T\Delta t$；
- $K$：采样条数（`mppi_num_samples`）；
- $\sigma_v,\sigma_\omega$：采样噪声标准差（`mppi_noise_v_std / mppi_noise_w_std`），决定探索强度；
- $\lambda$：softmax 温度（`mppi_lambda`），越小越“只相信最优少数轨迹”，越大越“平均”；
- $\alpha$：控制序列更新步长（`mppi_update_alpha`），越大更新越激进、越小越平滑；
- $(v_{\min},v_{\max})$ 与 $(\omega_{\min},\omega_{\max})$：来自 `planner_util_->getCurrentLimits()`，实现对角速度使用对称边界 $\omega_{\min}=-|\texttt{max\_vel\_theta}|$。

### 3.2.2（补充）MPPI 代价权重的物理意义

MPPI 以“代价越小越好”为准则，代码中权重结构体 `w_` 与动态参数一一对应：

| 权重（实现变量） | 配置参数名 | 物理意义 |
|---|---|---|
| `w_.obstacle` | `mppi_obstacle_weight` | 避障代价（读取 costmap uchar 并归一化） |
| `w_.path` | `mppi_path_weight` | 到全局路径最近距离（米）的惩罚 |
| `w_.lookahead_goal` | `mppi_lookahead_weight` | 到 lookahead 点距离（米）的惩罚（鼓励沿路径前进） |
| `w_.heading` | `mppi_heading_weight` | 航向对准 lookahead 方向的角误差平方惩罚（rad$^2$） |
| `w_.path_heading` | `mppi_path_heading_weight` | 航向对准路径切向的角误差平方惩罚（rad$^2$） |
| `w_.control` | `mppi_control_weight` | 控制能量 $v^2+\omega^2$ 的惩罚（随 $\Delta t$ 积分） |
| `w_.smooth` | `mppi_smooth_weight` | 控制增量平方惩罚（抑制左右摇摆与抖动） |
| `w_.forward` | `mppi_forward_weight` | 低前进速度惩罚（减少停走） |
| `w_.goal` | `mppi_goal_weight` | rollout 末端到局部计划终点距离（米）的惩罚 |
| `w_.reachability` | `reachability_scale` | 局部可达性软惩罚 $1-\bar R_\ell$ |

### 3.2.3 MPPI 代价函数构成（逐项对照实现）

对每条 rollout，总代价由以下项组成（均在 `reachability_mppi.cpp` 中出现）：

1) **越界/障碍硬惩罚**（近似不可行）：

- 若 `worldToMap` 失败（出图），则 `+1e6`；
- 若 costmap 代价达到 `INSCRIBED_INFLATED_OBSTACLE`（含膨胀后的致命区域），则 `+1e6`。

2) **障碍代价软项**：

$$
S\mathrel{+}=w_{\text{obs}}\cdot \frac{c(x_t)}{255}
$$

其中 $c(x_t)$ 为 costmap 栅格代价。

3) **路径距离项**（到全局路径的最近距离）：

$$
S\mathrel{+}=w_{\text{path}}\cdot d_{\text{path}}(x_t)
$$

其中 $d_{\text{path}}$ 在实现中通过遍历 `global_plan_` 取最小欧氏距离得到。

4) **lookahead 目标项**（鼓励沿计划前进）：

$$
S\mathrel{+}=w_{\text{look}}\cdot \|x_t-x_{\text{look}}\|
$$

其中 $x_{\text{look}}$ 由 `mppi_lookahead_dist` 在当前路径上向前“行进”得到。

5) **朝向项**（面向 lookahead 点）：

$$
S\mathrel{+}=w_{\text{head}}\cdot \big(\Delta\theta(\theta_t,\ \mathrm{atan2}(y_{\text{look}}-y_t, x_{\text{look}}-x_t))\big)^2
$$

6) **路径切向朝向项**（帮助拐弯提前转向）：

$$
S\mathrel{+}=w_{\text{phead}}\cdot \big(\Delta\theta(\theta_t,\theta_{\text{tangent}})\big)^2
$$

其中 $\theta_{\text{tangent}}$ 为机器人附近全局路径的切向方向（由邻近两点方向计算）。

7) **控制能量项**：

$$
S\mathrel{+}=w_{\text{ctrl}}\cdot (v_t^2+\omega_t^2)\,\Delta t
$$

8) **平滑项**（抑制抖动）：

$$
S\mathrel{+}=w_{\text{smooth}}\cdot\big((v_t-v_{t-1})^2+(\omega_t-\omega_{t-1})^2\big)
$$

9) **前进项**（减少停走）：

代码对低前进速度施加惩罚，等价于：

$$
S\mathrel{+}=w_{\text{fwd}}\cdot\Big(1-\mathrm{clip}_{[0,1]}(\max(0,v_t)/v_{\max})\Big)^2
$$

10) **终端目标项**（靠近路径末端/目标）：

$$
S\mathrel{+}=w_{\text{goal}}\cdot \|x_T-x_{\text{goal}}\|
$$

### 3.2.4 局部可达性 critic 的融入方式（软 + 硬，严格对应实现）

MPPI 同时使用 local costmap 与局部可达性缓存 $R_\ell$，其作用分为两层：

1) **硬阈值（近似不可行）**

若轨迹上某栅格 $R_\ell(x_t) < \texttt{min\_reachability}$，则实现对该采样轨迹施加 `+1e6` 惩罚（等价于强约束/近似不可行）。此外 `checkTrajectory()`（用于 stop-rotate 逻辑的一步可行性检查）也使用相同阈值直接返回 false。

实现细节补充：只有当 `LocalReachabilityLayer` 缓存有效（`reach_cache_valid_==true`）且索引合法时，以上硬阈值与软惩罚才会启用；若缓存无效，控制器会自动退化为“不使用可达性 critic”的标准 MPPI 代价。

2) **软惩罚（偏好更“好走”的走廊）**

实现对整条轨迹聚合 $R_\ell$：

$$
\bar R_\ell=
\begin{cases}
\min_t R_\ell(x_t), & \texttt{reachability\_use\_min}=\texttt{true}\\
\frac{1}{N}\sum_t R_\ell(x_t), & \texttt{reachability\_use\_min}=\texttt{false}
\end{cases}
$$

并在 rollout 结束后加入：

$$
S\mathrel{+}=w_{\text{reach}}\cdot (1-\bar R_\ell)
$$

其中 $w_{\text{reach}}$ 对应参数 `reachability_scale`。

### 3.2.5 局部规划器与 local costmap 的分工

局部规划中的“约束/偏好”可总结为：

- **避障硬约束**：来自 local costmap（含 inflation），当代价达到 `INSCRIBED_INFLATED_OBSTACLE` 直接判不可行；
- **通行性/瓶颈约束与偏好**：来自 $R_\ell$（局部可达性缓存），可通过 `min_reachability` 形成硬阈值，并通过 `reachability_scale` 形成软偏好。

这使得系统能同时做到：不碰撞 + 不钻窄缝 + 更偏好走廊中心。

### 3.2.6 轨迹点云可视化（用于实验展示）

MPPI 发布 `/trajectory_cloud`（PointCloud2），字段包含 `x,y,z,rgb,intensity`：

- 采样轨迹：浅灰 RGB(180,180,180)，`intensity=0`；
- 当前执行（nominal）轨迹：红色 RGB(255,0,0)，`intensity=1`。

该可视化适合在 RViz 中用于展示“采样分布与最终选择”，并可用于论文中的实验图。

---

# 附录 A 复现实验的关键参数入口（仓库对照）

- 全局规划器参数：`src/sim_env/config/planner/reachability_planner_params.yaml`
	- `min_reachability, cost_weight, reachability_weight, step_base_cost`
- 局部控制器参数：`src/sim_env/config/controller/reachability_controller_params.yaml`
	- MPPI：`mppi_num_samples, mppi_horizon_steps, mppi_dt, mppi_lambda, mppi_noise_*`
	- 代价权重：`mppi_*_weight`
	- 可达性 critic：`reachability_scale, min_reachability, reachability_use_min`
- costmap 插件顺序：
	- `src/sim_env/config/costmap/global_costmap_plugins.yaml`
	- `src/sim_env/config/costmap/local_costmap_plugins.yaml`


