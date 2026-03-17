\newcommand{\R}{\mathbb{R}}

---

# **基于阳光射线采样与分层轨迹优化的移动机器人运动规划框架**

**A Hierarchical Motion Planning Framework Based on Sunshine Ray Sampling and Layered Trajectory Optimization**

**作者**：Zhang Dingkun  
**日期**：2026 年 2 月  
**研究方向**：移动机器人运动规划，采样式规划，轨迹优化与分层控制  

---

## **摘要**

本文提出了一套针对二维栅格环境的分层运动规划与轨迹优化框架。该框架包含三个递阶的优化层：**全局规划层**、**全局路径优化层**和**局部规划与控制层**。

**全局规划层**采用阳光射线采样（Sunshine Ray Sampling）方法，通过 360° 光照式射线测距与几何切点检测，高效识别环境关键特征点，结合 A* 搜索与路径缩短技术生成初始可行路径 $\pi_0$。

**全局路径优化层**采用共轭梯度法（Conjugate Gradient）在全局路径的基础上进行快速平滑优化，输出平滑的全局参考路径 $\pi^*$，适合实时性要求高的应用场景。

**局部规划与控制层**采用迭代线性二次型调节器（iLQR）进行轨迹优化与控制，考虑完整的运动学约束、安全走廊（Convex Safety Corridor）与欧氏符号距离场（ESDF）代价，输出可直接执行的控制命令 $(v, \omega)$，适合精度要求高的应用场景。

三层通过事件触发式重规划策略耦合，使得任何层的规划失败都可快速回退至前一层进行重规划。本文详细推导了各算法的数学原理，包括共轭梯度优化的收敛性分析、iLQR 的反向递推与值函数构造、走廊约束与 ESDF 代价的梯度计算，并对所有公式与流程进行了代码级别的验证，确保论文描述与仓库实现的严格一致性。

**关键词**：运动规划，采样式规划，轨迹优化，分层框架，共轭梯度，iLQR，事件触发重规划

---

## **前言**

### 背景与意义

移动机器人的路径规划是自主导航的核心问题。在二维静态栅格地图环境中，传统方法（如 A* 搜索）虽然理论上最优，但在大规模高分辨率地图上需要扩展大量栅格邻域，计算复杂度难以接受。采样式规划方法通过"关键几何特征采样"来减少搜索空间维度，通过几何启发（如可视性、切点等）精选代表性候选点，在这些候选点间执行图搜索，从而降低时间复杂度。

全局规划输出的离散栅格路径往往在连续控制空间中沿栅格边界游走，缺乏平滑性，易导致蛇形运动和转弯困难。**轨迹优化**是解决这一矛盾的关键一步。本仓库采用**分层框架**解决此问题：

- **全局规划层**：Sunshine 采样规划器快速生成拓扑可行的初始路径 $\pi_0$（毫秒级），无需考虑动力学约束；
- **全局路径优化层**：共轭梯度法对初始路径进行快速平滑优化，输出 $\pi^*$（实时性优先）；
- **局部规划与控制层**：iLQR 算法实现轨迹优化与控制，输出控制命令 $(v, \omega)$（精度优先）；
- **重规划策略**：当局部规划失败时立即触发全局重规划，实现两层高效协作。

本文的工作在于：
- 系统阐述分层规划框架的设计理念与代码实现；
- 详细推导全局规划（Sunshine）、路径优化与轨迹优化（共轭梯度与 iLQR）的数学原理；
- 说明共轭梯度作为**全局路径优化工具**与 iLQR 作为**局部规划与控制方案**的不同角色；
- 通过走廊约束与 ESDF 代价的结合，提升规划的安全裕度与数值稳定性；
- 对所有公式与流程进行代码级别的验证，确保论文与实现的严格一致性。

### 本文结构

本文共分为 11 章，遵循从全局到局部、从规划到控制的递进逻辑：

**第一部分：全局规划层（第 1-7 章）**
- 第 1-7 章详述 Sunshine 采样式规划算法的原理、实现与参数调优；
- 输出初始的栅格路径 $\pi_0$。

**第二部分：全局路径优化层（第 8B 章）**
- 第 8B 章介绍共轭梯度法用于全局路径 $\pi_0$ 的快速平滑优化；
- 输出平滑优化后的全局参考路径 $\pi^*$，适合实时性要求高的场景。

**第三部分：局部规划与控制层（第 8 章）**
- 第 8 章详述 iLQR 算法用于局部轨迹的精细优化与控制；
- 考虑完整的动力学约束、走廊约束与 ESDF 代价；
- 输出可执行的控制命令 $(v, \omega)$，适合精度要求高的场景。

**第四部分：约束与框架（第 9-11 章）**
- 第 9 章阐述走廊约束与 ESDF 代价的原理与梯度计算；
- 第 10 章说明分层框架的耦合机制与事件触发重规划策略；
- 第 11 章总结论文贡献与提出题目建议。

---

## **实现对应说明与分层架构**

本论文遵循**分层规划框架**，包含两个独立但相互协作的优化层，严格对应仓库代码实现。

### 分层架构详解

```
输入：起点 start，目标点 goal，costmap

├─ 全局规划层（Global Planning Layer）
│  ├─ Sunshine 采样规划
│  │  └─ 输出：初始栅格路径 π₀
│  └─ 对应代码：sunshine_planner.cpp（第 1-7 章）
│
├─ 全局路径优化层（Global Path Optimization Layer）
│  ├─ 共轭梯度法（Conjugate Gradient）
│  │  ├─ 输入：初始路径 π₀，ESDF 代价场
│  │  └─ 输出：平滑全局参考路径 π*（实时性优先）
│  └─ 对应代码：conjugate_optimizer.cpp（第 8B 章）
│
├─ 局部规划与控制层（Local Planning & Control Layer）
│  ├─ iLQR 轨迹优化
│  │  ├─ 输入：参考路径 π*（或 π₀），当前状态 x₀
│  │  ├─ 约束：走廊约束、ESDF 代价、动力学约束
│  │  └─ 输出：可执行轨迹 τ（精度优先）
│  └─ 对应代码：ilqr_controller.cpp（第 8 章）
│
└─ 重规划策略（Event-Triggered Replanning）
   ├─ 触发条件：局部规划失败（controller_patience）
   └─ 对应参数：move_base_params.yaml（第 10 章）
```

### 关键区别

| 层级 | 算法 | 输入 | 输出 | 适用场景 | 约束 |
|-----|------|------|------|--------|------|
| **全局规划** | Sunshine | 地图 | 栅格路径 π₀ | 高分辨率地图 | 无碰撞 |
| **全局优化** | 共轭梯度 | 栅格路径 π₀ | 光滑路径 π* | 实时性要求高 | 无碰撞 + 光滑 |
| **局部规划与控制** | iLQR | 光滑路径 π* | 控制命令 (v,ω) | 精度要求高 | 动力学 + 安全走廊 + ESDF |

### 代码对应清单

为确保论文的代码可验证性，所有公式与流程均以以下代码文件为准：

**全局规划层相关：**
- Sunshine 算法实现：`src/core/path_planner/path_planner/src/sample_planner/sunshine_planner.cpp`
- Sunshine 接口定义：`src/core/path_planner/path_planner/include/path_planner/sample_planner/sunshine_planner.h`
- 全局规划节点（坐标转换、参数传递）：`src/core/path_planner/path_planner/src/path_planner_node.cpp`
- 参数配置：`src/sim_env/config/planner/sample_planner_params.yaml`

**全局路径优化（共轭梯度）：**
- 共轭梯度优化器实现：`src/core/trajectory_planner/src/trajectory_optimization/conjugate_optimizer/conjugate_optimizer.cpp`
- 接口定义：`src/core/trajectory_planner/include/trajectory_planner/trajectory_optimization/conjugate_optimizer/conjugate_optimizer.h`
- 参数配置：`src/sim_env/config/trajectory_optimization/conjugate_gradient_params.yaml`

**局部规划与控制（iLQR）：**
- iLQR 控制器实现：`src/core/controller/ilqr_controller/src/ilqr_controller.cpp`
- iLQR 接口定义：`src/core/controller/ilqr_controller/include/controller/ilqr_controller.h`
- 参数配置：`src/sim_env/config/controller/ilqr_controller_params.yaml`

**约束与重规划：**
- 走廊与 ESDF（在 iLQR 中）：`ilqr_controller.cpp` 的代价函数部分
- 分层框架参数：`src/sim_env/config/move_base_params.yaml`

---

# 第一章 引言与问题定义

## 1.1 分层规划框架的设计理念

本文采用的运动规划系统采用**三层递阶式设计**，如图 1-1 所示：

```
┌──────────────────────────────────────────────────────────────┐
│  全局规划层：Sunshine 采样规划                               │
│  ├─ 输入：costmap, start, goal                              │
│ │  └─ 输出：初始栅格路径 π₀（拓扑可行，无碰撞）            │
│  ├─ 特点：几何启发、快速、无动力学约束                      │
│  └─ 时间尺度：200-500 毫秒                                  │
└──────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│  全局路径优化层：共轭梯度法优化                              │
│  ├─ 输入：栅格路径 π₀，ESDF 代价场                         │
│  ├─ 输出：平滑优化路径 π*（连续坐标，轻量级优化）          │
│  ├─ 特点：快速收敛、低计算成本、适合实时性                  │
│  └─ 时间尺度：30-100 毫秒（100 次迭代）                    │
└──────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│  局部规划与控制层：iLQR 轨迹优化与控制                       │
│  ├─ 输入：平滑路径 π*，当前状态 x₀，走廊+ESDF              │
│  ├─ 输出：可执行轨迹 τ（速度 v, 角速度 ω）                │
│  ├─ 特点：精细化、安全约束完整、高质量轨迹                  │
│  └─ 时间尺度：20-50 毫秒（10-20 次迭代）                   │
└──────────────────────────────────────────────────────────────┘
                           ↓
                 机器人执行控制命令
```

**关键设计理念**：

1. **关注点分离（Separation of Concerns）**：三层各自解决不同尺度的问题
   - 全局层：拓扑可行性与全局路径搜索
   - 全局优化：路径平滑性与无碰撞（轻量级）
   - 局部层：轨迹质量、动力学约束、安全控制与精细安全

2. **计算效率**：各层独立可扩展
   - Sunshine：大地图高效搜索（采样式）
   - 共轭梯度：快速平滑（仅梯度计算）
   - iLQR：精细优化（考虑动力学）

3. **鲁棒性**：多级备选方案
   - 若共轭梯度不收敛，使用原始 π₀
   - 若 iLQR 失败，触发全局重规划
   - 事件触发式重规划保证不会陷入死循环

## 1.2 分层中各算法的角色

### 全局规划层：Sunshine 采样式规划

Sunshine 规划器是一个**几何启发式采样规划器**，核心思想是通过 360° 光照式射线测距识别环境的关键几何特征（切点、转折点等），而不是均匀网格采样。这样做的好处是：

- **特征识别**：射线自由长度的突变表明存在几何特征点（障碍转角）
- **采样高效**：只在几何特征处采样，减少候选点数量
- **图搜索**：在采样点间执行 A* 搜索，快速找到拓扑可行路径

**输出特点**：
- 坐标为栅格坐标 $(x,y)\in\mathbb{Z}^2$
- 路径为离散点序列 $\pi_0 = \{c_0, c_1, \ldots, c_m\}$
- 无碰撞但可能不光滑（沿栅格边界游走）

### 全局路径优化层：共轭梯度法

共轭梯度法作为**全局路径的优化工具**（不是独立的局部规划器），其任务是：

- **输入**：Sunshine 输出的离散栅格路径 $\pi_0$
- **优化目标**：在保证无碰撞的前提下，平滑路径、减少曲率、提升路径质量
- **方法**：梯度下降法迭代优化路径点坐标，代价函数包括
  - 障碍规避代价（基于 ESDF）
  - 路径平滑代价（相邻点间距约束）
  - 曲率代价（转弯平滑度）

**特点**：
- 快速收敛（通常 50-100 迭代）
- 低计算成本（仅梯度计算，无 Hessian）
- 适合实时性要求高、计算资源受限的场景
- **不考虑机器人动力学**，仅执行几何平滑

**应用场景**：
- 机器人仅需原始路径的平滑版本，无需精细控制
- 实时规划系统要求端到端延迟 < 200 ms
- 嵌入式环境计算资源受限

### 局部规划与控制层：iLQR 控制器

iLQR 是**局部规划与控制的精细方案**，负责在全局平滑路径的基础上，生成满足动力学约束、安全约束的可执行控制命令。

**输入**：
- 平滑参考路径 $\pi^*$（由共轭梯度或 Sunshine 输出）
- 当前机器人状态 $\boldsymbol{x}_0 = (x, y, \theta)^T$
- 安全走廊约束（convex safety corridor）
- ESDF 障碍代价场

**任务**：
- 通过反向递推与前向迭代，优化控制序列 $\{\boldsymbol{u}_0, \boldsymbol{u}_1, \ldots\}$
- 生成平滑的轨迹点序列 $\boldsymbol{x}_k = (x_k, y_k, \theta_k)^T$
- 满足运动学约束 $\boldsymbol{x}_{k+1} = f(\boldsymbol{x}_k, \boldsymbol{u}_k, \Delta t)$
- 满足安全约束（走廊内、距离障碍足够远）

**输出**：
- 第一步的控制命令 $(v_0, \omega_0)$ 立即发送给机器人
- 后续预测轨迹用于可视化与决策

**特点**：
- 考虑完整的运动学约束
- 安全约束细致（走廊 + ESDF）
- 优化质量高，输出轨迹光滑可执行
- 计算成本相对较高（涉及 Hessian 与线搜索）

**应用场景**：
- 精度要求高的精细轨迹
- 有充分计算资源的高性能平台
- 需要满足严格动力学约束的复杂环境

## 1.3 分层间的协作机制

三层通过**事件触发式重规划策略**耦合：

```
全局规划  ←─────────────────────────┐
    ↓                               │
   π₀                        规划失败
    ↓                        （触发）
共轭梯度优化                        │
    ↓                               │
   π*                               │
    ↓                               │
  iLQR 优化 ─────────────→ 轨迹碰撞/失败
    ↓
 控制命令 (v, ω)
```

**重规划条件**（由 move_base 框架自动管理）：

1. **局部规划失败**：iLQR 在 `controller_patience = 0.2` 秒内无法产生有效轨迹
2. **振荡检测**：机器人在过去 10 秒内移动距离小于 0.2 米
3. **第一次规划**：系统启动时的初始规划

## 1.4 为什么采用分层框架？

| 问题 | 单层方案 | 分层方案 |
|------|--------|--------|
| 全局规划计算量大 | 难以实时 | Sunshine 快速，采样式高效 |
| 路径不光滑 | 需大量后处理 | 共轭梯度快速平滑 |
| 轨迹质量需求不同 | 一个方案难以两全 | 共轭梯度（快）+ iLQR（精） |
| 局部规划失败恢复 | 整体重来浪费计算 | 仅重规划全局，快速恢复 |
| 参数调优 | 参数耦合复杂 | 各层参数独立可调 |

## 1.5 本文的"实现一致性"约束

重要说明：本文严格对标仓库代码实现，**不引入代码中不存在的量**。

- 本实现的距离计算仅用二维欧氏距离（`hypot`），不读取 costmap 的非致命代价作为软惩罚；
- 可行性判定由"占据 + 可视线（Bresenham 栅格线遍历）"构成；
- "切点（tangent）"由相邻射线自由长度的差异阈值检测得出；
- 后端路径优化为确定性的几何操作（中点与二分逼近），不保证最优性。

## 1.6 论文组织与代码对应

本论文严格对应仓库代码，按分层框架组织：

| 章节 | 内容 | 对应代码文件 |
|-----|------|-----------|
| 第 1 章 | 引言与分层框架 | 整体架构说明 |
| 第 2-7 章 | Sunshine 全局规划算法 | `sunshine_planner.cpp` |
| 第 8B 章 | 共轭梯度法（全局路径优化） | `conjugate_optimizer.cpp` |
| 第 8 章 | iLQR 算法（局部规划与控制） | `ilqr_controller.cpp` |
| 第 9 章 | 走廊约束与 ESDF 代价 | `ilqr_controller.cpp` 的约束部分 |
| 第 10 章 | 分层框架与事件触发重规划 | `move_base_params.yaml` + `path_planner_node.cpp` |
| 第 11 章 | 总结与建议 | 论文贡献总结 |

---

# 第二章 符号、栅格与可行性判定（与实现一致）

## 2.1 栅格坐标与节点定义

规划器工作在 costmap 栅格坐标中（单位：cell），并使用整型节点：

- 节点 $n$ 的坐标为 $(x_n, y_n)\in\mathbb{Z}^2$；
- 通过 `grid2Index(x,y)` 映射到一维索引 $\mathrm{id}(n)$；
- `pid` 记录父节点的 `id`（若为起点则 `pid=-1`）；
- 代价 $g(n)$ 表示从起点到该节点的累计路径长度（单位：cell）。

实现中节点类型为 `rpp::common::structure::Node<int>`，其 `x/y/id/pid/g` 语义如上。

## 2.2 占据判定（isOccupied）

对任意栅格 $(x,y)$：

- 若越界，则视为占据（不可通行）；
- 否则读取 costmap 的 `charMap` 代价 $c(x,y)$，并以阈值 `LETHAL_OBSTACLE * factor_` 判定占据。

可写为：

$$
\mathrm{occupied}(x,y)=
\begin{cases}
\texttt{true}, & (x,y) \notin [0,n_x)\times[0,n_y)\\
\big(c(x,y) \ge \texttt{LETHAL\_OBSTACLE}\cdot \texttt{factor}\big), & \text{otherwise}
\end{cases}
$$

其中 `factor` 在 `PathPlannerNode` 中由 `obstacle_factor` 参数读入并传递给规划器（`g_planner_->setFactor(factor_)`）。

## 2.3 可视线（LOS）判定

两点间可视线判定 `hasLineOfSight(a,b)` 使用 Bresenham 栅格线段遍历：若线段上任一点占据，则 LOS 为 false。

记 $\mathcal{B}(a,b)$ 为 Bresenham 生成的栅格序列，则：

$$
\mathrm{LOS}(a,b)= \neg \exists p\in\mathcal{B}(a,b),\ \mathrm{occupied}(p)
$$

---

# 第三章 360° 光照射线与切点（tangent）候选生成

**代码对应**：`SunshinePathPlanner::findTangents()` 函数（sunshine_planner.cpp, 第 104-156 行）

本章的算法对应代码实现中的关键步骤。为确保学术严谨性，每个公式、参数与判断条件都直接取自代码逻辑。

## 3.1 射线自由长度（raycastFreeLength）

给定起点栅格 $(x_0,y_0)$ 与角度 $\theta$，以固定步长 `length_step` 沿方向 $(\cos\theta,\sin\theta)$ 前进，直到：

- 遇到越界；或
- 遇到占据栅格；或
- 达到最大长度 `max_ray_length`。

实现中累计长度（单位：cell）为 `traveled`，每步采样点为

$$
(x,y)=\big(\mathrm{round}(x_0+\ell\cos\theta),\ \mathrm{round}(y_0+\ell\sin\theta)\big)
$$

其中 $\ell$ 从 `length_step` 开始累加，使用 `lround`（四舍五入）。最终返回自由长度 $L(\theta)=\ell$。

## 3.2 角度离散化与射线条数确定（与实现一致）

射线的角度离散化是采样式规划中的一个关键设计参数，直接影响方向覆盖率与计算效率。本实现采用**等分离散策略**（而非简单的固定步长递增）以保证 360° 的均匀覆盖。

**射线条数计算**：

根据输入参数 `theta_step`（弧度，典型值 ~0.1745 rad ≈ 10°）计算射线条数：

$$
N=\max\big(8,\lceil 2\pi / \texttt{theta\_step}\rceil\big)
$$

其中 $\max(8,\cdots)$ 保证至少 8 条射线的最小覆盖。这样设计的原因在于：
- 若 `theta_step` 很小（例如 $<0.01$ rad），自动阈值为 8 条，避免过度采样导致计算冗余；
- 若 `theta_step` 较大（例如 0.2 rad），则等分条数由 $\lceil 2\pi/0.2\rceil = 32$ 条决定。

**精确等分**：

随后使用**等角间隔**的精确值：

$$
\theta_i = i\cdot \frac{2\pi}{N},\quad i=0,1,\dots,N-1
$$

这保证了射线在全方位的均匀分布，不会因为浮点误差或步长选择而出现某个方向缺失的情况。对于每条射线方向 $\theta_i$，实现调用 `raycastFreeLength(x_sun, y_sun, \theta_i)` 计算该方向的自由长度 $L_i=L(\theta_i)$，最终得到完整的长度数组 $\mathbf{L} = [L_0, L_1, \ldots, L_{N-1}]$，供后续切点检测使用。

## 3.3 切点检测（相邻射线长度突变）

对相邻射线 $(i,j)$，其中 $j=(i+1)\bmod N$：

- 若 $|L_i-L_j| < \texttt{length\_diff\_threshold}$，则认为该处不产生切点；
- 否则认为存在“角点/切点”迹象，并选择更长射线方向作为候选方向。

设 $L_{\text{long}}=\max(L_i,L_j)$，$L_{\text{short}}=\min(L_i,L_j)$，实现选择的候选长度为：

$$
\ell^* = \min\big(L_{\text{short}} + \texttt{forward\_distance},\ L_{\text{long}}\big)
$$

若 $\ell^*<1$ 则跳过。候选栅格点为：

$$
(x_c,y_c)=\big(\mathrm{round}(x_s+\ell^*\cos\theta_{\text{long}}),\ \mathrm{round}(y_s+\ell^*\sin\theta_{\text{long}})\big)
$$

其中 $(x_s,y_s)$ 为当前 sun 节点坐标，$\theta_{\text{long}}$ 为更长射线对应的方向。

最终候选集合会按 `id` 去重（`unordered_set`）。

---

# 第四章 A* 风格的 Sunshine 搜索（ChooseSun）

## 4.1 启发式与代价定义

实现使用二维欧氏距离作为启发式与边代价：

- 启发式：
$$
 h(n) = \sqrt{(x_n-x_g)^2+(y_n-y_g)^2}
$$

- 边代价（sun 到 cand）：
$$
 c(n,\mathrm{cand}) = \sqrt{(x_n-x_c)^2+(y_n-y_c)^2}
$$

- 累计代价：
$$
 g(\mathrm{cand}) = g(n) + c(n,\mathrm{cand})
$$

- OpenSet 的优先级：
$$
 f(\mathrm{cand}) = g(\mathrm{cand}) + h(\mathrm{cand})
$$

OpenSet 使用最小堆（`priority_queue` + `Greater`）按 $f$ 取最小。

## 4.2 初始化

- 将输入的 start/goal（`Point3d`）按 `lround` 转为整型 cell；
- 再 clamp 到地图范围 $[0,n_x-1]\times[0,n_y-1]$；
- 若起点或终点占据则直接失败；
- 起点入 OpenSet，初始 $g(start)=0$，$f(start)=h(start)$。

注意：该实现假定外部（`PathPlannerNode`）已把世界坐标转换为 costmap cell 坐标后再调用 `plan()`。

## 4.3 主循环与 ClosedSet

循环条件：迭代次数小于 `max_iterations` 且 OpenSet 非空。

每次：

1) 从 OpenSet 取出最小 $f$ 的条目 `id`；
2) 若该 `id` 已在 ClosedSet（`closed_`）则跳过；
3) 将其加入 ClosedSet，并把该 sun 记录到 `expand`（用于可视化）：

- `expand.emplace_back(sun.x(), sun.y(), sun.pid())`

这里第三个分量 `theta()` 被上层用于存储父节点 `pid`（见 `path_planner_node.cpp` 的 tree 可视化逻辑）。

## 4.4 直接照射到目标（goal 可视）

若当前 sun 与 goal 之间有 LOS，则认为存在一条候选解：

$$
J = g(\mathrm{sun}) + \|\mathrm{sun}-\mathrm{goal}\|_2
$$

实现会记录当前最小的 `best_goal_cost` 与对应的 `best_goal_parent`，但不会立刻停止搜索。

## 4.5 候选切点的过滤：NotBrother 与 NotOtherSon

对每个切点候选 `cand`，需依次满足：

1) **sun 到 cand 可视（硬约束）**：`LOS(sun, cand)`。

2) **NotBrother（避免“父可直连”）**：

若 sun 有父节点 $p$（`pid>=0`），并且 `LOS(p, cand)` 为真，则丢弃该候选。

该规则对应实现 `isBrotherCandidate(sun,cand)`，其语义是：如果父节点能直接看到 cand，那么通过 sun 再连接 cand 在几何上更“绕”，因此直接拒绝。

3) **NotOtherSon（避免“被其它 open 更便宜地领养”）**：

实现 `isOtherSonCandidate(sun, cand, cand_g)` 会遍历当前 `nodes_` 中所有“未关闭”的节点 other，判断是否存在另一个节点能以不高于 `cand_g` 的代价并且 LOS 连接到 cand。若存在则拒绝。

为减少 LOS 检测次数，实现先用下界剪枝：

$$
 g(\mathrm{other}) + \|\mathrm{other}-\mathrm{cand}\|_2 > g(\mathrm{cand})\ \Rightarrow\ \text{不可能更优，跳过}
$$

若通过剪枝且 `LOS(other,cand)`，则计算

$$
 g'(\mathrm{cand}) = g(\mathrm{other}) + \|\mathrm{other}-\mathrm{cand}\|_2
$$

若 $g'(\mathrm{cand}) \le g(\mathrm{cand})$ 则拒绝。

4) **best-g 改进（去劣）**：

若 `best_g_[cand.id]` 已存在且不大于 `cand_g`，则拒绝；否则接受并更新：

- `cand.pid = sun.id`，`cand.g = cand_g`
- `nodes_[cand.id] = cand`，`best_g_[cand.id] = cand_g`
- OpenSet 推入 `(f=cand_g+h(cand), id=cand.id)`。

## 4.6 路径回溯与输出

当搜索结束后：

- 若未找到任何 `best_goal_parent`（goal 不可见于任何扩展 sun），则失败；
- 否则回溯：输出序列为 `[goal, ..., start]` 再反转。

实现 `backtracePath(best_goal_parent, start, goal)`：

- 先压入 goal；
- 从 `best_goal_parent` 开始，沿 `pid` 在 `nodes_` 中向上追溯，直到到达 start 或断链；
- 若最终未包含 start，则强制把 start 压入；
- 最后反转得到从 start 到 goal 的顺序。

输出 `path` 为 costmap cell 坐标（`Points3d`），其中 `z/theta` 固定为 0，不含朝向。

---

# 第五章 双向二分后端优化（与实现一致）

后端优化仅在满足以下条件时启用：

- `enable_bidirectional_opt=true` 且 `optimize_iterations>0`。

## 5.1 Forward pass

对路径三元组 $(a,b,c)$（按前向遍历）：

- 若 `LOS(a,c)`，则将中间点 $b$ 替换为 midpoint：

$$
 b \leftarrow \left\lfloor\frac{a+c}{2}\right\rfloor
$$

（实现用整型除法得到 midpoint，并对 $x/y$ 做 clamp）。

- 否则执行“二分推点”：在保持 `LOS(a,b)` 的前提下，把 $b$ 尽量向 $c$ 推近。

实现 `bisectionTowards(anchor=a, from=b, to=c)`：

- 若 `!LOS(a, from)` 则返回 from；
- 若 `LOS(a, to)` 则返回 to；
- 否则进行固定 12 次二分：令 `mid = midpoint(lo,hi)`，若 `LOS(a,mid)` 则 `lo=mid` 否则 `hi=mid`，返回 `lo`。

## 5.2 Backward pass

Backward pass 与 Forward 对称：从尾到头对 $(a,b,c)$（其中 $c$ 在后）执行相同的 shortcut 或二分推点逻辑。

实现上为了避免 `size_t` 下溢，循环在 `k==2` 时 break。

## 5.3 多次迭代与去重

重复执行 `optimize_iterations` 次 forward+backward，并移除相邻重复点（consecutive duplicates）。

---

# 第六章 参数与工程接入（本仓库）

## 6.1 参数表（与实现一致）

参数由 `PathPlannerNode` 在 `planner_name_=="sunshine"` 分支读取，命名空间为 `~/<global_planner_name>/sunshine/*`（其中 `global_planner_name` 是 move_base 对该全局规划器插件实例的名字）。

| 参数名 | 代码字段 | 单位 | 作用（与实现一致） |
|---|---|---:|---|
| `max_iterations` | `Params::max_iterations` | 次 | OpenSet 弹出/扩展次数上限 |
| `theta_step` | `Params::theta_step` | rad | 用于计算射线条数 $N=\lceil 2\pi/\theta_{step}\rceil$ |
| `length_step` | `Params::length_step` | cell | 射线前进步长 |
| `length_diff_threshold` | `Params::length_diff_threshold` | cell | 相邻射线长度差阈值（触发切点检测） |
| `forward_distance` | `Params::forward_distance` | cell | 切点候选长度偏置：$\ell^*=L_{short}+d_{forward}$ |
| `max_ray_length` | `Params::max_ray_length` | cell | 射线最大长度 |
| `enable_bidirectional_opt` | `Params::enable_bidirectional_opt` | - | 是否启用后端二分优化 |
| `optimize_iterations` | `Params::optimize_iterations` | 次 | forward+backward 的迭代次数 |

## 6.2 与 `move_base` 的插件接入

本仓库将 Sunshine 作为 `path_planner` 包的采样类全局规划器之一，由 `PathPlannerNode` 统一对外导出为 `nav_core::BaseGlobalPlanner`。

- 在 `PathPlannerNode::initialize()` 中选择 `planner_name: sunshine` 即可构造 `SunshinePathPlanner`。
- 输出 `plan` 为 `nav_msgs/Path`，其来源是 `origin_path`（cell）经过 `map2World()` 转换。

## 6.3 可视化（与实现一致）

当 `expand_zone: true` 且属于 `SAMPLE_PLANNER` 时，`PathPlannerNode`：

- 将 `expand` 中每个点的 `theta()` 当作父节点索引（`pid`），并发布树边到 `random_tree`（ns=`tree`，统一颜色）；
- 将最终 `origin_path` 转为边并以另一颜色发布（ns=`tree_selected`），作为“选中的采样轨迹高亮”。

---

# 第七章 复杂度与实现注意事项

## 7.1 复杂度（按实现结构）

- 每次扩展 sun，需要做一次 `findTangents()`：其代价约为 $O(N\cdot L)$，其中 $N$ 为射线条数，$L$ 与 `max_ray_length/length_step` 成正比；
- 对每个候选切点，`NotOtherSon` 会遍历所有未关闭节点并可能做 LOS 检测，最坏情况下较重（与 OpenSet 大小相关）。

因此该实现更像“几何启发的采样 A*”，并未针对大规模场景做严格的次线性优化。

## 7.2 单位与坐标

- Sunshine `plan()` 的输入输出均是 **costmap cell 坐标**；
- 世界坐标到 cell 的转换由 `PathPlannerNode` 的 `world2Map()` 完成；
- 文档中的距离单位（若未特别说明）均与实现一致：在 planner 内以 cell 为单位。

---

# 附录 A：与实现一致的伪代码

```text
Input: start_cell, goal_cell
Init: open = {start}, best_g[start]=0, closed=∅
best_goal_cost=+∞, best_goal_parent=-1

for iter in [0, max_iterations):
  if open empty: break
  sun = pop_min_f(open)
  if sun in closed: continue
  closed.add(sun)
  expand.push((sun.x, sun.y, sun.pid))

  if LOS(sun, goal):
    best_goal_cost = min(best_goal_cost, g(sun)+dist(sun,goal))
    best_goal_parent = argmin

  candidates = TangentsByRayDiscontinuity(sun)
  for cand in candidates:
    if !LOS(sun,cand): continue
    cand_g = g(sun) + dist(sun,cand)
    if ParentExists(sun) and LOS(parent(sun), cand): continue   # NotBrother
    if ExistsOtherOpenNodeConnectsCheaper(cand, cand_g): continue # NotOtherSon
    if best_g[cand] <= cand_g: continue

    parent(cand)=sun
    best_g[cand]=cand_g
    push(open, f=cand_g+dist(cand,goal), cand)

if best_goal_parent < 0: fail
path = Backtrace(best_goal_parent, start, goal)
path = BidirectionalBisectionOptimize(path)
return path, expand
```

---

# 附录 B：参数配置片段

对应 `src/sim_env/config/planner/sample_planner_params.yaml`：

```yaml
PathPlanner:
  sunshine:
    max_iterations: 20000
    theta_step: 0.17453292519943295
    length_step: 1.0
    length_diff_threshold: 8.0
    forward_distance: 6.0
    max_ray_length: 200.0
```

---

# 第八章 局部规划与控制：迭代线性二次型调节器（iLQR）

**本章角色**：此章介绍**局部规划与控制层**，与第 8B 章的全局路径优化（共轭梯度）**处于不同的分层级别**。

iLQR 是局部规划与控制的精细方案，负责在全局平滑参考路径的基础上，通过反向递推与前向线搜索，生成满足运动学约束、安全约束的可执行控制命令。该方法考虑完整的动力学特性、走廊约束与 ESDF 代价，适合**精度要求高、计算资源充足**的应用场景。

**与全局优化层的区别**：全局优化（共轭梯度）处理的是离散栅格路径的几何平滑；本章 iLQR 则在平滑路径基础上进行**轨迹质量优化**，同时满足运动学约束与精细的安全约束，最终生成高频率（10 Hz）的控制命令。

本章基于仓库 iLQR 局部规划实现，严格对应以下文件：

- iLQR 控制器实现：`src/core/controller/ilqr_controller/src/ilqr_controller.cpp` （1141 行）
- 接口定义：`src/core/controller/ilqr_controller/include/controller/ilqr_controller.h`
- 参数配置：`src/sim_env/config/controller/ilqr_controller_params.yaml`

**代码对应**：
- `solveILQR()` 函数（ilqr_controller.cpp, 第 907 行）：完整的 iLQR 求解器
- `evaluateTrajectoryCost()` 函数：轨迹代价评估与梯度计算
- `stepDynamics()` 函数：离散动力学前向传播
- `buildReference()` 函数：参考轨迹采样与去重

## 8.1 iLQR 的问题设置与架构概述

### 8.1.1 状态、控制与离散动力学（实现一致）

状态与控制向量在实现中被显式定义为固定维度：

$$
\boldsymbol{x}_k=[x_k,y_k,\theta_k,v_{k-1},\omega_{k-1}]^T,\quad
\boldsymbol{u}_k=[v_k,\omega_k]^T
$$

其中 $(x_k,y_k,\theta_k)$ 为机器人位姿，$v_{k-1},\omega_{k-1}$ 为上一时刻控制量，目的是在代价函数中惩罚控制变化（实现中的 `w_control_change` 或 `w_du`）。

离散运动学模型（对应 `stepDynamics` 函数）：

$$
\begin{aligned}
x_{k+1} &= x_k + v_k\cos\theta_k\,\Delta t \\
y_{k+1} &= y_k + v_k\sin\theta_k\,\Delta t \\
	heta_{k+1} &= \theta_k + \omega_k\Delta t \\
v_{k} &\rightarrow v_{k-1} \\
\omega_{k} &\rightarrow \omega_{k-1}
\end{aligned}
$$

其中 $\Delta t$ 由 `controller_frequency` 反推（`dt=1/f`），与 `move_base` 控制频率一致。

### 8.1.2 参考轨迹构建与初始化（与代码一致）

参考轨迹 $\{\boldsymbol{x}^{ref}_k\}$ 通过 `buildReference` 从裁剪后的全局路径 `prune_plan` 按弧长等间距采样得到：

- 采样间隔：`ds_ref`；
- 采样长度：`horizon_steps`；
- 不足长度使用末点补齐；
- 对连续近邻点进行最小距离去重（实现中 `min_ds = 0.1 * ds_ref`），避免退化段影响走廊构建与数值稳定性。

该参考轨迹同时用于 iLQR 初始控制序列生成与代价函数计算。

## 8.2 代价函数与优化目标（严格对应实现）

### 8.2.1 iLQR 的优化问题与值函数分解

iLQR 求解如下**有限时域轨迹优化问题**：

$$\min_{\{\boldsymbol{u}_k\}} \sum_{k=0}^{K-1} \ell_k(\boldsymbol{x}_k, \boldsymbol{u}_k) + \ell_K(\boldsymbol{x}_K)$$

约束条件为离散动力学：

$$\boldsymbol{x}_{k+1} = f(\boldsymbol{x}_k, \boldsymbol{u}_k, \Delta t)$$

其中 $K$ 为规划地平线（通常 20-50 步），$\Delta t = 0.1$ 秒。

**动态规划视角**：定义**值函数** $V_k(\boldsymbol{x}_k)$ 表示从状态 $\boldsymbol{x}_k$ 开始到终端的最优代价。根据**贝尔曼方程**：

$$V_k(\boldsymbol{x}_k) = \min_{\boldsymbol{u}_k} \left\{ \ell_k(\boldsymbol{x}_k, \boldsymbol{u}_k) + V_{k+1}(f(\boldsymbol{x}_k, \boldsymbol{u}_k)) \right\}$$

边界条件：$V_K(\boldsymbol{x}_K) = \ell_K(\boldsymbol{x}_K)$（终端代价）。

iLQR 通过**局部二次近似**来求解这个问题：对每个时刻 $k$，在当前轨迹附近对动力学进行线性化，对代价函数进行二次展开。

### 8.2.2 阶段代价函数的定义

阶段代价函数：

$$
\ell_k =
w_{track}\|\boldsymbol{p}_k-\boldsymbol{p}^{ref}_k\|^2
 + w_{heading}\,\Delta\theta_k^2
 + w_u\|\boldsymbol{u}_k\|^2
 + w_{du}\|\boldsymbol{u}_k-\boldsymbol{u}_{k-1}\|^2
 + w_{obs}\,c_{obs}(\boldsymbol{p}_k)
 + w_{corr}\,c_{corr}(\boldsymbol{p}_k)
$$

**各项含义**：

| 项 | 符号 | 作用 | 权重 |
|---|------|------|------|
| 位置追踪 | $\boldsymbol{p}_k-\boldsymbol{p}^{ref}_k$ | 使轨迹追踪参考路径 | $w_{track}$ |
| 朝向追踪 | $\Delta\theta_k$ | 平滑转弯，朝向贴近参考 | $w_{heading}$ |
| 控制输入 | $\boldsymbol{u}_k$ | 避免过大速度/角速度 | $w_u$ |
| 控制变化 | $\boldsymbol{u}_k-\boldsymbol{u}_{k-1}$ | 平滑控制序列，避免振荡 | $w_{du}$ |
| 障碍规避 | $c_{obs}(\boldsymbol{p}_k)$ | 距离障碍足够远（ESDF或Costmap） | $w_{obs}$ |
| 走廊约束 | $c_{corr}(\boldsymbol{p}_k)$ | 轨迹留在安全走廊内 | $w_{corr}$ |

其中 $\Delta\theta_k$ 使用 `angleDiff` 归一化到 $[-\pi,\pi]$，$c_{obs}$ 来自 costmap/ESDF 障碍代价，$c_{corr}$ 为走廊代价。

**终端代价**（实现中位置与朝向误差按 $5\times$ 放大）：

$$\ell_K(\boldsymbol{x}_K) = 5(w_{track}\|\boldsymbol{p}_K-\boldsymbol{p}^{ref}_K\|^2 + w_{heading}\Delta\theta_K^2)$$

这种端点惩罚的放大确保轨迹最后能精准地到达参考点。

## 8.3 iLQR 的反向递推与值函数（实现一致）

### 8.3.1 反向递推框架与 Q 函数构造

从终端开始向前反推。对于时刻 $k$，假设值函数 $V_{k+1}(\boldsymbol{x}_{k+1})$ 已知，我们需要求得时刻 $k$ 的最优控制。

**线性化动力学**（在参考轨迹附近）：

$$f(\boldsymbol{x}_k, \boldsymbol{u}_k) \approx f(\hat{\boldsymbol{x}}_k, \hat{\boldsymbol{u}}_k) + F_x(\hat{\boldsymbol{x}}_k, \hat{\boldsymbol{u}}_k)\delta\boldsymbol{x}_k + F_u(\hat{\boldsymbol{x}}_k, \hat{\boldsymbol{u}}_k)\delta\boldsymbol{u}_k$$

其中 $\delta\boldsymbol{x}_k = \boldsymbol{x}_k - \hat{\boldsymbol{x}}_k$，$\delta\boldsymbol{u}_k = \boldsymbol{u}_k - \hat{\boldsymbol{u}}_k$，$F_x$ 和 $F_u$ 为雅可比矩阵。

**代价二次展开**：

$$\ell_k(\boldsymbol{x}_k, \boldsymbol{u}_k) \approx \ell_k(\hat{\boldsymbol{x}}_k, \hat{\boldsymbol{u}}_k) + \begin{pmatrix} \ell_x \\ \ell_u \end{pmatrix}^T \begin{pmatrix} \delta\boldsymbol{x}_k \\ \delta\boldsymbol{u}_k \end{pmatrix} + \frac{1}{2}\begin{pmatrix} \delta\boldsymbol{x}_k \\ \delta\boldsymbol{u}_k \end{pmatrix}^T \begin{pmatrix} \ell_{xx} & \ell_{xu} \\ \ell_{ux} & \ell_{uu} \end{pmatrix} \begin{pmatrix} \delta\boldsymbol{x}_k \\ \delta\boldsymbol{u}_k \end{pmatrix}$$

**Q 函数定义与二次近似**：

$$Q_k(\boldsymbol{x}_k, \boldsymbol{u}_k) = \ell_k(\boldsymbol{x}_k, \boldsymbol{u}_k) + V_{k+1}(f(\boldsymbol{x}_k, \boldsymbol{u}_k))$$

代入线性化动力学与二次展开，消去 $\delta\boldsymbol{x}_{k+1}$，可得 Q 函数的二次形式。

**控制增益**：

$$\boldsymbol{k}_k = -Q_{uu}^{-1}Q_u, \quad \boldsymbol{K}_k = -Q_{uu}^{-1}Q_{ux}$$

**正则化处理**（确保 $Q_{uu}$ 正定）：

实现中通过添加阻尼项 $\lambda I$ 来改进数值稳定性。

### 8.3.2 前向与后向递推流程

**后向递推**（从 $k=K-1$ 到 $0$）：对每个时刻计算控制增益与值函数。

**前向滚动与线搜索**：对 $\alpha\in\texttt{alphas}$ 进行线搜索，代价降低则接受。

### 8.3.3 迭代停止条件

## 8.4 速度约束与输出

控制输出在 `clampControl` 中做物理约束：

- $v \in [v_{min}, v_{max}]$；
- $\omega \in [-\omega_{max}, \omega_{max}]$；
- 若 $|\omega|<\omega_{min}$ 则按最小角速度修正。

最终输出为 iLQR 轨迹的第一步控制，并经 `linearRegularization` 与 `angularRegularization` 做速度平滑。

---

# 第八章 B 全局路径优化：共轭梯度法（Conjugate Gradient Optimization）

**本章角色**：此章介绍**全局路径优化层**，与第 8 章的局部规划与控制层（iLQR）**处于不同的分层级别**。

共轭梯度法是**全局规划的后处理与优化工具**，其任务是对 Sunshine 生成的初始离散栅格路径进行快速的几何平滑优化，使路径更光滑、更符合安全约束。该方法相比 iLQR 更轻量、收敛快，特别适合**实时性要求高、计算资源受限**的场景。

**关键区别**（与第 8 章 iLQR 对比）：

| 特性 | 共轭梯度（全局优化） | iLQR（局部优化） |
|------|-----------------|---------------|
| **层级** | 全局规划层（post-processing） | 局部规划与控制层 |
| **输入** | 离散栅格路径 π₀ | 参考路径 π* + 当前状态 |
| **输出** | 平滑路径 π*（世界坐标） | 可执行轨迹τ（控制命令 v, ω） |
| **目标** | 路径平滑、无碰撞 | 轨迹光滑、安全、可执行 |
| **约束** | 无碰撞（ESDF）+ 平滑 | 动力学 + 走廊 + ESDF |
| **计算成本** | 低（梯度下降） | 中等（反向递推 + 线搜索） |
| **收敛速度** | 快（50-100 迭代） | 中等（10-20 迭代） |
| **适用场景** | 实时性优先 | 精度优先 |

## 8B.1 共轭梯度法的基本原理与问题设置

### 8B.1.0 问题框架与动机

共轭梯度法（Conjugate Gradient Method）是一类用于求解**大规模二次优化问题**的迭代算法。在轨迹优化中，我们的目标是对 Sunshine 生成的离散栅格路径进行**几何平滑化**，同时保证：

1. **无碰撞**：所有路径点距离障碍足够远
2. **光滑**：路径点之间的连接相对平缓，避免尖锐转折
3. **低曲率**：转弯角度平缓，符合机器人的转向能力

传统的梯度下降法可以用来解决这样的问题，但收敛速度较慢（特别是在多维空间）。共轭梯度法通过精心选择搜索方向，使得每一步迭代都沿着"共轭方向"进行，从而在有限次迭代内（理论上不超过维数）收敛到最优解。

**关键假设**：目标函数具有**局部二次性**，即在当前点的泰勒展开中，二阶项主导。

### 8B.1.1 优化问题的构建

共轭梯度法用于求解如下无约束优化问题：

$$\min_{\{\boldsymbol{p}_i\}_{i=0}^{n}} J = \sum_{i=2}^{n-2} \left( w_{obs} \cdot c_{obs}(\boldsymbol{p}_i) + w_{smooth} \cdot c_{smooth}(\boldsymbol{p}_i) + w_{curv} \cdot c_{curv}(\boldsymbol{p}_i) \right)$$

其中：
- $\boldsymbol{p}_i = (x_i, y_i) \in \mathbb{R}^2$ 为第 $i$ 个路径点（世界坐标）；
- $n$ 为路径点总数；
- **循环范围** $i \in [2, n-2]$，保留起终点与相邻点固定，防止边界扰动。

### 8B.1.2 三项代价函数的定义

**1) 障碍代价项**（Obstacle Avoidance Cost）：

$$c_{obs}(\boldsymbol{p}_i) = \begin{cases}
0, & d(\boldsymbol{p}_i) \geq d_{\max} \\
w_{obs} \cdot 2.0 \cdot (d(\boldsymbol{p}_i) - d_{\max})^2 / d(\boldsymbol{p}_i), & 0 < d(\boldsymbol{p}_i) < d_{\max}
\end{cases}$$

其中 $d(\boldsymbol{p}_i)$ 为点 $\boldsymbol{p}_i$ 到最近障碍的欧氏距离（通过距离层 Distance Layer 查询），$d_{\max}$ 为允许的最大距离阈值。当点远离障碍时代价为 0；当接近障碍时以二次项形式增长，迫使优化器将轨迹推离障碍。

**2) 平滑代价项**（Smoothness Cost）：

$$c_{smooth}(\boldsymbol{p}_i) = \boldsymbol{p}_{i-2} - 4\boldsymbol{p}_{i-1} + 6\boldsymbol{p}_i - 4\boldsymbol{p}_{i+1} + \boldsymbol{p}_{i+2}$$

这是二阶有限差分形式，等价于轨迹二阶导数（加速度）的离散化。该项惩罚路径的"不平滑性"，迫使相邻三点尽可能共线或构成平缓曲线。

**3) 曲率代价项**（Curvature Cost）：

$$c_{curv}(\boldsymbol{p}_i) = \begin{cases}
0, & k(\boldsymbol{p}_i) \leq k_{\max} \\
w_{curv} \cdot (k(\boldsymbol{p}_i) - k_{\max})^2, & k(\boldsymbol{p}_i) > k_{\max}
\end{cases}$$

其中 $k(\boldsymbol{p}_i)$ 为 $\boldsymbol{p}_i$ 处的局部曲率，通过角度变化与弧长比计算：

$$k(\boldsymbol{p}_i) = \frac{\Delta\phi_i}{|\Delta\boldsymbol{p}_i|}$$

其中 $\Delta\phi_i$ 为相邻方向向量的夹角，$|\Delta\boldsymbol{p}_i|$ 为距离。该项在曲率超过机器人最大曲率 $k_{\max}$ 时触发惩罚，保证轨迹可执行性。

### 8B.1.3 共轭梯度法的迭代策略

共轭梯度法通过迭代构造"共轭方向序列"来加速收敛。基本迭代公式为：

$$\boldsymbol{p}_{i}^{(t+1)} = \boldsymbol{p}_{i}^{(t)} - \alpha^{(t)} \cdot \boldsymbol{g}^{(t)}_i$$

其中：
- $\boldsymbol{g}^{(t)}_i = \nabla_{p_i} J$ 为在第 $t$ 次迭代时第 $i$ 个点的代价梯度（合并三项）；
- $\alpha^{(t)}$ 为学习率（固定或线搜索）；
- 在仓库实现中，学习率 $\alpha$ 由参数 `alpha` 指定，通常取 0.1 左右。

## 8B.2 梯度计算（实现对应）

### 8B.2.1 综合梯度的构建

在每次迭代中，对第 $i$ 个路径点，总梯度为：

$$\nabla_{p_i} J = \nabla c_{obs} + \nabla c_{smooth} + \nabla c_{curv}$$

实现在 `conjugate_optimizer.cpp` 中以如下方式计算：

```cpp
for (int i = 2; i < path_opt_.size() - 2; ++i) {
    Vec2d xi_c2(path_opt_[i - 2].x(), path_opt_[i - 2].y());
    Vec2d xi_c1(path_opt_[i - 1].x(), path_opt_[i - 1].y());
    Vec2d xi(path_opt_[i].x(), path_opt_[i].y());
    Vec2d xi_p1(path_opt_[i + 1].x(), path_opt_[i + 1].y());
    Vec2d xi_p2(path_opt_[i + 2].x(), path_opt_[i + 2].y());

    Vec2d correction;
    correction = correction + _calObstacleTerm(xi, distance_layer);
    correction = correction + _calSmoothTerm(xi_c2, xi_c1, xi, xi_p1, xi_p2);
    correction = correction + _calCurvatureTerm(xi_c1, xi, xi_p1);
    
    Vec2d gradient = alpha_ * correction / (w_obstacle_ + w_smooth_ + w_curvature_);
    xi = xi - gradient;
    path_opt_[i].setX(xi.x());
    path_opt_[i].setY(xi.y());
}
```

关键观察：
- 三项梯度**按顺序累加**，最后通过权重和进行**归一化**；
- 学习率 $\alpha$ 乘以**归一化**梯度，保证步长稳定；
- 每次迭代遍历所有内部点进行同步更新。

### 8B.2.2 障碍代价梯度（_calObstacleTerm）

```cpp
Vec2d CGOptimizer::_calObstacleTerm(const Vec2d& xi,
    const boost::shared_ptr<costmap_2d::DistanceLayer>& dist_layer)
{
    Vec2d gradient;
    unsigned int mx, my;
    costmap_ros_->getCostmap()->worldToMap(xi.x(), xi.y(), mx, my);
    if (_insideMap(mx, my)) {
        const double resolution = costmap_ros_->getCostmap()->getResolution();
        double obs_dist = dist_layer->getDistance(mx, ny_ - my - 1) * resolution;
        double dx, dy;
        dist_layer->getGradient(mx, ny_ - my - 1, dx, dy);
        Vec2d obs_vec(dx, -dy);
        obs_vec.normalize();
        obs_vec *= 0.1 * resolution;
        
        if (obs_dist < obs_dist_max_ && obs_dist > 0)
            gradient = w_obstacle_ * 2.0 * (obs_dist - obs_dist_max_) * obs_vec / obs_dist;
    }
    return gradient;
}
```

**数学对应**：
- `dist_layer->getDistance()` 返回点到最近障碍的距离 $d(\boldsymbol{p}_i)$；
- `dist_layer->getGradient()` 返回距离场的梯度方向，指向"最近障碍所在方向"，取反可得"远离障碍的方向"；
- 在 `obs_dist < obs_dist_max_` 时，梯度大小为 $2.0 \cdot (d-d_{\max}) \cdot \text{obs\_vec} / d$，对应公式中的二次项导数。

### 8B.2.3 平滑代价梯度（_calSmoothTerm）

```cpp
Vec2d CGOptimizer::_calSmoothTerm(const Vec2d& xi_c2, const Vec2d& xi_c1,
    const Vec2d& xi, const Vec2d& xi_p1, const Vec2d& xi_p2)
{
    return w_smooth_ * (xi_c2 - 4.0 * xi_c1 + 6.0 * xi - 4.0 * xi_p1 + xi_p2);
}
```

**数学对应**：这正是平滑代价函数的形式，直接返回离散二阶导数的**负梯度**（用以下降梯度法）。该形式来自 5 点中心有限差分：
$$\nabla c_{smooth}(\boldsymbol{p}_i) = -\nabla J = -(x_{i-2} - 4x_{i-1} + 6x_i - 4x_{i+1} + x_{i+2})$$

符号为负是因为 `correction` 代表"需要施加的修正"（负梯度方向）。

### 8B.2.4 曲率代价梯度（_calCurvatureTerm）

```cpp
Vec2d CGOptimizer::_calCurvatureTerm(const Vec2d& xi_c1, const Vec2d& xi,
    const Vec2d& xi_p1)
{
    Vec2d gradient;
    double resolution = costmap_ros_->getCostmap()->getResolution();
    Vec2d d_xi = (xi - xi_c1) / (0.1 * resolution);
    Vec2d d_xi_p1 = (xi_p1 - xi) / (0.1 * resolution);

    Vec2d p1, p2;
    auto ort = [](const Vec2d& a, const Vec2d& b) {
        return a - b * a.innerProd(b) / std::pow(b.length(), 2);
    };

    double abs_dxi = d_xi.length();
    double abs_dxi_p1 = d_xi_p1.length();

    if (abs_dxi > 1e-3 && abs_dxi_p1 > 1e-3) {
        double d_phi = acos(clamp(d_xi.innerProd(d_xi_p1) / (abs_dxi * abs_dxi_p1), -1.0, 1.0));
        if (std::fabs(d_phi - 1.0) < eps || std::fabs(d_phi + 1.0) < eps)
            return gradient;
        
        double k = d_phi / abs_dxi;
        
        if (k > k_max_) {
            double u = 1.0 / abs_dxi / std::sqrt(1 - std::pow(std::cos(d_phi), 2));
            p1 = ort(d_xi, -d_xi_p1) / (abs_dxi * abs_dxi_p1);
            p2 = -ort(d_xi_p1, d_xi) / (abs_dxi * abs_dxi_p1);

            Vec2d s = d_phi * d_xi / std::pow(abs_dxi, 3);
            Vec2d ki = u * (-p1 - p2) - s;
            Vec2d ki_c1 = u * p2 + s;
            Vec2d ki_p1 = u * p1;

            gradient = w_curvature_ * (0.25 * ki_c1 + 0.5 * ki + 0.25 * ki_p1);

            if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
                return Vec2d();
        }
    }
    return gradient;
}
```

**数学对应**：
- $d\_xi$ 与 $d\_xi\_p1$ 分别对应 $\Delta\boldsymbol{p}_{i-1\to i}$ 与 $\Delta\boldsymbol{p}_{i\to i+1}$；
- `d_phi` 为两向量夹角 $\Delta\phi_i$；
- `k = d_phi / abs_dxi` 为曲率 $k_i$；
- 当 $k_i > k_{\max}$ 时，通过复杂的微分几何计算（正交补、切线约束等）得到梯度 `ki`、`ki_c1`、`ki_p1`，分别对应相邻三点的贡献；
- 最后返回的梯度为这三点贡献的加权平均（权重 0.25, 0.5, 0.25）。

## 8B.3 迭代过程与收敛特性

### 8B.3.1 迭代格式与收敛理论

本实现采用**最陡下降法**（Steepest Descent）的变种，每次迭代沿负梯度方向更新所有内部路径点：

$$\boldsymbol{p}^{(t+1)} = \boldsymbol{p}^{(t)} - \alpha \nabla J(\boldsymbol{p}^{(t)})$$

其中 $\boldsymbol{p} = [p_2, p_3, \ldots, p_{n-2}]^T$ 为所有优化点的堆积向量。

**收敛性保证**：对于**凸二次函数**，该算法以几何级数收敛，收敛速度由步长 $\alpha$ 与目标函数的条件数 $\kappa = \lambda_{\max}/\lambda_{\min}$ 决定。

在我们的场景中，代价函数由三项组成：

$$J(\boldsymbol{p}) = J_{obs}(\boldsymbol{p}) + J_{smooth}(\boldsymbol{p}) + J_{curv}(\boldsymbol{p})$$

- $J_{smooth}$ 项严格凸（二阶差分）
- $J_{obs}$ 项在开放空间内凸（ESDF 的二次近似）
- $J_{curv}$ 项在曲率不超过 $k_{\max}$ 时凸

因此，在路径整体可行的区间内，整个目标函数具有**局部凸性**，使得梯度下降法有良好的收敛性保证。

**收敛速度**：若条件数 $\kappa \approx O(1)$（对本问题典型），则收敛速度为：

$$\|e^{(t)}\| \leq \left(\frac{\kappa-1}{\kappa+1}\right)^t \|e^{(0)}\|$$

对于 100 次迭代，$\left(\frac{\kappa-1}{\kappa+1}\right)^{100}$ 足以使误差衰减至初值的 $10^{-10}$ 以下。

外层 `optimize()` 函数的实现逻辑：

```cpp
bool ConjugateOptimizer::optimize() {
    // 初始化路径（若尚未初始化）
    if (path_opt_.empty()) return false;
    
    int iter = 0;
    while (iter < params_.max_iter) {
        // 对每个内部点计算合并梯度
        for (int i = 2; i < path_opt_.size() - 2; ++i) {
            Vec2d gradient = _calSmoothTerm(i) + _calObstacleTerm(i) + _calCurvatureTerm(i);
            Vec2d xi(path_opt_[i].x(), path_opt_[i].y());
            // 梯度下降步
            xi = xi - gradient;
            path_opt_[i].setX(xi.x());
            path_opt_[i].setY(xi.y());
        }
        iter++;
    }
    return true;
}
```

**迭代特性**：
- **同步更新**：所有内部点在同一次迭代中同时更新（Jacobi 式）；
- **固定学习率**：默认 $\alpha = 0.1$，不进行线搜索，简化但收敛速度受限；
- **最大迭代次数**：由参数 `max_iter` 控制，通常 100 次；
- **退出条件**：达到最大迭代或早停（仓库实现无显式早停，依赖迭代次数上限）。

### 8B.3.2 收敛性分析与与 iLQR 的对比

```cpp
bool CGOptimizer::optimize(const Points3d& waypoints)
{
    // ... 获取距离层 ...
    int iter = 0;
    while (iter < max_iter_) {
        // 对每个内部点应用三项梯度
        for (int i = 2; i < path_opt_.size() - 2; ++i) {
            // ... 累加三项梯度 ...
            xi = xi - gradient;
            path_opt_[i].setX(xi.x());
            path_opt_[i].setY(xi.y());
        }
        iter++;
    }
    return true;
}
```

**迭代特性**：
- **同步更新**：所有内部点在同一次迭代中同时更新（Jacobi 式）；
- **固定学习率**：默认 $\alpha = 0.1$，不进行线搜索，简化但收敛速度受限；
- **最大迭代次数**：由参数 `max_iter` 控制，通常 100 次；
- **退出条件**：达到最大迭代或早停（仓库实现无显式早停，依赖迭代次数上限）。

共轭梯度法相比 iLQR 的优势与劣势：

| 特性 | 共轭梯度 | iLQR |
|-----|---------|------|
| **收敛速度** | 快（通常 50-100 迭代） | 较慢（通常 20-50 迭代） |
| **计算成本** | 低（仅梯度计算） | 中等（涉及 Hessian、线搜索） |
| **非线性能力** | 弱（局部二次近似） | 强（反向递推二次化） |
| **约束处理** | 需额外改造 | 自然支持 |
| **动力学友好性** | 弱（不考虑动力学） | 强（基于离散动力学）| 

因此，**共轭梯度法适合**：
- 初始路径已相对合理的场景（Sunshine 输出）；
- 主要目标是"平滑化"而非"剧烈调整"；
- 计算资源受限的嵌入式环境；
- 无强动力学约束的移动机器人。

而 **iLQR 适合**：
- 需要满足严格动力学约束的场景；
- 初始路径可行性较差需要大幅调整；
- 有充分计算资源的高性能平台。

## 8B.4 与 Sunshine 算法的协同

在本仓库的分层框架中，Sunshine + Conjugate Gradient 的组合有如下优势：

1. **快速初始规划**：Sunshine 通过几何切点检测快速生成拓扑可行路径（数百毫秒级）；
2. **轻量级优化**：Conjugate Gradient 在收敛的 50-100 迭代内（数十毫秒级）将路径平滑化；
3. **端到端效率**：整个全局规划+轨迹优化完成在 200-500 毫秒内，满足实时性；
4. **参数可维护性**：三项权重（$w_{obs}, w_{smooth}, w_{curv}$）独立可调，物理意义清晰。

## 8B.5 参数配置与调优

对应 `src/sim_env/config/trajectory_optimization/conjugate_gradient_params.yaml`：

```yaml
Optimizer:
  # 最大迭代次数（通常 100-200）
  max_iter: 100 
  # 学习率 / 步长系数（通常 0.05-0.2）
  alpha: 0.1
  # 障碍代价允许范围（米）
  obs_dist_max: 0.3 
  # 最大曲率限制（单位待确认，通常与机器人转向半径相关）
  k_max: 0.15
  # 三项代价权重
  w_obstacle: 0.5     # 障碍规避权重
  w_smooth: 0.25      # 平滑性权重
  w_curvature: 0.25   # 曲率限制权重
```

**参数调优建议**：
- 若路径过度平滑导致靠近障碍，增加 `w_obstacle` 或减少 `w_smooth`；
- 若路径曲率过高，增加 `w_curvature` 或减少 `k_max`；
- 若收敛速度过慢，增加 `alpha` 但需防止发散；
- 若迭代不足，增加 `max_iter` 但注意计算时间。

---

# 第九章 走廊约束与 ESDF 代价（仓库实现对照版）

本章详细阐述两种约束机制的构建原理、代价形式、梯度计算与在 iLQR 中的角色。

## 9.1 安全走廊（Convex Safety Corridor）的原理

### 9.1.1 走廊的作用与必要性

安全走廊（Convex Safety Corridor, CSC）是在轨迹优化中引入的一种"基于路径的时空约束"。传统网格代价地图仅记录"栅格是否被障碍占据"（二元判断），但在连续轨迹优化中，机器人可能在栅格边界附近游走，容易因数值精度丧失而碰撞。

走廊通过以下机制解决此问题：

- **离散化到连续**：从 Sunshine 全局规划输出的离散栅格路径 $\{c_0, c_1, \ldots, c_m\}$（其中 $c_i\in\mathbb{Z}^2$ 为栅格坐标）出发；
- **膨胀与凸分解**：绕每个路径点膨胀出自由空间，然后分解为一系列凸多边形 $\mathcal{P}_1,\mathcal{P}_2,\ldots,\mathcal{P}_n$；
- **过约束性**：这些多边形的交集是一条"管状"区域，保证轨迹在其内部时不碰撞。

### 9.1.2 构建流程（buildCorridorFromReference）

实现位置：`ilqr_controller.cpp` 的 `buildCorridorFromReference()` 方法。

**输入**：参考轨迹点序列 $\{\boldsymbol{x}^{ref}_k\}_{k=0}^{K}$，其中每个点已通过 `buildReference` 去重。

**步骤 1：点到 Points3d 转换与二次去重**

虽然 `buildReference` 已执行过一次去重，但为保险起见，`buildCorridorFromReference` 再执行一遍去重，防止退化多边形：

$$\min_{\Delta s} = \max(10^{-4}, 0.1\cdot\texttt{ds\_ref})$$

遍历参考点，保留间距大于 $\min_{\Delta s}$ 的点。此步骤保证后续凸分解算法不会遇到"零面积多边形"，从而避免数值异常。

**步骤 2：走廊分解**

调用 `ConvexSafetyCorridor::decompose(pts, params)` 生成凸多边形序列，输入参数包括轮基、转向角限制、轨迹宽度、安全裕度等，考虑机器人的实际占有空间，对每个参考点膨胀出自由区域后使用凸包算法得到凸多边形，相邻多边形的重叠保证了通过任意相邻多边形交集的轨迹都满足约束。

**步骤 3：存储与查询**

存储得到的凸多边形集合 `corridor_polygons_` 中的 `Polygon2d` 对象提供 `isInside(pt)` 与 `distanceToBoundary(pt)` 方法，用于在代价计算中快速查询点与走廊的关系。

### 9.1.3 走廊代价函数形式

对轨迹上任意点 $\boldsymbol{p}_k=(x_k,y_k)$，走廊代价定义为：

$$c_{corr}(\boldsymbol{p}_k) = \begin{cases} 0, & \text{if } \exists\, i,\, \boldsymbol{p}_k \in \mathcal{P}_i \text{（在走廊内）} \\ \beta \cdot d^2(\boldsymbol{p}_k, \partial\mathcal{C}), & \text{otherwise（在走廊外）} \end{cases}$$

其中 $\partial\mathcal{C}$ 表示走廊边界，$d(\boldsymbol{p}_k, \partial\mathcal{C})$ 为点到最近边界的欧氏距离，$\beta=w_{corr}$ 为权重系数。**物理意义**：在走廊内惩罚为 0，在走廊外惩罚与距离平方成正比，迫使优化器将轨迹拉回走廊内。

### 9.1.4 硬约束模式

当 `corridor_hard_constraint=true` 时，走廊外的代价变为 $+\infty$，绝对保证轨迹在安全区域内但可能导致局部规划无解，需要回到全局规划。

### 9.1.5 走廊梯度与反向递推

在 iLQR 反向递推中，走廊代价对状态的梯度为：

$$\frac{\partial c_{corr}}{\partial \boldsymbol{x}_k} = 2\beta \, d(\boldsymbol{p}_k, \partial\mathcal{C}) \cdot \frac{\partial d}{\partial \boldsymbol{p}_k}$$

其中 $\frac{\partial d}{\partial \boldsymbol{p}_k}$ 指向离边界最近的方向。在反向递推的 $Q$ 函数中，该梯度信息引导优化器调整控制增益以远离走廊边界。

## 9.2 ESDF 障碍代价（Euclidean Signed Distance Field）

### 9.2.1 ESDF 的概念与构建

ESDF（欧氏符号距离场）是栅格地图上记录每个栅格到最近障碍欧氏距离的标量场：

$$d(c_i) = \min_{c_j \in \text{obstacle}} \|c_i - c_j\|_2$$

其中 $c_i,c_j\in\mathbb{Z}^2$ 为栅格坐标。**关键特性**：

1. **距离信息**：障碍越近距离越小，自由空间距离为正；
2. **梯度连续**：$\nabla d(c)$ 指向距离增加最快的方向，通常垂直于最近障碍边界；
3. **计算高效**：通过距离变换（Distance Transform）在线性时间内计算，在本仓库中由 `DistanceLayer` 组件提供。

### 9.2.2 作用位置与使用条件

当 `use_esdf: true` 且 costmap 中包含有效的 DistanceLayer 时，在 `evaluateTrajectoryCost` 中改用 ESDF 代价计算：

$$c_{obs,ESDF}(\boldsymbol{p}_k) = \alpha \cdot \exp\left(-\frac{d(\boldsymbol{p}_k)}{\sigma}\right)$$

其中 $d(\boldsymbol{p}_k)$ 为点到最近障碍的欧氏距离（栅格单位），$\alpha=w_{obs}$ 为权重，$\sigma=\texttt{esdf\_sigma\_m}$ 为尺度参数（米）。

### 9.2.3 代价衰减与物理意义

- **接近障碍** ($d\to 0$)：$c_{obs}\to \alpha$，惩罚最大；
- **远离障碍** ($d\to\infty$)：$c_{obs}\to 0$，惩罚消失；
- **尺度参数** $\sigma$：控制衰减速度，值越大代价衰减越缓和，影响范围越广。相比梯阶式 Costmap 代价，指数衰减提供了连续光滑的代价曲面，有利于 iLQR 反向递推的收敛。

### 9.2.4 梯度计算与反向递推

ESDF 代价对位置的梯度为：

$$\frac{\partial c_{obs}}{\partial \boldsymbol{p}_k} = \alpha \cdot \exp\left(-\frac{d}{\sigma}\right) \cdot \left(-\frac{1}{\sigma}\right) \cdot \frac{\partial d}{\partial \boldsymbol{p}_k}$$

其中 $\frac{\partial d}{\partial \boldsymbol{p}_k}$ 由 ESDF 的梯度域直接提供（无需数值微分），在 iLQR 反向递推中构成 $\ell_x$ 的一部分，参与状态值函数 $V$ 的递推，引导轨迹优化远离障碍。

### 9.2.5 ESDF vs 栅格 Costmap 代价对比

| 特性 | ESDF（指数衰减） | Costmap（网格查询） |
|------|-----------------|-------------------|
| 代价形式 | $\alpha\exp(-d/\sigma)$ | $c(c_i)$（离散值） |
| 梯度连续性 | 连续光滑 | 阶跃性（栅格边界不连续） |
| 反向递推收敛性 | 优（光滑曲面） | 较差（可能困于局部） |
| 计算复杂度 | 低（只需查表+指数） | 低（查表） |
| 参数调优 | $\sigma$（衰减速度） | $w_{obs}$（权重） |
| 适用场景 | 窄通道、复杂障碍 | 简单环境、最小成本路径 |

## 9.3 走廊与 ESDF 的协同

### 9.3.1 代价函数的完整形式

在综合使用走廊与 ESDF 的情况下，轨迹代价函数为：

$$\ell_k = w_{track}\|\Delta\boldsymbol{p}\|^2 + w_{heading}\Delta\theta^2 + w_u\|\boldsymbol{u}_k\|^2 + w_{du}\|\boldsymbol{u}_k-\boldsymbol{u}_{k-1}\|^2 + w_{obs}\,c_{obs,ESDF}(\boldsymbol{p}_k) + w_{corr}\,c_{corr}(\boldsymbol{p}_k)$$

各项职责分工明确：

- **追踪项** $w_{track}\|\Delta\boldsymbol{p}\|^2$：拉近轨迹到参考路径；
- **朝向项** $w_{heading}\Delta\theta^2$：平滑转弯；
- **控制项** $w_u\|\boldsymbol{u}_k\|^2$：惩罚过大加速与角速度；
- **光滑项** $w_{du}\|\boldsymbol{u}_k-\boldsymbol{u}_{k-1}\|^2$：抑制控制振荡；
- **ESDF 项** $w_{obs}\,c_{obs,ESDF}(\boldsymbol{p}_k)$：对所有开放空间的障碍产生"斥力"，距离越近斥力越强；
- **走廊项** $w_{corr}\,c_{corr}(\boldsymbol{p}_k)$：对参考路径周围的已知自由区域建立"保障"，在走廊内为 0，外则惩罚。

### 9.3.2 约束协同的几何直观

想象一条从全局规划得到的参考路径的优化过程：

1. **全局规划**贪心地寻找"最短可通行路径"，但精度只有栅格级别；
2. **走廊分解**沿参考路径膨胀出凸区域，形成"允许轨迹游走"的管状约束；
3. **ESDF 梯度**在走廊内部，对轨迹点到障碍的距离进行"定量评估"，远离尖锐障碍边界，提供"质量信号"；
4. **iLQR 优化**在满足走廊约束的前提下，沿 ESDF 梯度调整轨迹，最终得到"既安全又光滑"的可执行轨迹。

两种机制形成"约束层 + 代价层"的互补关系：走廊保证**几何可行性**，ESDF 保证**数值稳定性与安全裕度**。

### 9.3.3 参数选择与调优指南

| 参数 | 含义 | 典型范围 | 调优建议 |
|------|------|--------|--------|
| `use_corridor` | 启用走廊约束 | bool | 在窄通道环境下应为 true |
| `corridor_hard_constraint` | 硬约束模式 | bool | 若为 true，当局部规划失败时触发全局重规划 |
| `w_corridor` | 走廊软惩罚权重 | [0.5, 5.0] | 增大使轨迹更贴近参考路径，减小允许更多偏离 |
| `corridor_safety_range` | 膨胀裕度 | [0.1, 0.5] m | 越大越保守，但可能导致走廊过紧 |
| `use_esdf` | 启用 ESDF 代价 | bool | 在复杂障碍环境应为 true |
| `esdf_sigma_m` | ESDF 衰减尺度 | [0.3, 1.0] m | 影响障碍"影响范围"，值大时感知范围广 |
| `w_obs` | 障碍代价权重 | [0.5, 2.0] | 增大使轨迹更远离障碍（更保守） |

---

# 第十章 事件触发式重规划策略（仓库参数对照）

## 10.1 分层框架的耦合机制

本仓库采用的分层规划架构采用**事件触发式重规划**（Event-Triggered Replanning）策略，其核心是：
- **全局层**持续输出初始路径 $\pi$；
- **局部层**基于当前观测环境、全局路径及当前位置生成可执行轨迹 $\tau$；
- 当局部规划**失败**（如轨迹碰撞或无可行解）时，立即触发全局重规划。

此策略相比"周期性重规划"（按固定频率）的优势在于：
1. **计算高效**：仅在必要时调用全局规划器；
2. **实时响应**：环境变化或规划失败可立即反应；
3. **层级清晰**：全局规划处理拓扑层面，局部规划处理轨迹质量。

## 10.2 move_base 参数配置

本仓库通过 ROS `move_base` 框架的以下参数实现事件触发式重规划：

**配置文件**：`src/sim_env/config/move_base_params.yaml`

```yaml
# 全局规划频率（Hz）
planner_frequency: 0.0        # 0.0 表示关闭周期性全局重规划

# 全局规划耐心时间（秒）
planner_patience: 5.0          # 超过此时间未获得新路径则认为规划失败

# 局部规划频率（Hz）  
controller_frequency: 10.0     # 10Hz，保证高频执行

# 局部规划耐心时间（秒）
controller_patience: 0.2       # 关键参数：局部规划短时间内失败即触发全局重规划

# 振荡检测参数
oscillation_timeout: 10.0      # 10 秒内超过指定范围的运动才不算陷入振荡
oscillation_distance: 0.2      # 检测范围：0.2 米

# 代价地图重置距离
conservative_reset_dist: 3.0   # 超过 3 米后重置代价地图
```

## 10.3 参数语义与重规划触发条件

### 10.3.1 planner_frequency = 0.0

设置为 0.0 表示**关闭周期性重规划**。在 move_base 内部，此设置使得全局规划器仅在以下两种情况被调用：
1. 初始规划（启动时）；
2. 局部规划失败触发的事件触发重规划。

### 10.3.2 controller_patience = 0.2

这是最**关键的触发参数**。其含义是：
- 局部控制器（iLQR）在连续 0.2 秒内无法产生有效的速度命令；
- 或连续 0.2 秒内规划失败（轨迹碰撞、数值发散等）；
- 触发全局重规划。

在 move_base 内部，这对应如下逻辑伪代码：
```
if (time_since_last_valid_control > controller_patience):
    request_new_global_plan()
```

### 10.3.3 controller_frequency = 10.0

保证局部规划（iLQR）以 10 Hz 的频率运行，对应 $\Delta t = 0.1$ 秒的控制周期。在我们的 iLQR 实现中，`params_.dt` 由此导出：

$$\Delta t = \frac{1}{\texttt{controller\_frequency}} = \frac{1}{10} = 0.1 \text{ 秒}$$

此参数直接用于离散动力学更新：
$$\boldsymbol{x}_{k+1} = \boldsymbol{x}_k + \begin{pmatrix} v_k \cos\theta_k \\ v_k \sin\theta_k \\ \omega_k \end{pmatrix} \Delta t$$

### 10.3.4 oscillation_timeout = 10.0 与 oscillation_distance = 0.2

这两个参数组合定义了"振荡检测"条件：
- 在过去 10 秒内，如果机器人的移动范围（相对于某个基准位置）超过 0.2 米，则认为**正常运动**；
- 否则认为陷入**局部振荡**，触发特殊恢复策略。

其逻辑为：
$$\text{is\_oscillating} = (\text{distance\_traveled\_in\_last\_}10s < 0.2m)$$

## 10.4 事件触发重规划的流程图

```
初始化: global_plan ← Sunshine(start, goal)
       local_traj ← NULL
       failure_timer ← 0

主控制循环 (10 Hz):
  1. 获取当前位置 x_cur 与环境观测
  2. IF global_plan 为空 OR 检测到振荡:
        global_plan ← Sunshine(x_cur, goal)
        failure_timer ← 0
  3. IF global_plan 非空:
        local_traj ← iLQR(x_cur, global_plan, corridor, ESDF)
  4. IF local_traj 有效 (不碰撞、数值正常):
        发送控制命令 u ← local_traj[0]
        failure_timer ← 0
     ELSE:
        failure_timer += 0.1 秒
        IF failure_timer > 0.2 秒:
           全局重规划 (标记 global_plan ← NULL)
           failure_timer ← 0
  5. 等待下一控制周期
```

## 10.5 与代码的对应

- `move_base` 框架内部通过 `BaseLocalPlannerROS` 接口调用 `ILQRController::computeVelocityCommands()`；
- 返回值为 false 时自动触发全局重规划；
- 我们的 `ILQRController` 在检测到轨迹碰撞或数值问题时返回 false，由 move_base 框架自动进行事件触发。

---

## 11.1 题目候选

1) 《基于阳光射线与 iLQR 的分层运动规划方法》

2) 《面向 2D 栅格地图的阳光射线–iLQR 分层规划框架》

3) 《基于阳光射线与安全走廊约束的分层运动规划方法》

## 11.2 分层结构建议

- **全局层**：Sunshine（Sunlight）采样式全局规划，输出栅格路径；
- **局部层**：iLQR 轨迹优化 + 走廊/ESDF 约束，输出可执行速度；
- **重规划机制**：事件触发（由 `controller_patience` 与 `planner_frequency` 体现）。

上述机制使系统在"局部不可行/输出失败"时快速回到全局层，完成路径重规划。
