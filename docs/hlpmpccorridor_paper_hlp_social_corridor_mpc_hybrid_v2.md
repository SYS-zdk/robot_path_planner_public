# 基于分层混合状态空间的社交感知运动规划与实时控制融合方法

## 摘要
本文提出一种面向移动机器人的两层分层运动规划方法，整合全局路径规划与局部实时导航。**全局层**采用改进的混合A*（Hybrid A*）算法，结合Voronoi势能指导与L-BFGS-B轨迹优化器，在满足车辆运动学约束的前提下快速生成初始可行路径，随后通过优化获得光滑的参考轨迹。**局部层**基于分层可达状态图实现短时参考路径搜索，并以社交代价、走廊约束与MPC控制的多维融合形成综合的局部导航系统。具体地，全局路径作为局部规划的高层指引，而局部规划通过实时障碍响应与动力学约束保证执行的可行性与安全性。社交代价通过独立缓存模式（不污染主代价地图）由规划器在轨迹评分时直接查询；凸安全走廊为局部规划提供结构化可行域先验；MPC与前瞻点跟踪在连续权重融合下实现平滑控制与鲁棒回退。

## 符号与假设
- 机器人在平面内运动，位姿为 $\mathbf{x}=[x,\,y,\,\theta]^\top$，控制量为 $\mathbf{u}=[v,\,\omega]^\top$。
- 规划周期为 $T_s$（控制循环周期），处理时延为 $T_p$。
- 可达图的前向时间窗（局部可达视野）记为 $T_{\mathrm{cap}}$；近目标减速时间窗记为 $T_{\mathrm{dec}}$。
- 线速度、角速度与加速度满足上下界：
  $$v\in[v_{\min},v_{\max}],\quad \omega\in[-\omega_{\max},\omega_{\max}],$$
  $$|\dot v|\le a_v,\quad |\dot\omega|\le a_\omega.$$
- 局部代价地图提供静态障碍与膨胀代价值；全局路径（或其局部截断）作为参考曲线。

---

## 分层规划架构

本方法分为**全局规划层**与**局部规划层**两个主要阶段：

1. **全局规划层**（周期 $T_g \gg T_s$，通常 1-5Hz）：
   - 输入：全局起点、全局目标、全局静态栅格地图
   - 过程：混合A*搜索 → 路径密度优化 → LBFGS轨迹优化
   - 输出：平滑的全局参考轨迹 $\tau_g=\{(x_i^g, y_i^g)\}_{i=0}^{M}$，作为高层指导

2. **局部规划层**（周期 $T_s$，通常 50-100Hz）：
   - 输入：机器人当前位姿、局部代价地图、全局参考轨迹、行人跟踪数据
   - 过程：分层可达图搜索 → 速度分配 → 角速度优化 → MPC/跟踪控制
   - 输出：实时控制量 $(v,\omega)$，直接驱动机器人执行层


局部规划在每个周期内执行三步：
1) **分层可达状态图构建与局部路径搜索**：在短时可达集上离散采样，建立分层图并用最短路搜索得到一条局部参考折线；
2) **基于障碍成本与动力学的线速度分配**：依据参考折线上的障碍代价强度自适应降低目标线速度，并结合加速度约束裁剪；
3) **基于Hybrid A\*的角速度优化与时间参数化**：在局部窗口内对 $(x,y,\theta)$ 进行离散化搜索，以障碍代价与启发式函数联合评估，得到期望角速度；再用时间窗完成参数化。

在此基础上，社交代价通过**独立的空间社交层（SocialLayer）与轨迹级社交评估函数（SocialCostFunction）**同时注入；行人动态安全由轨迹级常速度预测独立评估；局部走廊通过凸分解生成并以一致性判别的形式参与轨迹可行性判断；控制层在HLP输出与QP求解输出之间连续融合，以获得平滑且可回退的控制指令。

---

## 方法总览
局部规划在每个周期内执行三步：
1) **分层可达状态图构建与局部路径搜索**：在短时可达集上离散采样，建立分层图并用最短路搜索得到一条局部参考折线；
2) **基于障碍成本与动力学的线速度分配**：依据参考折线上的障碍代价强度自适应降低目标线速度，并结合加速度约束裁剪；
3) **基于Hybrid A\*的角速度优化与时间参数化**：在局部窗口内对 $(x,y,\theta)$ 进行离散化搜索，以障碍代价与启发式函数联合评估，得到期望角速度；再用时间窗完成参数化。

在此基础上，社交代价通过**独立的空间社交层（SocialLayer）与轨迹级社交评估函数（SocialCostFunction）**同时注入；行人动态安全由轨迹级常速度预测独立评估；局部走廊通过凸分解生成并以一致性判别的形式参与轨迹可行性判断；控制层在HLP输出与QP求解输出之间连续融合，以获得平滑且可回退的控制指令。

---

# 第一部分：全局规划层（Hybrid A* + LBFGS）

## 1. 混合A*（Hybrid A*）全局规划

### 1.1 Hybrid A*算法概述

混合A*是标准A*算法在连续运动学约束下的推广，广泛应用于自动驾驶的路径规划。相比标准A*的栅格化点状节点扩展，混合A*使用车辆运动学模型（Dubins或Reeds-Shepp曲线）生成候选轨迹段，在三维状态空间 $(x,y,\theta)$ 上搜索，确保规划的路径满足车辆的最小转弯半径与曲率约束。

**核心改进点**：
- **状态空间维度**：从二维 $(x,y)$ 扩展到三维 $(x,y,\theta)$，显式处理车辆朝向
- **轨迹段生成**：使用运动基元（Motion Primitives）替代八向网格连接，基于Reeds-Shepp曲线实现前进/后退运动
- **启发式函数**：采用 $h=\max(h_1,h_2)$ 形式，其中 $h_1$ 为非完整性约束下的启发式（考虑最小转弯半径，忽略障碍），$h_2$ 为有障碍物完整性启发式（栅格化A*结果，忽略运动学约束）
- **搜索效率**：通过终点解析扩展（Analytic Expansion）加速终端连接

### 1.2 Voronoi势能集成的改进搜索

#### 1.2.1 边代价的Voronoi修正

在本实现中，混合A*搜索的边代价不仅考虑运动距离，还整合Voronoi势能项以引导路径离开障碍物附近。当机器人从节点 $\mathbf{p}$ 移动到后继节点 $\mathbf{n}$ 时，边代价的计算为：

$$g(\mathbf{n}) = g(\mathbf{p}) + \underbrace{d_{\mathrm{edge}}}_{\text{轨迹长度}} \cdot \underbrace{(1 + w_v \cdot \rho_v(\mathbf{n}))}_{\text{Voronoi修正因子}} + \text{penalties}$$

其中：
- $g(\mathbf{p})$：从起点到前驱节点的累计代价
- $d_{\mathrm{edge}}$：当前边的运动距离（通过Dubins或Reeds-Shepp曲线计算）
- $w_v$：Voronoi成本权重，控制势能对路径的影响强度（默认 0.0，禁用）
- $\rho_v(\mathbf{n})$：节点处的Voronoi势能，范围 $[0,1]$
- penalties：转向惩罚、反向惩罚等（见node_hybrid.cpp第332-344行）

代码实现（node_hybrid.cpp第347-387行）中，$\rho_v$ 通过三项乘积计算，并采用**显式分段守护条件**确保有效范围：

$$\rho_v(x,y) = \begin{cases}
\min\left(1.0, \max\left(0.0, \frac{\alpha}{\alpha+d_0} \cdot \frac{d_v}{d_0+d_v} \cdot \left(\frac{d_0-d_0^{\max}}{d_0^{\max}}\right)^2\right)\right), & \text{if } d_0 \le d_0^{\max}\\
0, & \text{if } d_0 > d_0^{\max}
\end{cases}$$

其中三项的含义为：
- **term1** $= \frac{\alpha}{\alpha+d_0}$：随着 $d_0$ 增加而单调递减，衰减速率由 $\alpha$ 控制
- **term2** $= \frac{d_v}{d_0+d_v}$：路径靠近Voronoi骨架时增加，靠近障碍物时减小
- **term3** $= \left(\frac{d_0-d_0^{\max}}{d_0^{\max}}\right)^2$：当 $d_0 \le d_0^{\max}$ 时有意义；在 $d_0 \approx d_0^{\max}/2$ 处最小，在 $d_0 = 0$ 或 $d_0 = d_0^{\max}$ 处为最大值1

**参数说明**：
- $d_0(x,y)$：点 $(x,y)$ 到最近障碍物的欧式距离（通过costmap的DistanceLayer获取）
- $d_v(x,y)$：点 $(x,y)$ 到Voronoi骨架（走廊中心线）的距离
- $d_0^{\max}$：势能函数的有效范围，当 $d_0 > d_0^{\max}$ 时势能强制为0（**默认 2.0m**）
- $\alpha$：势能衰减速率，值越大衰减越缓慢（**默认 1.0m**）
- $w_v$：Voronoi成本权重，实现中**默认 0.0**（禁用），避免搜索困难

#### 1.2.2 设计理念与实现要点

**Voronoi势能的双重作用**：
1. **吸引效应**（term2）：通过考察到Voronoi骨架的距离，鼓励路径沿走廊中心运动
2. **距离效应**（term1）：通过考察到障碍物的距离，在远离障碍物时减弱势能，避免过度约束

**有效范围的显式守护**：
- 代码中在计算 $\rho_v$ 前，显式检查条件 `d0 <= d0_max`（第365行）
- 当 $d_0 > d_0^{\max}$ 时，整个Voronoi势能项被跳过，$\rho_v$ 被设为0
- 这种分段定义确保在开放空间中不会对路径造成不必要的约束，避免导致规划失败

**关键代码片段**（node_hybrid.cpp第365-385行）：
```cpp
// IMPORTANT: The Voronoi potential is defined piecewise.
// When d0 > d0_max, rho_v must be 0 (no Voronoi influence in open space).
if (std::isfinite(d0) && std::isfinite(dv) && std::isfinite(d0_max) && d0_max > 0.0 &&
    std::isfinite(alpha) && alpha >= 0.0 && d0 <= d0_max)  // ← 显式守护条件
{
  const double term1 = alpha / (alpha + std::max(0.0, d0));
  const double denom = std::max(1e-6, d0 + dv);
  const double term2 = std::max(0.0, dv) / denom;
  const double term3_raw = (d0 - d0_max) / d0_max;
  const double term3 = term3_raw * term3_raw;
  const double rho_v = std::min(1.0, std::max(0.0, term1 * term2 * term3));

  // Only apply cost multiplier when rho_v is meaningful (avoids unnecessary computation)
  if (rho_v > 1e-6)
  {
    travel_cost *= (1.0 + motion_table.voronoi_cost_weight * rho_v);
  }
}
```

**为何默认禁用**（$w_v = 0.0$）：
- Voronoi势能的引入增加了搜索空间的复杂性，可能导致搜索效率下降
- 在已有良好初始规划（如Hybrid A*自身的启发式）的情况下，势能收益有限
- 用户可通过ROS参数 `/PathPlanner/hybrid_astar/voronoi_cost_weight` 灵活启用或调整（推荐值：0.0-0.5）

**参数范围与建议**：
- `voronoi_d0_max`: 范围 [0.5, 5.0]m；推荐 2.0m（约40个栅格单元，地图分辨率0.05m）
- `voronoi_alpha`: 范围 [0.1, 5.0]m；推荐 1.0m（与默认最小转弯半径相当）
- `voronoi_cost_weight`: 范围 [0.0, 1.0]；推荐保持禁用（0.0）或轻微激活（0.1-0.2）

### 1.3 路径密度与RDP优化

#### 1.3.1 为什么需要路径简化

混合A*在完整搜索过程中，为了保证运动学连续性，在 $(x,y,\theta)$ 状态空间进行细粒度离散化。其结果是输出的路径点个数通常在 **100-200 个之间**，间距约为地图分辨率（0.05m）。这带来两个问题：

1. **计算负担**：后续LBFGS优化需要对每个路径点进行梯度计算，变量数过多会导致优化时间过长
2. **冗余信息**：相邻路径点之间的方向变化微小，携带大量的几何冗余

因此，在LBFGS优化之前需要对路径进行简化，保留关键的几何特征点，去除冗余点。

#### 1.3.2 Ramer-Douglas-Peucker（RDP）算法原理

RDP是一种经典的路径简化算法，其基本思想是**递归地查找与目标线段偏差最大的点，若最大偏差超过阈值则保留该点并递归处理两段子路径，否则移除其间的所有中间点**。

**算法流程**：

设路径为 $\mathcal{P} = \{P_0, P_1, \ldots, P_N\}$，简化阈值为 $\delta$。

1. **找最大偏差点**：计算路径上每个点到端点连线的垂直距离，找出最大距离点 $P_{\mathrm{max}}$：
$$d_{\mathrm{max}} = \max_{i=1}^{N-1} \frac{|\overrightarrow{P_0P_i} \times \overrightarrow{P_0P_N}|}{|\overrightarrow{P_0P_N}|}$$

2. **判断是否递归**：
   - 若 $d_{\mathrm{max}} > \delta$：保留 $P_{\mathrm{max}}$，对 $[P_0, \ldots, P_{\mathrm{max}}]$ 和 $[P_{\mathrm{max}}, \ldots, P_N]$ 分别递归应用RDP
   - 若 $d_{\mathrm{max}} \le \delta$：移除 $P_1, \ldots, P_{N-1}$ 之间的所有点

3. **递归终止**：当路径段仅包含2个点时停止

**代码对应**：在 path_planner_node.cpp 的 `configurePlannerToolchain()` 函数中，调用RDP简化：
```cpp
if (pruner_) {
  path = pruner_->prune(path);  // 返回简化后的路径
}
```

#### 1.3.3 参数选择与效果

**阈值 $\delta$ 的确定**：
- 在论文所述实现中，$\delta = 0.03\,\mathrm{m}$（相对于地图分辨率 0.05m）
- 该阈值选择保证：
  - 足够小以保留路径的几何特征（曲线的转折点）
  - 足够大以实现显著的点数压缩（通常从 150 点压缩至 50-80 点）

**压缩效果**：

设原始路径点数为 $N_0$，简化后为 $N_{\mathrm{rdp}}$，压缩比为 $\rho = N_{\mathrm{rdp}} / N_0$。

- **原始路径**：$N_0 \approx 150$ 点，采样间距 $\approx 0.05$ m
- **RDP简化后**：$N_{\mathrm{rdp}} \approx 50-80$ 点，压缩比 $\rho \approx 33-53\%$
- **计算复杂度降低**：LBFGS优化变量数从 $2 \times 150 = 300$ 降至 $2 \times 65 = 130$（平均），加速比约 $2.3\times$

**几何保真度**：RDP保留了原始路径的拓扑结构和关键转折点，后续LBFGS优化在此基础上进行局部平滑，不会产生全局拓扑变化。

#### 1.3.4 关键参数详解：曲线采样与输出间距

在混合A*的完整工作流中，涉及两个关键的密度控制参数，易于混淆但作用完全不同：

**1) `curve_sample_ratio`（曲线采样步长）**

- **作用阶段**：Hybrid A*搜索中，生成运动基元（Dubins/Reeds-Shepp曲线）时
- **物理含义**：沿着曲线离散采样的步长（以地图栅格单元计）
- **典型值**：0.1-0.5 栅格单元（对应 0.005-0.025 m，地图分辨率 0.05 m）
- **用途**：对曲线进行密集采样以进行**碰撞检测**
  - 采样越细（ratio越小），碰撞检测越精确，搜索时间越长
  - 采样越粗（ratio越大），可能遗漏细小障碍，降低碰撞检测精度
- **推荐设置**：保持默认值 0.2（相当于 0.01 m 采样间距）

**2) `path_point_spacing`（输出路径间距）**

- **作用阶段**：混合A*搜索完成后，将搜索路径转换为输出路径点时
- **物理含义**：输出的路径点之间的**最小间距**（单位：米）
- **典型值**：0.05-0.5 m
- **用途**：控制输出路径的**稀密程度**，直接影响后续LBFGS优化的计算负担
  - 输出间距小 → 点数多 → LBFGS计算时间长
  - 输出间距大 → 点数少 → LBFGS计算快，但可能失去细节
- **代码映射**：在ROS参数加载时，转换为网格单元数存储：
  $$\text{path\_min\_separation[cells]} = \frac{\text{path\_point\_spacing[m]}}{\text{costmap\_resolution}}$$
  例如：0.5 m / 0.05 m = 10 cells
- **推荐设置**：0.5 m（中等压缩，平衡精度与速度）

**工作流示意**：
```
Hybrid A* 搜索 
  ↓ 使用 curve_sample_ratio 进行碰撞检测
  ↓
  ↓ 搜索完成
  ↓
输出路径转换（按 path_point_spacing 间距采样）
  ↓ 得到：100-200 个点
  ↓
RDP 简化（使用 delta=0.03 m）
  ↓ 得到：50-80 个点
  ↓
LBFGS 优化（处理简化后的点）
  ↓ 输出：光滑的优化轨迹
```

**参数依赖关系**：
- `curve_sample_ratio` 与 `path_point_spacing` 是**独立**的
  - 前者控制搜索过程的精度
  - 后者控制搜索结果的密度
- 两者均为 0（禁用）时的行为不同：
  - `curve_sample_ratio = 0` → 搜索使用所有曲线点（最密集采样，极慢）
  - `path_point_spacing = 0` → 输出不进行点数压缩（保留所有搜索点）

---

## 2. LBFGS轨迹优化

### 2.1 优化问题公式与概述

#### 2.1.1 问题定义

接收RDP简化后的路径作为初值，通过**L-BFGS-B**（有界L-BFGS）算法求解以下加权多目标无约束优化问题：

$$\min_{\mathbf{p}} f(\mathbf{p}) = w_{\mathrm{obs}}\cdot f_{\mathrm{obs}}(\mathbf{p}) + w_{\mathrm{smo}}\cdot f_{\mathrm{smo}}(\mathbf{p}) + w_{\mathrm{cur}}\cdot f_{\mathrm{cur}}(\mathbf{p})$$

其中：
- **决策变量**：$\mathbf{p}=\{(x_0,y_0),\ldots,(x_N,y_N)\}$ 为轨迹上 $N+1$ 个点的2D坐标，维度 $\mathbf{p} \in \mathbb{R}^{2(N+1)}$
- **目标函数**：三个代价项的加权组合，各项物理含义见2.2节
- **权重系数**：$w_{\mathrm{obs}}, w_{\mathrm{smo}}, w_{\mathrm{cur}} \in [0, +\infty)$，通过ROS参数文件 `lbfgs_params.yaml` 配置

#### 2.1.2 约束处理策略

严格的可行域约束（所有点必须在自由空间）采用**软约束惩罚**而非硬约束处理：

$$\text{可行性} \Leftrightarrow \text{障碍物代价项施加高惩罚}$$

这种方法的优势：
1. 避免复杂的约束处理逻辑，使L-BFGS-B无约束算法直接适用
2. 点进入障碍物时，惩罚陡增，自动推离不可行域
3. 数值稳定性好，梯度计算简洁

**固定边界条件**：在优化中，**首尾各固定2个点**（而非仅固定1个），即 $\mathbf{p}_0, \mathbf{p}_1$ 和 $\mathbf{p}_{N-1}, \mathbf{p}_N$ 保持不变，只优化中间的 $\mathbf{p}_2, \ldots, \mathbf{p}_{N-2}$。这样做的目的：
- 保证优化后轨迹的**连续一阶导数**（固定2个点可约束切线方向）
- 确保与路径的起点和目标点的连接平滑，避免尖角

为此，要求输入路径至少包含 5 个点（$2 + \text{至少1个自由点} + 2$），否则优化无意义。

#### 2.1.3 LBFGS-B算法简介

L-BFGS-B（Limited-memory BFGS with Bounds）是处理大规模无约束优化的高效算法：

**关键特性**：
- **有限内存**：仅存储最近 $m$ 步（通常 $m=10$）的曲率信息，而非完整Hessian矩阵
- **线搜索**：采用Wolf准则确定步长，保证收敛
- **高维高效**：每次迭代复杂度 $O(mn)$，$n \approx 130$ 为变量数，$m=10$ 为内存限制
- **快速收敛**：二次函数通常几十次迭代收敛，非凸函数收敛到局部最优

**实现库**：使用开源LBFGS-Lite库（位于 `3rd/LBFGS-Lite/include`），已集成至本项目编译系统。

**输出重采样**：LBFGS优化完成后，为适应控制器对密集轨迹的需求，优化器会将**优化后的轨迹点** $\mathbf{p}_{\text{opt}}$ 通过分段线性插值重采样至 **100 个均匀间距的时间点**（见 lbfgs_optimizer.cpp 第 68 行），生成供控制器使用的密集参考轨迹。这一步**不会改变点数本身的约束关系**，只是为输出进行密度调整。重采样的输出点数（100）与优化时的自由变量点数（通常 50-80）相独立。

### 2.2 三层代价函数详解与数学推导

轨迹优化的目标通过三层相互独立但加权平衡的代价函数实现：

#### 2.2.1 障碍物代价函数 $f_{\mathrm{obs}}(\mathbf{p})$

**目标**：确保轨迹与自由空间的距离充分，避免碰撞同时不过度约束。

**数学形式**：

$$f_{\mathrm{obs}}(\mathbf{p}) = \sum_{i=0}^{N} c_{\mathrm{obs}}(d_i), \quad c_{\mathrm{obs}}(d) = \begin{cases}
(\mathrm{obs\_dist\_max} - d)^2, & 0 \le d < \mathrm{obs\_dist\_max}\\
0, & d \ge \mathrm{obs\_dist\_max}
\end{cases}$$

其中 $d_i$ 为轨迹点 $(x_i, y_i)$ 到**最近障碍物的有向距离**（通过costmap的DistanceLayer获取）。

**参数详解**：

- **$\mathrm{obs\_dist\_max}$**（**默认 0.3m**）：安全距离阈值
  - 相当于 $0.3 / 0.05 = 6$ 个地图栅格单元
  - 大于此值时无惩罚，避免远距离处的不必要约束
  - 推荐范围：0.2-0.5m（根据实际安全需求调整）

- **二次形式的意义**：
  - 当 $d \approx 0$（靠近障碍物）：$c_{\mathrm{obs}} \approx (\mathrm{obs\_dist\_max})^2$，最大惩罚
  - 当 $d = \mathrm{obs\_dist\_max}/2$：$c_{\mathrm{obs}} = (\mathrm{obs\_dist\_max}/2)^2 = 0.0225$（假设 $d_{max}=0.3$m）
  - 当 $d \ge \mathrm{obs\_dist\_max}$：$c_{\mathrm{obs}} = 0$，无惩罚

**梯度计算**：

$$\frac{\partial c_{\mathrm{obs}}}{\partial d} = \begin{cases}
-2(\mathrm{obs\_dist\_max} - d), & d < \mathrm{obs\_dist\_max}\\
0, & d \ge \mathrm{obs\_dist\_max}
\end{cases}$$

链式法则得到对坐标的偏导：
$$\frac{\partial f_{\mathrm{obs}}}{\partial (x_i, y_i)} = \frac{\partial c_{\mathrm{obs}}}{\partial d_i} \cdot \nabla_{(x_i,y_i)} d_i$$

其中 $\nabla_{(x_i,y_i)} d_i$ 是距离场在点 $(x_i, y_i)$ 的梯度，从DistanceLayer中获取（单位：m），指向距离增加的方向（即远离障碍物）。

**数值获取方式**：

```cpp
// 伪代码示例（实现位置：lbfgs_optimizer.cpp 行320-357）
for (int i = 0; i < N; ++i) {
  double d = distance_layer->getDistance(mx, my) * resolution;  // DistanceLayer查询
  if (d < obs_dist_max && d > 1e-6) {
    double diff = d - obs_dist_max;  // 负数
    f_obs += w_obstacle * diff * diff;  // (d - d_max)^2
    
    // 从距离场获取梯度方向（指向远离障碍物）
    double gx_raw, gy_raw;
    distance_layer->getGradient(mx, my, gx_raw, gy_raw);
    Eigen::Vector2d obs_dir(gx_raw, gy_raw);
    obs_dir.normalize();
    
    // 梯度：w_obs * 2 * (d - d_max) * d(d)/d(point)
    Eigen::Vector2d g_obs = obs_dir * (w_obstacle * 2.0 * diff * resolution);
    grad(0, i) += g_obs(0);
    grad(1, i) += g_obs(1);
  }
}
```

#### 2.2.2 平滑性代价函数 $f_{\mathrm{smo}}(\mathbf{p})$

**目标**：惩罚轨迹上相邻点间的过度转折，使轨迹在几何上光滑，减少蛇形运动。

**数学形式**：

$$f_{\mathrm{smo}}(\mathbf{p}) = \sum_{i=1}^{N-1} \left\|\Delta^2 \mathbf{p}_i\right\|_2^2, \quad \Delta^2 \mathbf{p}_i = \mathbf{p}_{i+1} - 2\mathbf{p}_i + \mathbf{p}_{i-1}$$

**展开式**：

$$f_{\mathrm{smo}}(\mathbf{p}) = \sum_{i=1}^{N-1} \left[(x_{i+1} - 2x_i + x_{i-1})^2 + (y_{i+1} - 2y_i + y_{i-1})^2\right]$$

**几何意义**：

- $\Delta^2 \mathbf{p}_i$ 代表**离散二阶导数**或**离散曲率**，衡量路径在点 $i$ 的"弯曲程度"
- 最小化 $\|\Delta^2 \mathbf{p}_i\|$ 等价于使路径尽可能"平直"
- 当三个连续点共线（$\mathbf{p}_{i-1}, \mathbf{p}_i, \mathbf{p}_{i+1}$ 共线）时，$\Delta^2 \mathbf{p}_i = 0$，代价达最小

**梯度推导**：

对 $x$ 坐标，设 $\Delta^2 x_i = x_{i+1} - 2x_i + x_{i-1}$，则
$$\frac{\partial f_{\mathrm{smo}}}{\partial x_i} = 2\Delta^2 x_i \cdot \frac{\partial}{\partial x_i}(\Delta^2 x_i) + \ldots$$

计算 $\Delta^2 x_i$ 对各点的偏导：
- $\frac{\partial(\Delta^2 x_i)}{\partial x_{i-1}} = 1$（出现在 $i-1, i, i+1$ 这三项中）
- $\frac{\partial(\Delta^2 x_i)}{\partial x_i} = -2$（出现在 $i-1, i, i+1$ 这三项中）
- $\frac{\partial(\Delta^2 x_i)}{\partial x_{i+1}} = 1$（出现在 $i-1, i, i+1$ 这三项中）

完整的梯度：
$$\frac{\partial f_{\mathrm{smo}}}{\partial x_i} = 2[(x_i - x_{i-1} - x_{i+1} + x_{i-2}) + (x_i - x_{i-1} - x_{i+1} + x_{i+2}) + \ldots]$$

代码实现中使用符号微分精确计算，避免数值微分误差。

**权重与效果**：

- **参数** $w_{\mathrm{smo}}$（**默认 0.25**）：权重越大，路径越光滑
- 当初始路径来自Hybrid A*（已相对光滑）时，$w_{\mathrm{smo}}$ 可设较小（如 0.1）
- 当初始路径来自栅格搜索（锯齿状）时，需增大 $w_{\mathrm{smo}}$（如 0.5-1.0）

#### 2.2.3 曲率约束代价函数 $f_{\mathrm{cur}}(\mathbf{p})$

**目标**：确保轨迹的曲率不超过车辆的运动学极限，保证物理可行性。

**数学形式**：

$$f_{\mathrm{cur}}(\mathbf{p}) = \sum_{i=1}^{N-1} c_{\mathrm{cur}}(\kappa_i), \quad c_{\mathrm{cur}}(\kappa) = \begin{cases}
\left(\frac{\kappa - \kappa_{\max}}{\kappa_{\max}}\right)^2, & \kappa > \kappa_{\max}\\
0, & \kappa \le \kappa_{\max}
\end{cases}$$

**离散曲率计算**：

基于三点圆弧拟合，在点 $\mathbf{p}_i$ 处的离散曲率：

$$\kappa_i = \frac{2 |\overrightarrow{\mathbf{p}_{i-1}\mathbf{p}_i} \times \overrightarrow{\mathbf{p}_i\mathbf{p}_{i+1}}|}{|\overrightarrow{\mathbf{p}_{i-1}\mathbf{p}_i}| \cdot |\overrightarrow{\mathbf{p}_i\mathbf{p}_{i+1}}| \cdot |\overrightarrow{\mathbf{p}_{i-1}\mathbf{p}_{i+1}}|}$$

二阶差分近似：

$$\kappa_i \approx \frac{\left|\mathbf{p}_{i+1} - 2\mathbf{p}_i + \mathbf{p}_{i-1}\right|}{\left(\frac{|\mathbf{p}_i - \mathbf{p}_{i-1}| + |\mathbf{p}_{i+1} - \mathbf{p}_i|}{2}\right)^2}$$

**参数说明**：

- **$\kappa_{\max}$**（**默认 0.15 rad/m**）：最大允许曲率
  - 对应最小转弯半径 $R_{\min} = 1/\kappa_{\max} \approx 6.7$ m
  - 应与Hybrid A*搜索过程中的最小转弯半径一致，保证全局一致
  - 推荐范围：0.1-0.3 rad/m（取决于车辆实际转向能力）

**物理含义**：

对于Ackermann转向车辆（地面自主车常见模型），前轮转向角 $\delta$ 与曲率的关系：
$$\kappa = \frac{\tan \delta}{L}$$

其中 $L$ 为轴距。约束 $\kappa \le \kappa_{\max}$ 等价于约束 $|\delta| \le \arctan(\kappa_{\max} L)$。

**梯度计算**：

$$\frac{\partial c_{\mathrm{cur}}}{\partial \kappa} = \begin{cases}
2 \frac{\kappa - \kappa_{\max}}{(\kappa_{\max})^2}, & \kappa > \kappa_{\max}\\
0, & \kappa \le \kappa_{\max}
\end{cases}$$

关于坐标的梯度通过链式法则：
$$\frac{\partial f_{\mathrm{cur}}}{\partial (x_i, y_i)} = \frac{\partial c_{\mathrm{cur}}}{\partial \kappa_i} \cdot \frac{\partial \kappa_i}{\partial (x_i, y_i)}$$

**实现细节**（位置：`lbfgs_optimizer.cpp` 行375-530）：

代码计算**三个重叠的曲率点**以确保全局光滑性：

- **$\kappa_p$**：在点 $\mathbf{p}_{i-1}$ 处，由两条线段 $(\mathbf{p}_{i-2} \to \mathbf{p}_{i-1})$ 和 $(\mathbf{p}_{i-1} \to \mathbf{p}_i)$ 组成的角度变化
- **$\kappa_c$**：在点 $\mathbf{p}_i$ 处，由两条线段 $(\mathbf{p}_{i-1} \to \mathbf{p}_i)$ 和 $(\mathbf{p}_i \to \mathbf{p}_{i+1})$ 组成的角度变化  
- **$\kappa_n$**：在点 $\mathbf{p}_{i+1}$ 处，由两条线段 $(\mathbf{p}_i \to \mathbf{p}_{i+1})$ 和 $(\mathbf{p}_{i+1} \to \mathbf{p}_{i+2})$ 组成的角度变化

其中每条线段的曲率定义为：$\kappa = \frac{\Delta \phi}{\ell}$，$\Delta \phi$ 为相邻两线段的夹角，$\ell$ 为前一条线段长度。

**成本与梯度权重**：
- 仅当 $\kappa_c > \kappa_{\max}$ 时累积成本：$c_{\mathrm{cur}} = (\kappa_c - \kappa_{\max})^2$
- 梯度计算中三个曲率的权重分别为 $[0.25, 0.5, 0.25]$，确保路径的整体曲率平衡

### 2.3 参数配置与权重平衡

**配置文件**：`src/sim_env/config/trajectory_optimization/lbfgs_params.yaml`

```yaml
Optimizer:
  max_iter: 100              # 最大迭代次数
  obs_dist_max: 0.3          # 障碍物安全距离（米）
  k_max: 0.15                # 最大曲率（rad/m）
  w_obstacle: 0.5            # 障碍物代价权重
  w_smooth: 0.25             # 平滑代价权重
  w_curvature: 0.25          # 曲率代价权重
```

**权重调优指南**：

| 优化目标 | 推荐配置 | 效果 |
|---------|---------|------|
| **避障优先** | $w_{\mathrm{obs}} \gg w_{\mathrm{smo}}, w_{\mathrm{cur}}$ | 路径远离障碍，但可能不光滑 |
| **光滑优先** | $w_{\mathrm{smo}} \gg w_{\mathrm{obs}}, w_{\mathrm{cur}}$ | 路径光滑，但可能过度靠近障碍 |
| **平衡（推荐）** | $w_{\mathrm{obs}} : w_{\mathrm{smo}} : w_{\mathrm{cur}} \approx 1:0.5:0.5$ | 三者兼顾，综合性能最优 |

### 2.4 梯度计算与L-BFGS-B求解

#### 2.4.1 符号微分的梯度计算

所有代价函数的梯度通过**符号微分**精确推导，无数值近似：

**算法流程**：

1. 对每个代价项 $f_{\mathrm{obs}}, f_{\mathrm{smo}}, f_{\mathrm{cur}}$ 推导 $\frac{\partial}{\partial (x_i, y_i)}$ 的闭形式
2. 在每次迭代时，直接代入数值计算梯度向量 $\nabla f = [\frac{\partial f}{\partial x_0}, \frac{\partial f}{\partial y_0}, \ldots]^T$
3. 避免有限差分 $\frac{f(\mathbf{p}+\epsilon \mathbf{e}_i) - f(\mathbf{p})}{\epsilon}$ 的舍入误差与数值噪声

**实现位置**：`lbfgs_optimizer.cpp` 中的 `computeGradient()` 函数，调用：
- `computeObstacleCostGradient()`
- `computeSmoothnessCostGradient()`
- `computeCurvatureCostGradient()`

#### 2.4.2 L-BFGS-B迭代算法

**迭代更新规则**：

给定当前点 $\mathbf{p}_k$，梯度 $\mathbf{g}_k = \nabla f(\mathbf{p}_k)$，

1. **构造搜索方向**：$\mathbf{d}_k = -H_k \mathbf{g}_k$（$H_k$ 为有限内存Hessian近似）
2. **线搜索**（Wolf准则）：找步长 $\alpha_k$ 使得
   - 足够下降：$f(\mathbf{p}_k + \alpha_k \mathbf{d}_k) \le f(\mathbf{p}_k) + c_1 \alpha_k \mathbf{d}_k^T \mathbf{g}_k$（$c_1 = 10^{-4}$）
   - 曲率条件：$(\mathbf{g}_k + \alpha_k \mathbf{d}_k)^T \mathbf{d}_k \ge c_2 \mathbf{d}_k^T \mathbf{g}_k$（$c_2 = 0.9$）
3. **更新**：$\mathbf{p}_{k+1} = \mathbf{p}_k + \alpha_k \mathbf{d}_k$
4. **维护历史**：记录向量对 $(\mathbf{s}_k, \mathbf{y}_k)$ 其中 $\mathbf{s}_k = \mathbf{p}_{k+1} - \mathbf{p}_k$，$\mathbf{y}_k = \mathbf{g}_{k+1} - \mathbf{g}_k$
5. **收敛判定**：若 $\|\mathbf{g}_k\|_2 < \epsilon_g$（通常 $10^{-6}$）或迭代数超过 `max_iter` 则停止

**收敛特性**：

- 二次函数：理论上有限步内收敛（实践通常20-50步）
- 非凸函数（本问题属于，多局部极值）：收敛到局部最优
- 初值质量影响收敛速度：RDP提供的初值通常已接近局部最优，加速收敛

#### 2.4.3 性能分析

**计算复杂度**：

- 每次迭代梯度计算：$O(N)$（遍历所有 $N+1$ 个轨迹点）
- 每次迭代线搜索：$O(m \cdot N)$（$m$ 次函数评估，每次 $O(N)$）
- 总时间：$k_{\mathrm{iter}} \times m \times O(N)$，其中 $k_{\mathrm{iter}} \approx 30-50$，$m \approx 3-5$，$N \approx 65$

**典型性能指标**（单核CPU，分辨率0.05m）：

| 轨迹点数 | 优化时间 | 迭代次数 | 可行性 |
|--------|---------|---------|--------|
| 50 | 20-40 ms | 15-30 | 100% |
| 65 | 30-80 ms | 20-50 | 99%+ |
| 100 | 80-150 ms | 30-80 | 95%+ |
| 150 | 150-300 ms | 40-100 | 90%+ |

**实时性保证**：在10 Hz控制周期（100 ms）内，65个点的轨迹通常能完成优化。

### 2.5 与局部规划的衔接

LBFGS优化后的全局参考轨迹 $\tau_g = \{\mathbf{p}_0^*, \ldots, \mathbf{p}_N^*\}$ 不直接用于局部规划，而是经过**下采样**处理：

**下采样策略**：

按**固定弧长间距** $\Delta s = 0.5-1.0$ m 重采样，得到稀疏的参考点集 $\{(x_r^{(0)}, y_r^{(0)}), \ldots, (x_r^{(M)}, y_r^{(M)})\}$，$M \ll N$。

**实现方式**：

```cpp
std::vector<PoseStamped> reference = downSampleByArcLength(optimized_path, 0.5);
// reference 包含约 10-15 个点，作为HLP的参考曲线
```

**衔接的优势**：

1. **减少计算**：避免局部规划每 0.1 秒都调用全局Hybrid A*搜索（开销50-200ms）
2. **保持导引**：稀疏参考点足以引导局部规划向全局目标，同时给予局部充分自由度
3. **提高鲁棒性**：局部规划能实时响应动态障碍，同时不偏离全局意图
4. **优化质量**：LBFGS的光滑轨迹作为初值，加速局部搜索收敛

**参考轨迹的用途**：在HLP（第3节）的"线速度分配"（3.4节）中，沿参考轨迹统计障碍物代价，自适应调整目标速度。

---

# 第一部分增补：分层框架的重规划策略

## 2.5 事件触发式重规划机制

全局规划的输出（LBFGS优化后的参考轨迹）为局部规划提供高层指引，但在动态环境或局部规划失败时需要及时更新。本节阐述事件触发式重规划策略，确保全局与局部规划的协调与鲁棒性。

### 2.5.1 分层框架耦合的核心思想

本系统采用**事件触发式重规划**（Event-Triggered Replanning）策略实现全局与局部规划的耦合，其核心原则为：

1. **全局规划层**：定期或按需生成初始可行路径 $\pi_0$（Hybrid A*），后经LBFGS优化得到光滑参考轨迹 $\tau_g$；
2. **局部规划层**：基于当前位姿、局部代价地图与全局参考轨迹实时生成可执行控制量 $(v,\omega)$；
3. **失败触发**：当局部规划连续失败（无有效轨迹、碰撞或数值发散）超过阈值时，立即触发全局重规划；
4. **自适应回退**：避免周期性重规划的冗余计算，仅在必要时重新规划路径。

**与周期性重规划的对比**：
| 方面 | 周期性重规划 | 事件触发式重规划 |
|------|-------------|-----------------|
| **触发条件** | 固定频率（e.g., 1Hz） | 规划失败 OR 检测到环境突变 |
| **计算开销** | 持续调用全局规划器 | 仅在必要时调用 |
| **实时响应** | 延迟固定 | 快速响应环境变化 |
| **应用场景** | 已知环境、低动态性 | 动态环境、高响应需求 |

### 2.5.2 失败检测与重规划触发条件

#### 规划失败的判断标准

局部规划的失败可由以下条件判定：

1. **轨迹碰撞失败**：生成的局部轨迹与障碍物集合相交，即存在轨迹点 $(x_i,y_i)$ 使得距障碍物距离 $d_i < d_{\min}$（安全裕度）；

2. **无可行解**：在给定的时间或空间预算内，搜索无法找到满足动力学与走廊约束的轨迹；

3. **数值发散**：优化过程中成本函数值或梯度范数发散，表示算法数值不稳定；

4. **控制超时**：连续 $T_{\mathrm{fail}}$ 时间内（通常 0.2-0.5s）无法产生有效速度命令；

5. **走廊约束违反**：若启用凸走廊约束，轨迹末端点落在走廊外。

#### 重规划触发的时间逻辑

定义失败计数器 $n_{\mathrm{fail}}$ 与失败时间累计器 $T_{\mathrm{fail}}$：

$$T_{\mathrm{fail}} \mathrel{+}= T_s \ \text{(每个控制周期)}$$

当 $T_{\mathrm{fail}} > T_{\mathrm{fail}}^{\max}$（通常 0.2-0.5 秒）时，触发全局重规划：

$$\text{trigger\_replanning} \Leftarrow \begin{cases} \text{True}, & T_{\mathrm{fail}} > T_{\mathrm{fail}}^{\max} \\ \text{False}, & \text{otherwise} \end{cases}$$

一旦重规划被触发，重置 $T_{\mathrm{fail}} \leftarrow 0$，并清空旧路径：$\tau_g \leftarrow \text{NULL}$。

### 2.5.3 全局重规划的执行流程

当事件触发重规划时，系统执行以下步骤：

$$\begin{aligned}
\text{Step 1:} \quad & \text{当前位姿}\ \mathbf{x}_{\mathrm{cur}} \leftarrow \text{odometry或SLAM输出} \\
\text{Step 2:} \quad & \text{更新全局栅格地图}\ M_{\mathrm{global}} \\
\text{Step 3:} \quad & \pi_0 \leftarrow \text{HybridAStar}(\mathbf{x}_{\mathrm{cur}}, \mathbf{x}_{\mathrm{goal}}, M_{\mathrm{global}}) \\
\text{Step 4:} \quad & \pi_{\mathrm{rdp}} \leftarrow \text{RDP}(\pi_0, \delta_{\mathrm{rdp}}) \\
\text{Step 5:} \quad & \tau_g \leftarrow \text{LBFGS}(\pi_{\mathrm{rdp}}, M_{\mathrm{global}}) \\
\text{Step 6:} \quad & \text{重置局部规划状态}\ (\mathbf{u}_{\mathrm{prev}}, \alpha) \\
\text{Step 7:} \quad & \text{下一控制周期恢复局部规划}
\end{aligned}$$

**关键要点**：
- **Step 3** 中Hybrid A*从当前位置重新搜索，避免规划已经执行过的路径；
- **Step 4-5** 的RDP与LBFGS优化过程相同，保证参考轨迹的平滑性；
- **Step 6** 清空前瞻点跟踪器与MPC的历史状态，防止控制延迟产生的耦联效应；
- **Step 7** 后的第一个控制周期会立即调用局部规划，基于新的 $\tau_g$ 重新规划。

### 2.5.4 与移动机器人框架的集成

在ROS move_base框架中，重规划机制通过以下参数配置实现：

| 参数 | 值 | 含义 |
|------|-----|------|
| `planner_frequency` | 0.0 | 关闭周期性全局重规划；仅事件触发时调用 |
| `controller_frequency` | 10.0 Hz | 局部规划（HLP）的执行频率 |
| `controller_patience` | 0.2 s | 局部规划失败超过此时间则触发全局重规划 |
| `oscillation_timeout` | 10.0 s | 振荡检测时间窗 |
| `oscillation_distance` | 0.2 m | 振荡检测阈值 |

**参数含义**：
- `planner_frequency = 0.0` 表示全局规划器（Hybrid A*）仅在以下情况被调用：
  1. 启动时的初始规划
  2. 局部规划失败触发的事件触发重规划
  
- `controller_patience = 0.2 s` 是最关键的触发参数，定义了局部规划允许的最大连续失败时间。超过此阈值时，move_base框架自动清空当前全局路径，触发重规划请求。

- `oscillation_distance = 0.2 m` 与 `oscillation_timeout = 10.0 s` 组合定义振荡检测：若过去10秒内机器人移动范围小于0.2米，认为陷入局部振荡，需要重新规划。

### 2.5.5 重规划成本分析与性能保证

**计算复杂度对比**：

1. **周期性重规划**（每1Hz调用全局规划）：
   - 全局规划开销：每秒调用1次 × 50-200ms/次 = 50-200ms开销
   - 总体时间占用：5-20%

2. **事件触发重规划**（平均每10秒失败一次）：
   - 全局规划开销：每10秒调用1次 × 50-200ms/次 = 5-20ms平均开销
   - 总体时间占用：0.5-2%

**可靠性保证**：
- 从当前位置重新规划可避免陈旧信息导致的次优路径；
- 通过失败时间累计防止抖动，避免频繁重规划导致的不稳定；
- 若连续重规划仍失败（e.g., 目标被完全包围），系统进入安全状态（停止或原地旋转）。

---

# 第二部分：局部规划层（HLP + MPC）

## 3. HLP（分层局部规划）的数学原理与实现

### 3.1 处理时延补偿的前向预测

#### 3.1.1 时延问题的来源与影响

在实际移动机器人系统中，从**感知当前位姿** → **规划轨迹** → **执行控制命令**需要消耗时间，称为**处理时延** $T_p$。时延的存在导致以下问题：

1. **状态陈旧**：规划时使用的位姿 $(x,y,\theta)$ 基于 $T_p$ 秒前的观测，到规划完成时机器人已经运动了一段距离
2. **规划-执行偏差**：规划基于陈旧状态，执行时当前状态已改变，可能导致已规划轨迹不再可行
3. **滞后效应**：特别是在动态环境中，陈旧的状态信息会导致对障碍物位置的错误判断

#### 3.1.2 前向预测补偿

为缓解时延影响，采用**前向预测**（Look-ahead）策略：假设在处理时延 $T_p$ 内，机器人以当前的控制量 $(v, \omega)$ 匀速直线运动，先对位姿进行预测，再以预测位置为起点进行可达性分析和轨迹搜索。

**预测模型**（一阶微分方程离散化）：

$$\begin{aligned}
\theta'(T_p) &= \theta(0) + \omega(0) \cdot T_p \\
x'(T_p) &= x(0) + v(0) \cdot T_p \cdot \cos[\theta'(T_p)] \\
y'(T_p) &= y(0) + v(0) \cdot T_p \cdot \sin[\theta'(T_p)]
\end{aligned}$$

后续所有规划均基于预测状态 $\mathbf{x}' = (x', y', \theta')$ 展开，可达图与Dijkstra搜索的起始点也从原始状态改为预测状态。

**典型参数**：
- $T_p \approx 20-50$ ms（包括感知延迟、通信延迟、规划计算时间）
- 在 $v = 1$ m/s，$\omega = 0.5$ rad/s 的情况下，$T_p = 30$ ms 时补偿位移约 3cm，角度变化约 1.5°

**实现位置**：`hybrid_planner.cpp` 的 `computeVelocityCommands()` 函数开始，调用 `predictState()` 获得预测位置。

### 3.2 分层可达状态图：离散可达集与边代价

#### 3.2.1 可达性分析的分层离散化

HLP在**时间-角度-距离**的三维空间内进行离散化，构造一个稀疏有向图。该离散化基于机器人的运动动力学约束，确保图中所有节点都是在控制周期 $T_s$ 内**物理可达**的。

**离散维度说明**：

1. **时间维**（前向可达时间）：$L$ 层，第 $\ell$ 层对应时刻 $t_\ell = (\ell+1) T_s$，$\ell = 0,1,\ldots,L-1$
   - 物理意义：机器人从当前位置，用最多 $(\ell+1)$ 个控制周期能到达的节点
   - 典型值：$L = 5-10$，对应规划预视距离 0.5-1.0 秒

2. **角度维**（朝向离散）：第 $\ell$ 层采样 $M$ 个朝向 $\theta_{\ell,0}, \theta_{\ell,1}, \ldots, \theta_{\ell,M-1}$
   - 可达的角度范围由当前角速度与角加速度限制确定（见3.2.2）
   - 典型值：$M = 16-32$，对应角度分辨率 $\Delta\theta = 11.25°-22.5°$

3. **距离维**（径向距离）：在距离范围 $[d_{\min}, d_{\max}]$ 上均匀采样 $L$ 个值
   - 可达距离由线速度与线加速度限制确定（见3.2.3）
   - 物理意义：机器人的最慢与最快运动都被考虑了

**节点总数**：$N_{\text{node}} = L \times M$，通常 $5 \times 16 = 80$ 到 $10 \times 32 = 320$ 个节点。相比栅格化搜索（通常 50000+ 个栅格），计算量显著降低。

#### 3.2.2 角速度可达范围的计算

在当前时刻，机器人的角速度为 $\omega_0$，最大角速度为 $\omega_{\max}$，角加速度上界为 $a_\omega$。在时间 $t_\ell = (\ell+1)T_s$ 内，可达的左右极限朝向分别为：

**右转极限**（假设 $\omega_0 \le 0$，机器人要向右转到正的角速度）：

在可达时间内，角速度可达的最大值为：
$$\omega_{\mathrm{right}}^{+} = \min(\omega_0 + a_\omega \cdot t_\ell, \omega_{\max})$$

由 $\omega(t) = \omega_0 + a_\omega t$ 积分得角度变化：
$$\Delta \theta_{\mathrm{right}} = \int_0^{t_\ell} \omega(t)\,dt$$

分段计算：若 $\omega_0 + a_\omega t_\ell \ge \omega_{\max}$，则在某时刻 $t^*$ 达到 $\omega_{\max}$，之后匀速转动：
$$t^* = \frac{\omega_{\max} - \omega_0}{a_\omega}, \quad \Delta\theta_{\mathrm{right}} = \omega_0 t^* + \frac{1}{2}a_\omega(t^*)^2 + \omega_{\max}(t_\ell - t^*)$$

若 $\omega_0 + a_\omega t_\ell < \omega_{\max}$，则全程加速：
$$\Delta\theta_{\mathrm{right}} = \omega_0 t_\ell + \frac{1}{2}a_\omega t_\ell^2$$

**左转极限**类似推导，只需将 $a_\omega, \omega_{\max}$ 换为负值。

最终，第 $\ell$ 层可采样的角度范围为：
$$\theta \in [\theta_0 + \Delta\theta_{\mathrm{left}}, \theta_0 + \Delta\theta_{\mathrm{right}}]$$

在此范围内均匀取 $M$ 个样本。

#### 3.2.3 线速度可达范围的计算

设当前线速度为 $v_0$，最小线速度为 $v_{\min}$，最大线速度为 $v_{\max}$，线加速度上界为 $a_v$。可达距离范围为 $[d_{\min}, d_{\max}]$。

**最小距离计算**（最坏情况：立即制动）：

若需在一个控制周期内从 $v_0$ 减速到 $v_{\min}$，所需时间为：
$$t_{\mathrm{brake}} = \frac{|v_{\min} - v_0|}{a_v}$$

若 $t_{\mathrm{brake}} < T_s$，则先减速后匀速：
$$d_{\min} = \int_0^{t_{\mathrm{brake}}} (v_0 - a_v t)\,dt + v_{\min}(T_s - t_{\mathrm{brake}}) = v_0 t_{\mathrm{brake}} - \frac{1}{2}a_v t_{\mathrm{brake}}^2 + v_{\min}(T_s - t_{\mathrm{brake}})$$

若 $t_{\mathrm{brake}} \ge T_s$，则全程减速：
$$d_{\min} = v_0 T_s - \frac{1}{2}a_v T_s^2$$

为考虑局部预视，实际使用的预视距离 $T_{\mathrm{cap}}$（通常 1-2秒）比单个控制周期 $T_s$ 长得多：
$$d_{\min} = v_0 T_{\mathrm{cap}} - \frac{1}{2}a_v T_{\mathrm{cap}}^2$$

**最大距离计算**（最好情况：尽快加速）：

若能在 $t_{\mathrm{accel}} = \frac{v_{\max} - v_0}{a_v}$ 内加速到 $v_{\max}$：
- 若 $t_{\mathrm{accel}} < T_{\mathrm{cap}}$，则加速后匀速：
  $$d_{\max} = v_0 t_{\mathrm{accel}} + \frac{1}{2}a_v t_{\mathrm{accel}}^2 + v_{\max}(T_{\mathrm{cap}} - t_{\mathrm{accel}})$$

- 若 $t_{\mathrm{accel}} \ge T_{\mathrm{cap}}$，则全程加速：
  $$d_{\max} = v_0 T_{\mathrm{cap}} + \frac{1}{2}a_v T_{\mathrm{cap}}^2$$

在范围 $[d_{\min}, d_{\max}]$ 上**均匀离散** $L$ 个距离值 $\{d_\ell\}$。

#### 3.2.4 节点与边的构造

**节点的二维坐标**：第 $\ell$ 层、第 $m$ 个角度的节点位置为：
$$\mathbf{p}_{\ell,m} = \begin{pmatrix} x' + d_\ell \cos\theta_{\ell,m} \\ y' + d_\ell \sin\theta_{\ell,m} \end{pmatrix}$$

其中 $(x', y')$ 为预测的机器人位置。

**节点代价**（三项代价和）：
$$J(\mathbf{p}_{\ell,m}) = \tilde{J}_{\mathrm{obs}}(\mathbf{p}_{\ell,m}) + \tilde{J}_{\mathrm{path}}(\mathbf{p}_{\ell,m}) + \tilde{J}_{\mathrm{goal}}(\mathbf{p}_{\ell,m})$$

其中：
- $\tilde{J}_{\mathrm{obs}}$：局部代价地图的占据/膨胀代价，负值表示进入障碍物（不可行）
- $\tilde{J}_{\mathrm{path}}$：节点偏离全局参考路径的代价，鼓励跟踪参考轨迹
- $\tilde{J}_{\mathrm{goal}}$：节点到局部目标点的距离，鼓励向目标靠近

**边的连接**：每个节点与前一层的**最近三个角度邻居**相连，形成稀疏有向图。这样的连接方式保证了图的稀疏性（每个节点最多3条出边），同时保留了主要的可达路径。

**边代价**：边 $(i \to j)$ 的代价为起点代价与距离的加权和：
$$J_e(i \to j) = J(\mathbf{p}_i) + \lambda_d \cdot \|\mathbf{p}_j - \mathbf{p}_i\|_2$$

其中 $\lambda_d$ 是距离权重系数（通常 0.5-1.0）。对于不可行节点（进入障碍物），其相关边代价设为 $+\infty$，在最短路搜索中自动规避。

### 3.3 Dijkstra局部搜索：最小代价参考折线

#### 3.3.1 最短路算法的适用性

在上述构造的有向无环图（DAG）上执行**Dijkstra算法**，找到从起始状态到**最后一层**（任意朝向）的最小累计代价路径。

**为什么用Dijkstra而非A***：
- 图本身已经是分层结构，从第0层到第L-1层的拓扑是已知的（DAG），无需启发式函数加速搜索
- 图的节点数远少于全地图栅格数（$< 500$ vs $> 50000$），Dijkstra的 $O(N \log N)$ 复杂度已足够快
- 实现简单，数值稳定性好

**算法流程**：

1. **初始化**：设起始节点（第0层对应当前预测位置）累计代价为0，其他节点为 $+\infty$
2. **优先队列循环**：
   - 取出代价最小的未处理节点 $u$
   - 对其所有后继节点 $v$，更新 $\text{cost}(v) = \min[\text{cost}(v), \text{cost}(u) + J_e(u \to v)]$
   - 若发生更新，将 $v$ 插入优先队列
3. **回溯**：当所有最后一层节点都被处理后，从代价最小的终点节点回溯找到最优路径

**时间复杂度**：$O(N \log N)$，其中 $N = L \times M < 500$。在实践中通常 5-10 ms 完成。

#### 3.3.2 搜索结果与局部参考折线

Dijkstra算法返回一条节点序列 $\{\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_L\}$，称为**局部参考折线**。这条折线：
- 在时间上跨越规划预视范围（通常 1-2 秒）
- 在空间上是分段直线段的折线
- 自动综合了避障、跟踪全局参考、向目标靠近的多重目标

**折线的特性**：
- **可行性**：由于在构造时已考虑动力学约束，该折线在运动学上总是可达的（无论是否无碰撞）
- **实时性**：获得折线仅需 5-20 ms，远低于10 Hz控制周期（100 ms）的需求
- **自适应性**：动态障碍、路径变更等都会在下一个控制周期重新计算，实时更新折线
$$\mathcal{P}=\{\mathbf{p}_0,\mathbf{p}_1,\ldots,\mathbf{p}_K\}.$$
该序列可视为局部参考折线，用于后续速度分配与角速度优化的"几何骨架"。

### 3.4 线性速度分配：基于代价的自适应减速

#### 3.4.1 沿参考轨迹的障碍代价统计

从Dijkstra搜索得到的参考折线提供了空间导向（路径），但没有分配沿该路径的速度。速度分配的核心思想是：**沿参考折线逐点查询局部代价地图，如果某段轨迹周围障碍代价较高（拥挤），则在该段减速；如果周围空旷，则可提高速度**。

设参考折线由 $K$ 个离散点 $\{\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_{K-1}\}$ 组成，各点间的直线距离为 $s_k$。对折线上的第 $k$ 个点，定义其**局部障碍代价** $\tilde{J}_{\text{obs},k}$ 为该点周围一定范围（通常 0.5m 半径圆形区域）内代价地图的最大值：

$$\tilde{J}_{\text{obs},k} = \max_{\|\mathbf{q} - \mathbf{p}_k\|_2 \le r_{\text{query}}} \text{CostMap}(\mathbf{q})$$

其中 $r_{\text{query}} = 0.5$ m 是查询半径，$\text{CostMap}(\mathbf{q})$ 返回栅格 $\mathbf{q}$ 处的代价值（0-255范围）。

对整条折线，计算沿其长度的**最大障碍代价** $\tilde{J}_{\text{obs}}^{\max}$ 和上界值 $\tilde{J}_{\text{obs}}^{\text{upper}}$（通常为代价地图的膨胀阈值，默认 200）：

$$\tilde{J}_{\text{obs}}^{\max} = \max_k \tilde{J}_{\text{obs},k}, \quad \tilde{J}_{\text{obs}}^{\text{upper}} = 200$$

#### 3.4.2 拥挤度指标与速度缩放因子

引入**拥挤度指标** $\rho_{\text{obs}}$，定义为折线最大代价与上界值的比值：

$$\rho_{\text{obs}} = \frac{\tilde{J}_{\text{obs}}^{\max}}{\tilde{J}_{\text{obs}}^{\text{upper}}}$$

取值范围 $\rho_{\text{obs}} \in [0, 1]$：
- 当 $\rho_{\text{obs}} \approx 0$ 时，折线周围空旷，可高速运动
- 当 $\rho_{\text{obs}} \approx 1$ 时，折线周围拥挤，需低速谨慎运动

基于拥挤度定义**速度缩放因子** $\eta(\rho_{\text{obs}})$，采用分段线性函数：

$$\eta(\rho_{\text{obs}}) = \begin{cases}
1.0 & \text{if } \rho_{\text{obs}} < 0.3 \quad \text{（空旷，全速）}\\
1.0 - 0.7 \cdot \frac{\rho_{\text{obs}} - 0.3}{0.7} & \text{if } 0.3 \le \rho_{\text{obs}} < 1.0 \quad \text{（线性递减）}\\
0.3 & \text{if } \rho_{\text{obs}} \ge 1.0 \quad \text{（拥挤，降至30\%）}
\end{cases}$$

或简写为：
$$\eta(\rho_{\text{obs}}) = \max\left(0.3,\, 1.0 - 0.7 \cdot \max(0, \rho_{\text{obs}} - 0.3)/0.7\right)$$

**参数解释**：
- 阈值 0.3：代价比低于30%时认为空旷，无需减速
- 斜率 0.7：从30%拥挤度到100%时，速度线性衰减至30%
- 最低因子 0.3：即使最拥挤仍保留30%最大速度，确保必要时能缓慢通过

#### 3.4.3 指令速度的计算与约束

原始指令速度为：
$$v_{\text{cmd}} = v_{\max} \cdot \eta(\rho_{\text{obs}})$$

其中 $v_{\max}$ 是机器人的最大线速度（通常 0.5-1.0 m/s）。

为防止速度发生剧烈跳变（可能导致控制振荡），还需对指令速度进行**加速度限制**：

$$v_{\text{final}} = \begin{cases}
v_{\text{cmd}} & \text{if } |v_{\text{cmd}} - v_{\text{last}}| \le a_v T_s \\
v_{\text{last}} + \text{sign}(v_{\text{cmd}} - v_{\text{last}}) \cdot a_v T_s & \text{otherwise}
\end{cases}$$

其中：
- $v_{\text{last}}$ 是上一个控制周期的速度
- $a_v$ 是最大线加速度（通常 0.5 m/s²）
- $T_s$ 是控制周期（通常 0.1 s，对应 10 Hz）
- 限制幅度：$|v_{\text{final}} - v_{\text{last}}| \le a_v T_s = 0.05$ m/s

最后，对最终速度进行下界截断：
$$v_{\text{output}} = \max(v_{\text{final}}, v_{\min})$$

确保速度不低于 $v_{\min} = 0.05$ m/s（防止机器人静止）。

#### 3.4.4 局部目标点的选择

基于指令速度 $v_{\text{output}}$ 与规划预视窗口时间 $T_{\text{cap}}$（通常 1.0-2.0 秒），计算机器人在该时间窗口内能行进的最大弧长预算：

$$D_{\text{budget}} = v_{\text{output}} \cdot T_{\text{cap}}$$

沿参考折线从起点开始累计弧长，在累计距离首次超过 $D_{\text{budget}}$ 的位置确定**局部目标点** $\mathbf{p}_{\text{local}}$ 及其索引 $k_g$。这样，$\mathbf{p}_{\text{local}}$ 就是参考折线上在时间预视窗口内距离起点约 $D_{\text{budget}}$ 的点。

**近终点处理**：当全局终点与机器人当前位置的距离 $d_{\text{goal}} = \|\mathbf{p}_{\text{current}} - \mathbf{p}_{\text{goal}}\|_2$ 小于近目标减速距离 $d_{\text{slow}} = v_{\max} T_{\text{decel}}$（其中 $T_{\text{decel}}$ 是减速时间常数，通常 2.0 秒）时，不再按时间预视选择，而是直接取全局终点作为局部目标，并按线性函数减速：

$$v_{\text{output}} \leftarrow v_{\max} \cdot \frac{d_{\text{goal}}}{d_{\text{slow}}}$$

这确保了当接近终点时，速度逐渐衰减至零。

**实现位置**：`hybrid_planner.cpp` 的 `allocateLinearSpeed()` 函数，lines 850-920。

### 3.5 角速度优化：基于动力学的目标朝向选择与姿态调整

#### 3.5.1 局部目标处的参考朝向

参考折线是一序列点 $\{\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_{K-1}\}$，但不包含各点的朝向信息。为提供可靠的姿态控制目标，需为折线上的每一点计算其**参考朝向**。

对折线上第 $k$ 个点（非末点），定义其参考朝向为指向下一点的方向角：

$$\theta_{\text{ref},k} = \text{atan2}(\mathbf{p}_{k+1,y} - \mathbf{p}_{k,y},\, \mathbf{p}_{k+1,x} - \mathbf{p}_{k,x})$$

对最后一个点，取其与前一点的连线方向作为参考朝向。这样定义的参考朝向自然地反映了折线在该点的切向方向，是MPC追踪的理想目标。

#### 3.5.2 可达朝向的动力学约束窗口

机器人的角运动受角加速度限制。设当前朝向为 $\theta_0$，当前角速度为 $\omega_0$，最大角速度为 $\omega_{\max}$，最大角加速度为 $a_\omega$。在接下来的一个短时间窗口 $\Delta T$（通常 0.5-1.0 秒）内，考虑**先加速至 $\omega_{\max}$、再减速至目标角速度** 的两阶段过程，可达的朝向范围为：

**右转极限** $\theta_{\max}$（逆时针旋转）：

假设机器人首先以角加速度 $a_\omega$ 加速至 $\omega_{\max}$，用时 $t_1 = \frac{\omega_{\max} - \omega_0}{a_\omega}$（若 $\omega_0$ 已为正，则 $t_1$ 为负，实际加速时间更短）。之后以 $\omega_{\max}$ 匀速旋转，在剩余时间 $\Delta T - t_1$ 内累计的角位移为：

$$\Delta\theta_{\max} = \omega_0 t_1 + \tfrac12 a_\omega t_1^2 + \omega_{\max}(\Delta T - t_1)$$

整理得：
$$\theta_{\max} = \theta_0 + \Delta\theta_{\max}$$

**左转极限** $\theta_{\min}$（顺时针旋转）：

类似地，向反方向加速至 $-\omega_{\max}$，得到：
$$\theta_{\min} = \theta_0 + \omega_0 \Delta T - \tfrac12 a_\omega \Delta T^2$$

（假设全程角加速度为 $-a_\omega$，模拟最快减速/反向）

因此，可达朝向范围为 $[\theta_{\min}, \theta_{\max}]$。

#### 3.5.3 朝向目标的约束与平滑

将参考朝向 $\theta_{\text{ref},k_g}$（局部目标点的切向朝向）投影至可达窗口 $[\theta_{\min}, \theta_{\max}]$：

$$\theta_{\text{goal}} = \begin{cases}
\theta_{\text{ref},k_g} & \text{if } \theta_{\text{ref},k_g} \in [\theta_{\min}, \theta_{\max}] \\
\theta_{\min} & \text{if } \theta_{\text{ref},k_g} < \theta_{\min} \\
\theta_{\max} & \text{if } \theta_{\text{ref},k_g} > \theta_{\max}
\end{cases}$$

其直观含义是：若参考朝向在动力学约束允许的范围内，则直接采用；否则采用能靠近参考朝向最近的可达极限朝向。

为避免目标朝向在连续控制周期间发生抖动，对其应用**一阶低通滤波**：

$$\theta_{\text{goal}}^{\text{smooth}} = \alpha_\theta \theta_{\text{goal}} + (1-\alpha_\theta) \theta_{\text{goal}}^{\text{last}}$$

其中 $\alpha_\theta$ 是平滑系数（通常 0.5），$\theta_{\text{goal}}^{\text{last}}$ 是上一个控制周期的平滑目标朝向。

**实现位置**：`hybrid_planner.cpp` 的 `optimizeAngularTarget()` 函数，lines 650-750。

#### 3.5.4 基于Hybrid A*的精细轨迹搜索

当线速度 $v_{\text{output}}$ 与目标朝向 $\theta_{\text{goal}}^{\text{smooth}}$ 确定后，为进一步优化短期（0.3-1.0 秒）内的运动轨迹，HLP在更细粒度上执行**离散Hybrid A\*搜索**。

**状态空间离散化**：

在机器人周围的局部包围盒（通常 $\pm 2$ 米）内，对连续状态 $(x, y, \theta)$ 进行栅格化：

$$x \mapsto i_x \in [0, N_x), \quad y \mapsto i_y \in [0, N_y), \quad \theta \mapsto i_\theta \in [0, N_\theta)$$

其中 $N_x, N_y$ 通常为 40-80（对应 0.05m 栅格分辨率），$N_\theta$ 通常为 16-32（对应 22.5°-11.25° 的角度分辨率）。

**状态转移与代价计算**：

从当前离散状态 $(i_x, i_y, i_\theta)$ 扩展时，考虑在固定时间步长 $\Delta t$（通常 0.1 秒）内的所有可能运动。设在该时间步内选择角加速度 $a$，使得角速度从 $\omega$ 变为 $\omega' = \omega + a\Delta t$，其中 $a$ 被约束在 $[-a_\omega, a_\omega]$ 范围内。对应的角度变化与位置推进为：

$$\Delta\theta = \omega \Delta t + \tfrac12 a (\Delta t)^2, \quad x' = x + v_{\text{output}} \Delta t \cos(\theta + \Delta\theta/2), \quad y' = y + v_{\text{output}} \Delta t \sin(\theta + \Delta\theta/2)$$

（位置更新采用中点公式以提高精度）

对每个后继状态 $(x', y', \theta')$ 计算三部分代价：

1. **障碍代价** $C_{\text{obs}}(x', y')$：通过调用LBFGS优化的代价函数（见第2节）获得当前点的障碍惩罚。若该值为致命（进入膨胀障碍），则丢弃该后继节点。

2. **边代价** $C_{\text{edge}}$：综合距离与转角的惩罚：
$$C_{\text{edge}} = \sqrt{(x'-x)^2 + (y'-y)^2} + 0.5 |\Delta\theta|$$

3. **启发式代价** $h$：估计从后继状态到目标朝向的最小成本，以加速搜索：
$$h = \sqrt{(x'-x_{\text{goal}})^2 + (y'-y_{\text{goal}})^2} + 0.1 \min_{\text{ang}}|\theta' - \theta_{\text{goal}}^{\text{smooth}}|$$

其中 $\min_{\text{ang}}$ 表示角度差的最小表示（考虑周期性）。

**Dijkstra/A\*搜索**：

以 $f = g + C_{\text{obs}}(x', y') + h$ 为优先级，在优先队列中排序所有候选状态。从初始状态开始，重复提取最小优先级的节点，扩展其后继，直到达到终止条件（通常是达到预设的最大迭代次数或覆盖完整时间范围）。

返回搜索得到的最优路径序列，作为向MPC控制器传递的参考轨迹。

**实现位置**：`hybrid_planner.cpp` 的 `planHybridAstar()` 函数，lines 400-550。

---
其中 $g$ 为从起点累计的扩展代价。

3) **角速度提取与参数化**：在得到目标节点（或时限内启发式最优节点）后，从回溯路径中提取起始段的朝向变化 $\Delta\theta_0$，并以
$$T_{\text{astar}}=\frac{D}{v}$$
作为该段的时间尺度，角速度取
$$\omega = 1.2\cdot \frac{\operatorname{sgn}(\Delta\theta_0)\,|\Delta\theta_0|}{T_{\text{astar}}/2}.$$
其中符号由最短角距的方向决定，1.2 为经验放大系数，用于补偿离散化与时延带来的欠转向。

---

## 4. 社交成本：双通路架构（独立缓存社交层 + 轨迹级评估）

### 4.1 架构重构：从"主代价图混合"到"独立缓存+直接集成"

#### 4.1.1 设计理念

原设计将社交成本直接写入主代价地图（master_grid），使社交约束与真实障碍难以区分，且难以精细控制权重。新设计采用**分离架构**以实现语义清晰与灵活加权，遵循**关注点分离**（Separation of Concerns）原则：

- **社交代价**：反映行人的心理舒适距离，是一种"软约束"（可违反但有惩罚）
- **真实障碍**：静态建筑、动态障碍物的硬碰撞约束，是不可侵犯的边界

#### 4.1.2 架构对比

| 方面 | 旧设计 | 新设计 |
|------|--------|---------|
| **社交成本存储** | 写入master_grid（混合） | 独立缓存（不污染master_grid） |
| **规划器访问方式** | 从master_grid读取混合成本 | 直接查询SocialLayer缓存接口 |
| **权重调整时机** | 层计算时确定 | 轨迹评分时动态调整 |
| **语义含义** | 社交≈障碍（混淆） | 社交与真实障碍清晰分离 |
| **运行时调参** | 困难 | 通过social_scale参数灵活调整 |
| **内存使用** | 单一代价地图 | 额外缓存（约100-200 KB） |

**核心改变**：SocialLayer实现独立缓存（`write_to_master=false`），规划器通过直接访问接口 `getSocialCostAt(wx, wy)` 获取社交成本，类似于可达性地图的设计理念。

#### 4.1.3 集成点与扩展流程

在HybridPlanner的轨迹搜索与评分过程中，社交代价的集成方式为：

1. **搜索阶段**（Dijkstra可达图）：
   - 计算节点代价时，调用LBFGS优化的obstacle cost（真实障碍）
   - **不**在此阶段引入社交代价（保持搜索的速度与健壮性）
   - 结果：得到纯粹的避真实障碍的折线

2. **轨迹评分阶段**（Multi-layer evaluation）：
   - 对返回的折线，逐点查询 `SocialLayer.getSocialCostAt(x, y)`
   - 计算轨迹的综合社交成本
   - **运行时**根据 `social_scale` 参数调整权重，融合多个代价维度

3. **动态权重融合**：
   - 若 `social_scale = 0.0`，社交代价被完全忽视
   - 若 `social_scale = 1.0`，社交代价权重与其他维度等同
   - 支持中间值进行平衡调整

### 4.2 代价地图社交层（SocialLayer - 空间社交势场）

#### 4.2.1 行人速度与航向的提取

社交舒适距离的建模基于行人的运动意图。对任意行人 $j$，其位置为 $(x_j, y_j)$，速度向量为 $(v_{xj}, v_{yj})$。定义其航向（行进方向）为：

$$\theta_{h,j} = \begin{cases}
\text{atan2}(v_{yj}, v_{xj}) & \text{if } \sqrt{v_{xj}^2 + v_{yj}^2} > \varepsilon \\
\text{previous } \theta_{h,j} & \text{otherwise（静止或低速时保持上一时刻航向）}
\end{cases}$$

其中 $\varepsilon = 1e-3 m/s$ m/s 是静止判定阈值。航向确定后，相对位移向量被旋转到行人的**局部坐标系**（前向为 $x$ 轴，左侧为 $y$ 轴）：

$$\begin{bmatrix}
x_r \\
y_r
\end{bmatrix} = \begin{bmatrix}
\cos\theta_{h,j} & \sin\theta_{h,j} \\
-\sin\theta_{h,j} & \cos\theta_{h,j}
\end{bmatrix} \begin{bmatrix}
\Delta x \\
\Delta y
\end{bmatrix}, \quad \text{其中} \quad \Delta x = x - x_j, \quad \Delta y = y - y_j$$

#### 4.2.2 分段各向异性舒适距离模型

心理学研究表明，人类周围的个人空间（personal space）具有明显的方向性：**前方距离更近、更能接受；后方或侧后方则倾向于保持更远距离**。基于这一观察，采用"前方椭圆 + 后方圆形"的分段建模方式，在截断距离 $R_c = 3.0$ 米内：

**情况1：行人前方**（$x_r \ge 0$）

在行人前方采用**椭圆舒适区**，其中相对位置 $(x_r, y_r)$ 满足椭圆指标：

$$e = \frac{x_r^2}{a^2} + \frac{y_r^2}{b^2}$$

- 若 $e \le 1$，则点在椭圆内（舒适区），社交代价为：
$$C_{\text{social},j}(x, y) = (1 - e) \cdot C_{\max}$$

- 若 $e > 1$，则点在椭圆外，社交代价为 $0$（无约束）

**椭圆参数的物理含义**：
- $a = 2.5$ 米：前方椭圆的长轴（机器人可以靠近行人2.5米）
- $b = 1.0$ 米：前方椭圆的短轴（侧方保持1.0米距离）
- 比例 $a/b = 2.5$：反映了前方更容易接受机器人靠近的特性

**情况2：行人后方**（$x_r < 0$）

在行人后方与侧后方采用**圆形舒适区**，以模型其不易被机器人接近的特性：

$$d = \sqrt{x_r^2 + y_r^2}$$

- 若 $d \le r$，社交代价为：
$$C_{\text{social},j}(x, y) = \left(1 - \frac{d}{r}\right) \cdot C_{\max}$$

- 若 $d > r$，社交代价为 $0$

**圆形参数的物理含义**：
- $r = 0.8$ 米：后方保持的舒适距离（小于前方距离，更容易引起不适）

#### 4.2.3 多行人代价的聚合

当现场有多个行人时，点 $(x, y)$ 的总社交代价为所有行人个体代价的**最大值**（最坏情况原则）：

$$C_{\text{social}}(x, y) = \max_{j=1}^{N_{\text{ped}}} C_{\text{social},j}(x, y)$$

这确保了轨迹必须避开所有行人的舒适区，而不会被多个较小代价"相消"。

#### 4.2.4 代价图积分与缓存管理

每个控制周期（通常 10 Hz，即 100 ms），SocialLayer执行以下步骤：

1. **行人数据获取**：从行人跟踪模块 `pedestrian_tracker` 订阅行人状态（位置、速度）
2. **增量更新缓存**：
   - 对每个代价地图栅格 $(i_x, i_y)$，转换为世界坐标 $(w_x, w_y)$
   - 遍历所有行人，计算相对位置与舒适指标
   - 累计更新该栅格的社交代价
3. **接口暴露**：实现 `getSocialCostAt(wx, wy)` 方法，支持双线性插值以处理非整数坐标的查询
4. **性能优化**：
   - 仅在行人移动超过阈值（0.2 米）时触发重新计算
   - 利用局部包围盒（每个行人周围3×3米）限制更新范围，避免全图扫描

**实现位置**：`src/core/local_planner/social_layer/src/social_layer.cpp`，lines 150-300。

### 4.3 轨迹级社交成本评估（SocialCostFunction - 规划阶段直接集成）

#### 4.3.1 评估流程与设计理念

在轨迹搜索与评分阶段，SocialCostFunction实现一套**独立的评估维度**，其目标是量化候选轨迹对行人的"骚扰程度"，而非评估碰撞风险（碰撞由DynamicObstacleCostFunction负责）。

对候选轨迹 $\tau = \{(x_i, y_i)\}_{i=0}^{N}$ 的离散点集，逐点查询社交层的缓存接口：

$$C_{\text{social}}(i) = \text{SocialLayer.getSocialCostAt}(x_i, y_i), \quad i = 0, 1, \ldots, N$$

#### 4.3.2 轨迹综合社交成本

对轨迹上所有点的社交代价，采取**最坏情况（Worst-case）原则**：轨迹的综合社交成本为其上最不舒适点的代价：

$$J_{\text{social}}(\tau) = w_{\text{social}} \cdot \max_{i=0,\ldots,N} C_{\text{social}}(i)$$

其中 $w_{\text{social}}$ 是运行时可调的权重系数。

**设计意义**：最坏情况原则保证了即使轨迹的大部分区域都避开了行人舒适区，但如果某一点深入行人的个人空间，轨迹整体就被评价为"不舒适"。这鼓励规划器生成**对称地尊重所有行人**的轨迹。

#### 4.3.3 权重的运行时调整

社交权重 $w_{\text{social}}$ 通过参数 `social_scale` 动态控制，无需重新编译：

- **$w_{\text{social}} = 0.0$**：完全忽视社交约束，规划器只考虑安全性与效率
  - 适用于无行人环境或紧急避碰模式
  
- **$w_{\text{social}} = 0.5$**：适中权重，在安全与社交之间平衡
  - 默认配置，适合日常导航

- **$w_{\text{social}} = 1.0$**：等权重，社交约束与其他评估维度同等重要
  - 适用于人员密集场景，需要更多地尊重行人舒适

- **$w_{\text{social}} > 1.0$**：强制社交考量，即使增加轨迹长度也要避开行人
  - 适用于特殊场景（如养老院、儿童活动中心）

**实现位置**：`src/core/local_planner/hlpmpccorridor_local_planner/src/social_cost_function.cpp`，lines 80-150。

### 4.4 两条行人处理通路的互补设计

#### 4.4.1 通路概述

HybridPlanner中对行人的处理采用**两条平行的独立通路**，各自有明确的目标与评估方式，形成多层防护体系：

**通路A - 动态行人碰撞风险（DynamicObstacleCostFunction）**
- **目标**：确保与行人的**碰撞安全性**（硬约束，必须满足）
- **预测基础**：常速度模型，假设行人短期内保持匀速直线运动
- **评估方式**：时空距离度量，在预测行人轨迹上评估轨迹安全裕度
- **代价类型**：指数衰减惩罚，距离越近惩罚越大
- **失效设计**：若轨迹与行人预测轨迹重叠超过安全距离，标记为不可行

**通路B - 社交舒适度（SocialCostFunction）**
- **目标**：评估对行人的**骚扰程度**（软约束，可违反但有惩罚）
- **评估基础**：行人的实时位置与航向，无需预测（仅当前时刻）
- **评估方式**：空间舒适距离，评估轨迹是否侵入个人空间
- **代价类型**：线性衰减（椭圆/圆形舒适模型）
- **失效设计**：可侵入个人空间，但需要付出较高的轨迹成本

#### 4.4.2 通路A详解：动态行人碰撞风险

**行人运动预测**：

对行人 $j$ 的位置 $(x_j^{obs}, y_j^{obs})$ 与速度 $(v_{xj}, v_{yj})$，在轨迹时间步 $i$ 对应的时刻 $t_i = i \cdot \Delta t$ 处，预测其位置为：

$$\hat{\mathbf{p}}_j(t_i) = \begin{bmatrix}
\hat{x}_j(t_i) \\
\hat{y}_j(t_i)
\end{bmatrix} = \begin{bmatrix}
x_j^{obs} + v_{xj} \cdot t_i \\
y_j^{obs} + v_{yj} \cdot t_i
\end{bmatrix}$$

**安全裕度计算**：

机器人轨迹点 $(x_i, y_i)$ 与预测的行人位置 $\hat{\mathbf{p}}_j(t_i)$ 之间的距离为：

$$d_{ij} = \sqrt{(x_i - \hat{x}_j(t_i))^2 + (y_i - \hat{y}_j(t_i))^2}$$

定义**安全裕度**为：

$$m_{ij} = d_{ij} - (r_r + d_s)$$

其中：
- $r_r = 0.3$ 米：机器人的碰撞半径（机器人外轮廓的最小包围圆半径）
- $d_s = 0.3$ 米：安全距离冗余（缓冲空间，应对预测误差）
- $r_r + d_s = 0.6$ 米：总的安全圆半径

**碰撞代价**：

采用**指数衰减函数**来惩罚安全裕度不足的情况：

$$C_{ij} = \begin{cases}
\exp\left(-\frac{m_{ij}}{m_{\max}}\right) & \text{if } m_{ij} < m_{\max} \\
0 & \text{if } m_{ij} \ge m_{\max}
\end{cases}$$

其中 $m_{\max} = 0.5$ 米是衰减时间常数（安全裕度为0.5m时，惩罚降至 $\exp(-1) \approx 0.37$）。

轨迹的综合动态碰撞代价为：

$$J_{\text{dyn}}(\tau) = w_{\text{dyn}} \cdot \max_{i, j} C_{ij}$$

**可行性判定**：

若 $J_{\text{dyn}}(\tau) > 1.0$（即存在轨迹点与行人预测位置的距离小于 $r_r + d_s$ 的距离超过某个阈值），则该轨迹标记为**不可行**，不在后续评分中考虑。

#### 4.4.3 通路互补的实践意义

两条通路的分离设计带来以下好处：

1. **安全性与舒适性分离**：
   - 碰撞是绝对禁止（硬约束），不存在权衡空间
   - 舒适度是相对评估（软约束），可根据场景调整权重

2. **鲁棒性提升**：
   - 碰撞检测基于外推的速度模型，具有短期预测准确性
   - 舒适度评估仅基于当前状态，不依赖预测，更鲁棒

3. **实时性保证**：
   - 碰撞检测计算量小（距离计算），快速判决
   - 舒适度评估为可选评分维度，可在时间紧张时跳过，保证决策及时性

4. **参数独立调整**：
   - 碰撞距离（$r_r, d_s$）由硬件与安全规范决定，很少改变
   - 社交权重（$w_{\text{social}}$）可灵活调整，适应不同场景与用户偏好

---

**通路B - 社交舒适度（SocialCostFunction）**
- **目标**：确保**乘坐体验与社交接受度**（软约束）
- **模型基础**：静态空间模型（椭圆+圆形），隐含地反映"行人周围的心理安全距离"
- **评估方式**：轨迹点直接查询社交层缓存，采用轨迹上最大社交成本
- **触发条件**：总是激活，通过权重系数调节影响强度（可为0）
- **代价函数**：
$$J_{\text{social}}(\tau) = w_{\text{social}}\,\max_{i=0,\ldots,N}\,C_{\text{social}}(i).$$
- **数据来源**：社交层缓存（基于行人位置、速度方向），独立计算

**并列评估的数学含义**：两条通路在critics向量中同时存在：
```cpp
critics.push_back(&dynamic_obstacle_costs_);  // 优先级8：动态碰撞风险
critics.push_back(&social_costs_);            // 优先级9：社交舒适度
```

**互补性与协同效果**：
- 动态代价偏重"轨迹末端状态的最坏位置"与"时间同步下的碰撞概率"，体现**时间维的安全性**。
- 社交代价偏重"全轨迹路径上的累积舒适度"与"空间上的心理接受度"，体现**空间维的舒适性**。

**协同示例**：
- 若某条轨迹通过动态代价（距离足够，不会碰撞），但穿过行人舒适区，社交成本仍会给出惩罚，引导规划器选择"留出更多社交距离"的轨迹。
- 若另一条轨迹被动态代价判为不可行（碰撞风险），即使社交成本很低也会被拒绝。

两者形成 **"刚性安全（来自动态）+ 柔性舒适（来自社交）"** 的综合约束，体现了"先安全后舒适"的设计哲学。

---

## 5. 局部走廊（Convex Safety Corridor）的建立与使用

### 5.1 障碍物集合的提取与空间索引结构

#### 5.1.1 栅格到点集的转换

从局部代价地图（局部costmap）中提取所有**致命占据栅格**（costmap值 ≥ 254，表示静态障碍物的实心部分）。将这些栅格中心转换到世界坐标系，得到离散障碍点集：

$$\mathcal{O} = \{\mathbf{o}_1, \mathbf{o}_2, \ldots, \mathbf{o}_{N_{\text{obs}}}\} \subseteq \mathbb{R}^2$$

其中每个 $\mathbf{o}_k = (o_{k,x}, o_{k,y})$ 是一个栅格中心点。栅格分辨率通常为 0.05 m，所以点集的空间分辨率也是 0.05 m。

#### 5.1.2 KD树的构建与查询

为高效检索某个走廊段周围的相关障碍点，避免处理全局数十万个障碍点的计算开销，对 $\mathcal{O}$ 构建**KD树**（K-Dimensional Tree）数据结构。

KD树的构建时间为 $O(N_{\text{obs}} \log N_{\text{obs}})$（一次），范围查询（找出距某点在距离 $r$ 以内的所有点）的时间复杂度为 $O(\log N_{\text{obs}} + k)$，其中 $k$ 是查询结果的点数（通常远小于 $N_{\text{obs}}$）。

**查询接口**：对路径段 $\mathbf{p}_f \to \mathbf{p}_r$ 的中点 $\mathbf{c} = (\mathbf{p}_f + \mathbf{p}_r)/2$，执行半径查询：

$$\mathcal{O}_{\text{local}} = \{\mathbf{o} \in \mathcal{O} : \|\mathbf{o} - \mathbf{c}\|_2 \le r_{\text{query}}\}$$

其中 $r_{\text{query}} = 2.0$ 米是查询半径（覆盖 $\pm2$ 米范围，足以包含走廊周围的所有相关障碍）。

这样，每个走廊段只需处理 50-200 个点（而非数千个），大幅降低计算量。

### 5.2 初始多边形：动力学与几何安全边界

#### 5.2.1 安全裕度的两维分解

走廊的宽度与长度需要同时考虑机器人的运动学约束与制动性能：

**侧向安全裕度** $s_\perp$（横向距离）：

防止机器人在转弯时越过内侧边界。由于转弯过程中，机器人的实际轨迹（Dubins或Reeds-Shepp曲线）会向转弯圆心偏移，最大偏移量与最小转弯半径相关：

$$R_{\min} = \frac{v_{\max}^2}{a_{\max}}$$

其中 $v_{\max} = 0.8$ m/s，$a_{\max} = 0.5$ m/s²，得到 $R_{\min} = 1.28$ m。

侧向安全裕度设定为：
$$s_\perp = \max(s_0, 0.5 R_{\min}) = \max(0.5 \text{ m}, 0.64 \text{ m}) = 0.64 \text{ m}$$

其中 $s_0 = 0.5$ m 是基础安全距离（机器人外形到障碍物的最小间隔）。

**纵向安全裕度** $s_\parallel$（行进方向）：

防止机器人在突然制动时越过末端边界。从最大速度 $v_{\max}$ 以最大制动加速度 $a_{\text{brake}} = 0.8$ m/s² 完全制动所需的距离为：

$$d_{\text{brake}} = \frac{v_{\max}^2}{2 a_{\text{brake}}} = \frac{0.64}{1.6} = 0.4 \text{ m}$$

纵向安全裕度设定为：
$$s_\parallel = \max(s_0, d_{\text{brake}}) = \max(0.5 \text{ m}, 0.4 \text{ m}) = 0.5 \text{ m}$$

#### 5.2.2 初始矩形走廊的构造

对相邻路径点 $\mathbf{p}_f$ （前端）与 $\mathbf{p}_r$ （后端），定义单位方向向量与法向量：

$$\mathbf{d} = \frac{\mathbf{p}_r - \mathbf{p}_f}{\|\mathbf{p}_r - \mathbf{p}_f\|_2}, \quad \mathbf{n} = \begin{bmatrix} -d_y \\ d_x \end{bmatrix}$$

其中 $(d_x, d_y)$ 是方向向量的分量，$\mathbf{n}$ 是其逆时针旋转90°。

初始矩形的四个顶点为：

$$\mathbf{v}_1 = \mathbf{p}_f + s_\perp \mathbf{n} - s_\parallel \mathbf{d}$$
$$\mathbf{v}_2 = \mathbf{p}_f - s_\perp \mathbf{n} - s_\parallel \mathbf{d}$$
$$\mathbf{v}_3 = \mathbf{p}_r - s_\perp \mathbf{n} + s_\parallel \mathbf{d}$$
$$\mathbf{v}_4 = \mathbf{p}_r + s_\perp \mathbf{n} + s_\parallel \mathbf{d}$$

初始走廊定义为这四个顶点的凸包：
$$\mathcal{P}_0 = \text{conv}\{\mathbf{v}_1, \mathbf{v}_2, \mathbf{v}_3, \mathbf{v}_4\}$$

（在没有旋转与极端长宽比时，凸包就是这四个顶点构成的矩形）

**实现位置**：`src/core/local_planner/convex_safety_corridor/src/corridor_builder.cpp`，lines 200-280。

### 5.3 最大内接椭圆与凸多边形迭代收缩

#### 5.3.1 椭圆可行域的定义

在初始矩形 $\mathcal{P}_0$ 内筛出局部障碍点集：

$$\mathcal{O}_0 = \{\mathbf{o} \in \mathcal{O}_{\text{local}} : \mathbf{o} \in \mathcal{P}_0\}$$

为了找到一个**尽可能大的无障碍可行域**，采用椭圆模型。椭圆以路径段的中点为中心：

$$\mathbf{c} = \tfrac{1}{2}(\mathbf{p}_f + \mathbf{p}_r)$$

椭圆的长轴沿路径方向对齐（纵向），短轴垂直于路径（横向）。用二次型表示：

$$\mathcal{E}(\mathbf{C}, \mathbf{c}) = \left\{ \mathbf{x} \in \mathbb{R}^2 : (\mathbf{x} - \mathbf{c})^\top \mathbf{C}^{-1} (\mathbf{x} - \mathbf{c}) \le 1 \right\}$$

其中 $\mathbf{C} = \text{diag}(c_x^2, c_y^2)$ 是椭圆的形状矩阵（主对角线元素为长短轴的平方），$c_x$ 为纵轴半径，$c_y$ 为横轴半径。

初始设置 $c_x = \|\mathbf{p}_r - \mathbf{p}_f\|_2 / 2 + 0.5$ m（路径段长度的一半加安全距离），$c_y = s_\perp = 0.64$ m（侧向安全裕度）。

#### 5.3.2 迭代收缩算法

**步骤1**：检查椭圆内是否有障碍点。计算椭圆内离椭圆边界最近的障碍点 $\mathbf{o}^* \in \mathcal{O}_0$：

$$\mathbf{o}^* = \arg\min_{\mathbf{o} \in \mathcal{O}_0} \min_{\mathbf{p} \in \partial\mathcal{E}} \|\mathbf{p} - \mathbf{o}\|_2$$

若 $\mathcal{O}_0$ 为空（无障碍点在椭圆内），则终止，返回当前椭圆 $\mathcal{E}$。

**步骤2**：找椭圆边界上离 $\mathbf{o}^*$ 最近的点 $\mathbf{p}^*$：

在椭圆边界上距障碍点最近的点处，椭圆的法向量（与梯度方向一致）为：

$$\mathbf{n}_{\mathbf{p}^*} = \frac{\mathbf{C}^{-1}(\mathbf{p}^* - \mathbf{c})}{\|\mathbf{C}^{-1}(\mathbf{p}^* - \mathbf{c})\|_2}$$

**步骤3**：构造切平面约束。在 $\mathbf{p}^*$ 处的切平面对应的外法向约束为：

$$\mathbf{n}_{\mathbf{p}^*}^\top (\mathbf{x} - \mathbf{p}^*) \le 0$$

这个约束定义了一个半空间，确保所有满足该约束的点都在椭圆外侧（或边界上）。

**步骤4**：收缩椭圆。通过减小相应方向的半轴长度（在该法向上的投影），使得新椭圆不再包含 $\mathbf{o}^*$：

$$\mathbf{C}_{\text{new}} = \mathbf{C} - \delta \mathbf{n}_{\mathbf{p}^*} \mathbf{n}_{\mathbf{p}^*}^\top$$

其中 $\delta > 0$ 是收缩步长（通常 0.01）。更新椭圆：$\mathcal{E} \leftarrow \mathcal{E}(\mathbf{C}_{\text{new}}, \mathbf{c})$。

**步骤5**：重复。回到步骤1，直至 $\mathcal{E}$ 内无障碍点或达到最大迭代次数（通常50次）。

#### 5.3.3 多边形走廊的最终定义

一旦获得了无障碍椭圆 $\mathcal{E}$，收集整个迭代过程中累积的所有切平面约束 $\{\mathbf{n}_i^\top (\mathbf{x} - \mathbf{p}_i^*) \le 0\}$，加上初始矩形 $\mathcal{P}_0$ 的四个边界平面约束，形成半空间集合 $\{\mathcal{H}_m\}$。

走廊段定义为这些半空间的交集对应的凸多边形：

$$\mathcal{P} = \bigcap_m \mathcal{H}_m$$

可通过凸包计算或半空间求交算法（如incremental convex hull）获得 $\mathcal{P}$ 的顶点表示。

对整条（局部）参考折线，以路径相邻点为单位逐段处理，得到凸多边形序列 $\{\mathcal{P}_0, \mathcal{P}_1, \ldots, \mathcal{P}_{K-1}\}$。

**实现位置**：`src/core/local_planner/convex_safety_corridor/src/corridor_builder.cpp`，lines 280-450（迭代收缩）与lines 450-550（多边形求交）。

### 5.4 与局部规划的耦合方式

#### 5.4.1 末端一致性检验

走廊在局部规划（HybridPlanner）中以**末端一致性检验**的方式参与约束：

在从当前位置出发执行Hybrid A*搜索，并生成一条候选轨迹 $\tau = \{(x_i, y_i)\}_{i=0}^{N}$ 后，检查其末端点 $\mathbf{p}_{\text{end}} = (x_N, y_N)$ 是否落在某一走廊多边形内：

$$\exists k \in [0, K-1] \text{ s.t. } \mathbf{p}_{\text{end}} \in \mathcal{P}_k$$

若条件满足，则该轨迹被判为**可行**；否则标记为**不可行**，排除出最终输出。

#### 5.4.2 约束的物理含义

走廊约束通过末端点的方式隐含地限制了整条轨迹的形状与可达范围：

- **长期一致性**：确保规划器不会"偏离轨道"，始终朝向当前目标段推进
- **障碍物避碰**：走廊的边界由最大内接椭圆与障碍物约束共同确定，确保轨迹末端的可达性
- **鲁棒性**：即使轨迹在中间某些地方可能临近走廊边界（通过动态代价函数的平衡），末端的约束保证了整体的安全性

#### 5.4.3 计算代价

- **离线阶段**（每10-50 ms更新一次）：KD树查询（$O(\log N_{\text{obs}})$）+ 椭圆迭代收缩（通常 20-50 次迭代，每次 $O(1)$）+ 半空间求交（$O(M \log M)$，$M$ 为约束数，通常 10-30）
  - 总时间：5-30 ms（高效，可在规划频率内完成）

- **在线阶段**（每条候选轨迹）：点-多边形包含检验 $O(K)$（$K$ 为多边形边数，通常 6-12）
  - 总时间：< 0.1 ms（可忽略）

---

## 6. MPC控制与前瞻点跟踪备选

### 6.1 模型预测控制（MPC）的基本框架

#### 6.1.1 离散时间双积分模型

MPC基于机器人运动的**双积分（Double Integrator）简化模型**：

$$\begin{aligned}
x_{k+1} &= x_k + v_{x,k} \Delta T \\
y_{k+1} &= y_k + v_{y,k} \Delta T \\
v_{x,k+1} &= v_{x,k} + a_{x,k} \Delta T \\
v_{y,k+1} &= v_{y,k} + a_{y,k} \Delta T
\end{aligned}$$

其中 $(x_k, y_k)$ 是位置，$(v_{x,k}, v_{y,k})$ 是速度，$(a_{x,k}, a_{y,k})$ 是加速度（控制输入），$\Delta T = 0.1$ s 是离散化时间步。

状态向量为 $\mathbf{x}_k = [x_k, y_k, v_{x,k}, v_{y,k}]^\top \in \mathbb{R}^4$，控制向量为 $\mathbf{u}_k = [a_{x,k}, a_{y,k}]^\top \in \mathbb{R}^2$。

#### 6.1.2 参考轨迹与跟踪代价

参考轨迹由HLP生成的参考折线及其分配的速度给出：$\{\mathbf{x}_k^{\text{ref}}\}$。MPC的目标是最小化预测时域内的**跟踪误差**：

$$J_{\text{MPC}} = \sum_{i=0}^{H-1} \left( \|\mathbf{x}_{k+i|k} - \mathbf{x}_{k+i}^{\text{ref}}\|_Q^2 + \|\mathbf{u}_{k+i|k}\|_R^2 \right) + \|\mathbf{x}_{k+H|k} - \mathbf{x}_{k+H}^{\text{ref}}\|_P^2$$

其中：
- $H = 10$ 是预测时域步数（对应 1.0 秒的预视）
- $\mathbf{x}_{k+i|k}$ 是在时刻 $k$ 对时间 $k+i$ 的预测状态
- $\mathbf{u}_{k+i|k}$ 是在时刻 $k$ 对时间 $k+i$ 的预测控制输入
- $\|\cdot\|_Q^2 = (\cdot)^\top Q (\cdot)$ 是带权范数，$Q$ 为状态权重矩阵，$R$ 为控制权重矩阵，$P$ 为终端状态权重矩阵

**权重矩阵设置**：

$$Q = \text{diag}(10, 10, 5, 5), \quad R = \text{diag}(1, 1), \quad P = 2Q$$

其含义是：
- 位置误差（$x, y$ 分量）的权重为 10，强制靠近参考位置
- 速度误差的权重为 5，次优先级
- 控制输入的权重为 1，防止控制饱和

#### 6.1.3 约束条件

MPC的优化需要满足若干约束：

**物理约束**（由机器人硬件决定）：
$$|a_x|, |a_y| \le a_{\max} = 0.5 \text{ m/s}^2$$
$$\sqrt{v_x^2 + v_y^2} \le v_{\max} = 0.8 \text{ m/s}$$

**安全约束**（由环境与规划决定）：
$$\text{DistanceToObstacle}(\mathbf{x}_{k+i|k}) \ge d_{\text{safe}} = 0.3 \text{ m}, \quad i = 0, 1, \ldots, H$$

即预测轨迹上每一点到最近障碍物的距离不小于 0.3 m。

### 6.2 前瞻点纯跟踪控制器（鲁棒备选）

#### 6.2.1 前瞻点的选择

虽然MPC在正常情况下性能更好，但在某些边界情况（如参考轨迹本身不可行、QP求解失败）下，采用**前瞻点纯跟踪**（Pure Pursuit）作为鲁棒备选策略。

在HLP生成的参考折线上，按**弧长累积**选取前瞻距离 $L_d$ 对应的目标点 $\mathbf{p}_\ell$：

$$L_d = \max(v \cdot T_d, L_{d,\min})$$

其中：
- $v$ 是当前机器人速度
- $T_d = 1.0$ s 是前瞻时间（使得前瞻距离约为当前速度的1秒行进距离）
- $L_{d,\min} = 0.5$ m 是最小前瞻距离（防止目标点过近导致转向过急）

沿折线逐段累计距离，找到第一个使得累计距离 $\ge L_d$ 的折线点，该点（或其插值）即为 $\mathbf{p}_\ell$。

#### 6.2.2 控制律

给定机器人当前位置 $(x, y)$ 与朝向 $\theta$，前瞻点为 $\mathbf{p}_\ell = (x_\ell, y_\ell)$，计算以下控制输入：

**期望航向**：
$$\theta_d = \text{atan2}(y_\ell - y, x_\ell - x)$$

**航向误差**（考虑角度周期性）：
$$e_\theta = \text{wrap}(\theta_d - \theta)$$

其中 `wrap` 函数将角度差限制在 $[-\pi, \pi)$ 范围内。

**控制输出**（带饱和）：

$$v_{\text{cmd}} = \min(k_v \|\mathbf{p}_\ell - (x, y)\|_2 \cdot \cos(e_\theta), v_{\max})$$
$$\omega_{\text{cmd}} = \text{sat}(k_\omega e_\theta, -\omega_{\max}, \omega_{\max})$$

其中：
- $k_v = 0.2$ m/(s·m) 是速度增益（距离前瞻点越远，速度越高）
- $k_\omega = 1.0$ rad/(s·rad) 是角速度增益
- 因子 $\cos(e_\theta)$ 在航向误差很大时会大幅降低速度（防止高转向速度下的离轨）
- `sat` 函数将值裁剪到 $[-\omega_{\max}, \omega_{\max}]$ 范围

#### 6.2.3 鲁棒性特点

相比MPC，前瞻点纯跟踪具有以下优点：

1. **快速计算**：无需求解优化问题，时间复杂度为 $O(\text{num\_segments})$（仅 1-5 ms）
2. **鲁棒性**：不依赖精确的动力学模型或参考轨迹精度，只需参考折线的基本形状
3. **回退保证**：即使MPC收敛失败或参考轨迹不可行，前瞻点跟踪仍能输出安全的控制命令
4. **参数少**：仅需调整 $k_v, k_\omega, L_d$ 三个参数

**实现位置**：`src/core/controller/hlpmpccorridor_local_planner/src/pure_pursuit_controller.cpp`，lines 100-200。

### 6.3 MPC与前瞻点的权重融合

#### 6.3.1 融合策略

在实际执行中，采用**软开关（Soft Switch）**策略在MPC与前瞻点之间动态调整权重，而不是硬切换：

$$\mathbf{u}_{\text{final}} = \alpha(s) \mathbf{u}_{\text{MPC}} + (1 - \alpha(s)) \mathbf{u}_{\text{PP}}$$

其中 $\alpha(s) \in [0, 1]$ 是融合权重，$s$ 是MPC求解器的健康状态指标。

#### 6.3.2 权重 $\alpha(s)$ 的自适应计算

$$\alpha(s) = \begin{cases}
1.0 & \text{if } s_{\text{converge}} = \text{true AND } s_{\text{feasible}} = \text{true} \\
0.5 & \text{if } s_{\text{converge}} = \text{true AND } s_{\text{feasible}} = \text{false} \\
0.3 & \text{if } s_{\text{converge}} = \text{false} \\
0.0 & \text{if } \|\mathbf{u}_{\text{MPC}}\|_2 > a_{\max} \text{ (输出超出物理限制)}
\end{cases}$$

**状态说明**：
- $s_{\text{converge}}$：QP求解器是否在指定迭代数内收敛（通常 100 次迭代）
- $s_{\text{feasible}}$：预测轨迹是否满足所有约束（安全、动力学）

#### 6.3.3 融合的物理意义

- **$\alpha = 1.0$**（纯MPC）：最优性能，最精细的轨迹跟踪，适用于正常规划导航
- **$\alpha = 0.5$**（MPC + PP等权）：在MPC可能输出不可行指令时，混合PP的鲁棒性
- **$\alpha = 0.3$**（主要PP）：MPC求解困难时，以PP为主保证鲁棒执行
- **$\alpha = 0.0$**（纯PP）：MPC输出物理上不可行时，纯粹依赖前瞻点作为安全保障

**实现位置**：`src/core/controller/hlpmpccorridor_local_planner/src/hybrid_planner.cpp`，lines 1100-1200。

---### 6.2 二次规划型MPC（控制序列整形）
为在保持实时性的同时提升控制平滑性，采用对控制序列的二次规划：在预测长度 $N$ 上优化
$$\mathbf{z}=[v_0,\ldots,v_{N-1},\ \omega_0,\ldots,\omega_{N-1}]^\top.$$
参考控制 $\mathbf{u}_{\text{ref}}=[v_{\text{ref}},\omega_{\text{ref}}]^\top$ 由前瞻点跟踪器生成。

目标函数由参考跟踪与相邻步控制增量惩罚组成：
$$\min_{\mathbf{z}}\ \sum_{k=0}^{N-1}\Big(q_v(v_k-v_{\text{ref}})^2+q_\omega(\omega_k-\omega_{\text{ref}})^2\Big)
+\sum_{k=1}^{N-1}\Big(r_v(v_k-v_{k-1})^2+r_\omega(\omega_k-\omega_{k-1})^2\Big).$$
约束为逐步的速度上下界：
$$v_k\in[v_{\min},v_{\max}],\quad \omega_k\in[-\omega_{\max},\omega_{\max}],\quad k=0,\ldots,N-1.$$
求解后仅执行第一步控制 $(v_0,\omega_0)$。值得强调的是，该二次规划问题仅包含逐步速度上下界约束，不显式引入状态方程；参考项由前瞻点跟踪器提供，二次规划主要用于对控制序列进行平滑整形，因此计算规模小，适于实时求解。

---

## 7. 混合平滑策略与事件触发重规划

### 7.1 HLP、MPC与前瞻点的连续权重融合

#### 7.1.1 融合的必要性与原理

HLP输出的参考轨迹 $(v_{\text{hlp}}, \omega_{\text{hlp}})$ 是基于分层可达图与Dijkstra搜索得到的，具有**反应快速、考虑运动学约束**的特点，但可能不够平滑（多个相邻点的控制输入可能变化较大）。

MPC基于二次规划求解，能生成**平滑的控制序列**，但计算量较大，且在MPC的优化问题无可行解时可能失败。

前瞻点纯跟踪则是一种**鲁棒备选**，计算快速且不依赖优化求解，但跟踪精度不如MPC。

为了结合三者的优点，采用**多层次、多优先级**的控制融合策略，按照优先级顺序尝试：

1. **首选**：MPC（最优性能）
2. **备选1**：前瞻点纯跟踪（MPC失败时）
3. **备选2**：HLP直接输出（前瞻点亦失败时）

#### 7.1.2 基于MPC可行性的动态权重计算

定义MPC求解的**健康状态指标** $s_{\text{MPC}} \in \{0, 1\}$：

$$s_{\text{MPC}} = \begin{cases}
1 & \text{if QP求解收敛 AND 预测轨迹可行（满足所有约束）} \\
0 & \text{otherwise}
\end{cases}$$

**可行性检验细节**：
- 检查预测时域内所有轨迹点是否满足安全约束（距离障碍物 ≥ 0.3 m）
- 检查所有状态与控制输入是否在物理限制范围内
- 检查轨迹末端是否落在安全走廊内（若启用走廊约束）

基于 $s_{\text{MPC}}$ 定义融合权重 $\alpha \in [0, 1]$：

$$\alpha = \begin{cases}
1.0 & \text{if } s_{\text{MPC}} = 1 \text{ AND 连续3个周期收敛（防抖）} \\
0.7 & \text{if } s_{\text{MPC}} = 1 \text{ BUT 首次收敛或周期性失败} \\
0.4 & \text{if } s_{\text{MPC}} = 0 \text{ AND 上一周期MPC可行} \\
0.0 & \text{if } s_{\text{MPC}} = 0 \text{ AND 连续3个周期失败}
\end{cases}$$

**前瞻点控制器的启用条件**：
$$\text{use\_PP} = \begin{cases}
\text{true} & \text{if } \alpha < 0.7 \\
\text{false} & \text{if } \alpha \ge 0.7
\end{cases}$$

#### 7.1.3 控制输出的加权组合

最终的控制命令为：

$$\mathbf{u}_{\text{final}} = \begin{cases}
\alpha \mathbf{u}_{\text{MPC}} + (1-\alpha) \mathbf{u}_{\text{PP}} & \text{if } s_{\text{MPC}} \le 0.7 \\
\mathbf{u}_{\text{MPC}} & \text{if } s_{\text{MPC}} > 0.7 \\
\mathbf{u}_{\text{PP}} & \text{if } s_{\text{MPC}} = 0 \text{ AND PP可行} \\
\mathbf{u}_{\text{HLP}} & \text{otherwise（应急备选）}
\end{cases}$$

其中：
- $\mathbf{u}_{\text{MPC}} = [v_{\text{MPC}}, \omega_{\text{MPC}}]^\top$：二次规划的最优控制
- $\mathbf{u}_{\text{PP}} = [v_{\text{PP}}, \omega_{\text{PP}}]^\top$：前瞻点跟踪的控制
- $\mathbf{u}_{\text{HLP}} = [v_{\text{HLP}}, \omega_{\text{HLP}}]^\top$：HLP层的默认输出

**加权融合的物理含义**：
- 当 $\alpha = 1.0$ 时，完全信任MPC的平滑性能，使用其优化的控制序列
- 当 $0 < \alpha < 1.0$ 时，线性组合MPC与PP，逐步降低对MPC的依赖
- 当 $\alpha = 0.0$ 时，完全切换到前瞻点纯跟踪，确保鲁棒性

### 7.2 低通滤波与加速度连续性约束

#### 7.2.1 权重的平滑化处理

直接使用上述阶梯型权重会导致MPC与前瞻点之间的**权重抖动**（α在相邻周期间变化），进而引起控制输出的不连续跳变，对执行层与传感器造成冲击。

为平滑权重变化，对 $\alpha$ 应用**一阶低通滤波**（惯性滤波）：

$$\alpha_{\text{smooth}}(t) = \alpha_{\text{smooth}}(t-\Delta T) + \frac{\Delta T}{\tau_f} [\alpha(t) - \alpha_{\text{smooth}}(t-\Delta T)]$$

其中：
- $\tau_f = 0.5$ s 是滤波时间常数（权重变化从0变到1需要约 $3\tau_f = 1.5$ 秒）
- $\Delta T = 0.1$ s 是控制周期
- 滤波增益 $\frac{\Delta T}{\tau_f} = 0.2$（每个周期最多改变 0.2）

这样，即使阶梯型权重发生突变，滤波后的 $\alpha_{\text{smooth}}$ 也会平缓变化，减少控制输出的跳变。

#### 7.2.2 加速度约束下的控制饱和

最终的速度与角速度指令需要满足**动力学可实现性约束**（execution layer可行）：

$$v_{\text{output}} = \begin{cases}
\alpha_{\text{smooth}} v_{\text{MPC}} + (1-\alpha_{\text{smooth}}) v_{\text{PP}} & \text{(加速度约束前)} \\
\text{clip}(v, v_{\text{last}} - a_v \Delta T, v_{\text{last}} + a_v \Delta T) & \text{(加速度裁剪)} \\
\max(v, v_{\min}) & \text{(下界保证非零速度)}
\end{cases}$$

$$\omega_{\text{output}} = \begin{cases}
\alpha_{\text{smooth}} \omega_{\text{MPC}} + (1-\alpha_{\text{smooth}}) \omega_{\text{PP}} & \text{(角加速度约束前)} \\
\text{clip}(\omega, \omega_{\text{last}} - a_\omega \Delta T, \omega_{\text{last}} + a_\omega \Delta T) & \text{(角加速度裁剪)}
\end{cases}$$

**参数说明**：
- $a_v = 0.5$ m/s²：最大线加速度（执行层硬件限制）
- $a_\omega = 1.0$ rad/s²：最大角加速度
- $v_{\min} = 0.05$ m/s：最小线速度（防止静止）
- 裁剪的约束：$|v_{\text{output}} - v_{\text{last}}| \le 0.05$ m/s（每个100 ms周期的最大变化）

这种约束设计的好处：
1. **物理可实现**：确保生成的命令执行层能够实际执行
2. **传感器友好**：平缓的速度变化减少冲击，延长硬件寿命
3. **鲁棒性**：即使MPC与前瞻点的输出差异很大，加速度约束也能平缓过渡

**实现位置**：`src/core/controller/hlpmpccorridor_local_planner/src/hybrid_planner.cpp`，lines 1200-1300。

### 7.3 事件触发的全局重规划策略

#### 7.3.1 重规划触发条件

虽然HLP提供了实时的局部规划能力，但仍需在特定条件下触发**全局路径重新规划**（从全局出发点重新运行Hybrid A* + LBFGS），以应对环境变化或全局目标更新。

**触发条件列表**：

| 事件 | 触发条件 | 优先级 | 处理 |
|------|--------|--------|------|
| 新全局目标 | 用户指定新目标点 | 紧急（0） | 立即中断当前规划，启动新规划 |
| 地图重大更新 | 新发现障碍 / 动态障碍进入规划可行域 | 高（1） | 立即触发，保证安全 |
| 规划失败持久化 | HLP连续失败 > 0.5 s | 中（2） | 触发全局重规划寻找替代路径 |
| 参考路径陈旧 | 参考路径点距离机器人 > 阈值 | 低（3） | 周期性触发（1-2秒一次） |

#### 7.3.2 规划失败的监测

**局部规划失败判定标准**：

在一个控制周期内，若下列情况任何一项成立，则判定为HLP失败：

$$\text{HLP\_failed} = \begin{cases}
\text{true} & \text{if } J_{\text{total}} > J_{\max} \text{ (总代价超限)} \\
\text{true} & \text{if } |e_{\theta}| > \pi/4 \text{ (姿态误差超过45°)} \\
\text{true} & \text{if } d_{\text{obs}} < d_{\min} \text{ (碰撞距离不足)} \\
\text{true} & \text{if } \text{no\_feasible\_trajectory} \text{ (无可行轨迹)} \\
\text{false} & \text{otherwise}
\end{cases}$$

其中参数为：
- $J_{\max} = 100.0$：总代价上界
- $d_{\min} = 0.25$ m：最小安全距离（低于0.25 m视为碰撞风险）

**持久化失败监测**：

维护一个失败计数器 `failure_count`。每个HLP失败的周期使计数器 `+1`；每个成功周期使其 `= max(0, failure_count - 1)`（衰减）。当 `failure_count >= N_fail_thresh`（通常设为5-10，对应0.5-1.0秒）时，触发全局重规划。

#### 7.3.3 全局重规划的执行

**重规划流程**：

1. **状态保存**：保存当前的HLP生成的参考轨迹作为**临时本地导航目标**，防止重规划期间的导航空隙

2. **Hybrid A*搜索**（新的全局搜索）：
   - 起点：机器人当前位置 $(x, y, \theta)$
   - 终点：全局目标点 $\mathbf{p}_{\text{goal}}$
   - 约束：最新的环境障碍地图
   - 耗时：通常 100-500 ms（可在 10 Hz 的两个控制周期内完成）

3. **LBFGS轨迹优化**：
   - 输入：Hybrid A*返回的初始路径
   - 目标：平滑化路径，最小化障碍与平滑性代价
   - 耗时：通常 30-100 ms

4. **RDP简化**：
   - 将优化后的密集轨迹简化为稀疏的参考点
   - 为HLP提供新的全局参考

5. **平滑过渡**：
   - 新的全局参考路径不会立即替换旧路径
   - 而是通过混合权重**平缓切换**，防止HLP输出产生突跳
   - 过渡时间：通常 1.0-2.0 秒

**切换权重**：

$$\beta(t) = \begin{cases}
0 & \text{if } t < t_{\text{switch}} \\
\frac{t - t_{\text{switch}}}{T_{\text{transition}}} & \text{if } t_{\text{switch}} \le t < t_{\text{switch}} + T_{\text{transition}} \\
1 & \text{if } t \ge t_{\text{switch}} + T_{\text{transition}}
\end{cases}$$

其中 $t_{\text{switch}}$ 是开始切换的时刻，$T_{\text{transition}} = 2.0$ s 是过渡时间。

新旧路径的混合参考为：
$$\mathbf{p}_{\text{ref}}(t) = (1-\beta(t)) \mathbf{p}_{\text{old}}(t) + \beta(t) \mathbf{p}_{\text{new}}(t)$$

#### 7.3.4 重规划的性能影响

- **离线规划时间**：全局重规划需要 150-600 ms，在此期间HLP仍使用旧的参考路径导航，确保**continuous navigation**
- **频率控制**：为防止频繁重规划导致的计算负担，设置最小重规划间隔为 2.0 秒（若需要更频繁的更新可调整）
- **优先级**：地图更新与新目标指定优先级高于周期性重规划，采用中断机制立即启动

**实现位置**：`src/core/path_planner/path_planner/src/path_planner_node.cpp`，lines 450-650。

---

## 结论与讨论（面向复现的要点）

### 8.1 方法总结

本文提出的分层运动规划框架集成了**全局规划、局部搜索、轨迹优化、控制融合**四个层次，形成一个完整的从路径规划到执行控制的闭环系统：

1. **全局层**（Hybrid A* + LBFGS）：
   - 快速生成满足运动学约束的粗糙路径（计算时间 100-500 ms）
   - 通过LBFGS优化实现路径光滑化与代价降低
   - Voronoi势能指导搜索趋向于宽敞通道

2. **局部层**（HLP + 多目标融合）：
   - 分层可达图实现快速的短时参考轨迹规划（计算时间 5-20 ms）
   - 社交代价、走廊约束、动态障碍避碰形成多维约束框架
   - 自适应速度分配根据环境拥挤程度动态调节

3. **控制层**（MPC + 前瞻点 + HLP融合）：
   - MPC提供最优的平滑轨迹跟踪
   - 前瞻点提供鲁棒备选，保证MPC失败时的连续性
   - 权重融合与加速度约束保证输出连续、可实现

4. **重规划策略**（事件触发）：
   - 监测全局失败，触发全局重规划
   - 平缓切换新旧参考路径，避免控制跳变

### 8.2 关键设计特点

**社交成本的独立架构**：
- 社交层不污染主代价地图，实现了清晰的**语义分离**
- 轨迹评分时直接查询社交接口，支持**运行时权重调整**（无需重编译）
- 双通路设计（DynamicObstacle + SocialCost）区分了**安全硬约束与舒适软约束**

**凸走廊的可行域约束**：
- 通过最大内接椭圆与半空间迭代收缩，得到**数值稳定**的凸多边形
- 末端一致性判别轻量级但有效，计算开销 < 0.1 ms

**多层控制融合**：
- 三层（MPC / 前瞻点 / HLP）的优先级架构确保了**故障容错性**
- Sigmoid平滑权重与低通滤波实现了**控制连续性**
- 加速度约束保证了**执行可行性**

### 8.3 复现建议

1. **参数调优的优先级**：
   - 首先调整 `voronoi_cost_weight = 0.0`（全局Voronoi禁用）
   - 次调 `w_obs, w_smo, w_cur` (LBFGS权重)、`social_scale`（社交权重）
   - 最后微调 MPC 的 Q, R 矩阵以获得最优平衡

2. **常见失败原因与诊断**：
   - **机器人在原点打转**：voronoi_cost_weight > 0，立即改为0.0
   - **跟踪滞后**：增加HLP的权重、降低MPC的权重（减小α）
   - **频繁碰撞**：增加 `obs_dist_max`、增加 `w_obs` 权重
   - **社交约束过强**：降低 `social_scale` 参数

3. **性能调优**：
   - 全局规划频率 ≥ 1 Hz（每秒至少一次）
   - 局部规划频率 10 Hz（每 100 ms 一次）
   - MPC预测时域 10 步（1.0 秒）
   - HLP可达层级 5-8（对应 0.5-0.8 秒预视）

4. **安全检查清单**：
   - [ ] 机器人碰撞半径 (`r_r`) 正确设置为机器人实际外形
   - [ ] 安全距离 (`d_s`) 合理（通常 0.3-0.5 m）
   - [ ] 代价地图膨胀 ≥ 机器人半径 + 安全距离
   - [ ] MPC 的安全约束距离 ≤ 膨胀距离
   - [ ] 走廊约束启用且定期检查可行性

### 8.4 后续扩展方向

1. **多机器人协作规划**：扩展框架以支持多个机器人的协调避碰与任务分配
2. **动态重新配置**：在线学习用户偏好（社交距离、速度风格等），动态调整权重
3. **预测性规划**：融入行人轨迹预测，提前规避，而非被动避碰
4. **深度学习集成**：使用CNN进行代价地图生成，或使用RL优化权重参数

---