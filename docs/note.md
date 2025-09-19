# GCOPTER 全流程详解笔记（计算机专业友好版，v2）

> 目标读者：熟悉 C++/ROS，但对飞控、优化和轨迹规划背景有限的工程师。本文将 README_old.md 的描述完全落地到源码，实现“看到输出→ 找到函数 → 理解算法 → 知道如何改参数/调试”。
>
> 关联源码：
> - `gcopter/src/global_planning.cpp`
> - `gcopter/include/gcopter/gcopter.hpp`
> - `gcopter/include/gcopter/minco.hpp`
> - `gcopter/include/gcopter/sfc_gen.hpp`
> - `gcopter/include/gcopter/flatness.hpp`
> - `gcopter/include/misc/visualizer.hpp`
> - `gcopter/config/global_planning.yaml`

---

## 0. 速查表 & 名词解释

| 名称 | 解释（直观理解） | 关键代码 |
| --- | --- | --- |
| `GlobalPlanner` | ROS 节点主类，负责订阅地图/目标、调用规划、发布可视化 | `gcopter/src/global_planning.cpp:37` |
| 体素地图 (voxel map) | 将 3D 空间切成小方块储存占据信息，便于碰撞检测 | `voxel_map::VoxelMap` (`gcopter/src/global_planning.cpp:95`) |
| 膨胀 (dilation) | 给障碍物加厚，等同把无人机半径体现在地图里 | `voxelMap.dilate` (`gcopter/src/global_planning.cpp:137`) |
| SFC (Safe Flight Corridor) | 一串凸多面体，保证只要轨迹落在里面就安全 | `sfc_gen::convexCover` (`gcopter/include/gcopter/sfc_gen.hpp:100`) |
| OMPL Informed RRT* | 一种随机采样路径搜索器，起/终点确定后逼近最短路 | `sfc_gen::planPath` (`gcopter/include/gcopter/sfc_gen.hpp:40`) |
| FIRI | Fast Iterative Regional Inflation，使用障碍物表面点生成最大安全凸集 | `gcopter/include/gcopter/firi.hpp` |
| MINCO | Minimum Control effort trajectory，解析求解多项式轨迹，使 snap 能量最小 | `gcopter/include/gcopter/minco.hpp:396` |
| PVA | Position / Velocity / Acceleration (位置、一阶导、二阶导) | `iniState`, `finState` (`gcopter/src/global_planning.cpp:171`) |
| L-BFGS | 准牛顿优化算法，这里用来同时调空间（路径内点）和时间分配 | `lbfgs_optimize` (`gcopter/include/gcopter/gcopter.hpp:795`) |
| 微分平坦性 | 把姿态/推力等非线性动力学量映射到轨迹导数的数学性质 | `flatness::FlatnessMap` (`gcopter/include/gcopter/flatness.hpp:24`) |
| 罚函数 (penalty) | 将“约束违反量”变成 cost，加大权重就越不容易违反 | `attachPenaltyFunctional` (`gcopter/include/gcopter/gcopter.hpp:340`) |

### 文件与数据流关系图

```
ROS Topic ──► GlobalPlanner ──► VoxelMap ──► SFC生成 ──► GCOPTER_PolytopeSFC ──► MINCO
                    │               │              │                              │
                    │               │              └── Visualizer ◄─────────────┘
                    │               │
                    └── targetCallBack  ─────────────── plan() ───────────────► Trajectory<5>
```

---

## 1. ROS 接口概览

| Topic/Param | 用途 | 发布/订阅者 |
| --- | --- | --- |
| `/voxel_map` (`sensor_msgs/PointCloud2`) | 静态地图点云（通常来自离线地图） | `mapSub` (`GlobalPlanner::mapCallBack`) |
| `/move_base_simple/goal` (`geometry_msgs/PoseStamped`) | RViz 的 2D Nav Goal 指令 | `targetSub` (`GlobalPlanner::targetCallBack`) |
| `/visualizer/*` | 可视化 Marker/浮点量，用于 RViz 和 rqt_plot | `Visualizer` | 
| YAML params (`global_planning.yaml`) | 地图范围、约束阈值、物理参数 | `Config` 构造函数读取 | 

`global_planning.launch` 中会同时启动：
- `global_planning_node`
- RViz（加载 `config/global_planning.rviz`）
- rqt_plot（预设监控推力/速度等话题）

---

## 2. 起点终点选择：从鼠标点击到 3D 坐标

源码：`GlobalPlanner::targetCallBack` (`gcopter/src/global_planning.cpp:232-256`)

1. **过滤**：若地图还没初始化（`mapInitialized == false`），直接忽略。
2. **保持 2 个点**：`startGoal` 容量到 2 时 `clear()`，保证 `[起点, 终点]` 两个元素。
3. **高度编码**：
   ```cpp
   const double zGoal = mapBound_low_z + radius + fabs(msg->pose.orientation.z)
                        * (mapBound_high_z - mapBound_low_z - 2 * radius);
   ```
   - `mapBound_low/high_z` = `MapBound[4]/MapBound[5]`
   - `radius` = `DilateRadius`
   - `orientation.z = sin(yaw/2)`，来源于 2D Nav Goal
4. **可行性检查**：`voxelMap.query(goal) == 0` 表示该位置未被膨胀障碍占据。
5. **可视化**：调用 `visualizer.visualizeStartGoal(goal, 0.5, index)` 在 RViz 标记起终点。
6. **触发规划**：每次成功放入点后调用 `plan()`。

> 对于只懂图形/导航的读者，可以把这段理解为：用鼠标在平面选点，系统自动根据箭头朝向计算高度，再检查地图是否允许，将其压入数组，凑满 “起点+终点” 就开始算路径。

---

## 3. 地图加载与膨胀：`mapCallBack`

源码：`gcopter/src/global_planning.cpp:103-141`

伪代码：
```cpp
if (!mapInitialized) {
    for 每个点云点 p:
        if p 有效：voxelMap.setOccupied(p);
    voxelMap.dilate(ceil(DilateRadius / voxelWidth));
    mapInitialized = true;
}
```

- `voxelMap` 尺寸由 `MapBound` 和 `VoxelWidth` 决定（`Config` 构造函数中计算）。
- `dilate(k)` 在立方体邻域拓宽障碍，常用 BFS/形态学方式实现。
- `Config::DilateRadius` 等同无人的半径或期望 buffer，后续 FIRI/优化就只需考虑质点轨迹。

调试方法：
- 若膨胀半径过大，走廊可能根本生成不了；过小则轨迹可能擦边。可在 RViz 中打开 `voxel_map` 点云和 `visualizer/mesh` 同时观察。

---

## 4. 从路径到安全走廊

### 4.1 `plan()` 主流程

源码：`gcopter/src/global_planning.cpp:143-229`

1. 检查 `startGoal` 已有 2 点；否则等待。
2. 调 `sfc_gen::planPath` 获取离散路径 `route`（包含起点终点）。
3. 调 `voxelMap.getSurf(pc)` 得到膨胀障碍表面点。
4. 调 `sfc_gen::convexCover(route, pc, ...)` 构造多面体序列。
5. 调 `sfc_gen::shortCut(hPolys)` 合并相邻冗余多面体。
6. 可视化多面体：`visualizer.visualizePolytope(hPolys)`。
7. 生成起终状态矩阵 `iniState`/`finState`：第一列位置，第二列速度，第三列加速度。
8. 构造并配置 `GCOPTER_PolytopeSFC`。
9. 调 `setup()` → `optimize()` 获得轨迹 `traj`。
10. 成功后记录开始时间 `trajStamp` 并可视化轨迹与路径。

### 4.2 `sfc_gen::planPath` 细节

源码：`gcopter/include/gcopter/sfc_gen.hpp:40-96`

- **空间映射**：把世界坐标系 `[lb, hb]` 映射到 `[0,1]^3` 方便 OMPL。
- **碰撞检测 Lambda**：`mapPtr->query(position) == 0` 表示安全。
- **规划器**：`ompl::geometric::InformedRRTstar`，输入 `timeout` (YAML:`TimeoutRRT`) 决定搜索时间。
- **路径还原**：解出路径后再映回世界坐标。

OMPL 知识补充：RRT* 是随机扩展树算法，Informed 版本会在找到初解后限制采样到椭球体，提升收敛。

### 4.3 `convexCover` + `shortCut`

源码：`gcopter/include/gcopter/sfc_gen.hpp:100-212`

- `progress` 与 `range` 是包围盒增长参数（示例 7.0, 3.0），用于控制多面体长度与宽度。
- 对每个小段 `[a,b]`：
  1. 生成 6 个半空间（轴对齐），保证段落处于包围盒内。
  2. 在障碍表面点中筛选满足 `hp * [p;1] < 0` 的点。
  3. 调 `firi::firi` 计算最大凸多面体。
  4. 如果前一个多面体与当前段跨越的连接点不够平滑，插入额外面保证连接。
- `shortCut` 进一步合并重复多面体，减少优化变量数量。

> 若想观察多面体，可在 RViz 中开启 `/visualizer/mesh`，颜色表示多面体序列。

---

## 5. `GCOPTER_PolytopeSFC`：轨迹优化核心

### 5.1 设定阶段 (`setup`)

源码：`gcopter/include/gcopter/gcopter.hpp:704-781`

要点：
1. **归一化多面体**：对每个半空间 `Ax + b <= 0`，把 `A` 的每一行归一化，方便后续距离计算。
2. **`processCorridor`**：将 H-表示 (`hPolytopes`) 转换成顶点表示 (`vPolytopes`)，并计算用于插值的索引（内部函数，位于同文件 168 行附近）。
3. **片段数量**：
   ```cpp
   pieceIdx = ceil(segment_length / lengthPerPiece);
   pieceN = pieceIdx.sum();
   ```
   - `lengthPerPiece` 传入 `INFINITY`：意味着每个走廊段至少一个片段（示例代码里 `INFINITY`，即根据默认逻辑每个多面体 1 片）。
4. **变量维度**：
   - `temporalDim = pieceN`（每片段一个时间变量）；
   - `spatialDim = Σ(多面体顶点数)`（内部点）。
5. **MINCO & 平坦性对象初始化**：`minco.setConditions(headPVA, tailPVA, pieceN)`，`flatmap.reset(...)`。
6. **分配缓存**：轨迹系数梯度、时间梯度、内点等矩阵在此分配，避免重复 new。

### 5.2 初值生成 (`setInitial` 等)

- `getShortestPath`：在顶点图上寻找最短路径（利用内置 Dijkstra，见文件前半部）。
- `setInitial(shortPath, allocSpeed, pieceIdx, ...)`：
  - 内部点 = 线段中的均匀点；
  - 初始时间 = 段长 / `allocSpeed`（`allocSpeed = 3 * v_max`）。
- `backwardT/backwardP`：将真实时间/位置压缩成优化变量 `tau/xi`。设计理念：优化变量无界，但通过指数映射确保时间正、内点落在多面体内部。

### 5.3 优化与 cost

`optimize(traj, relCostTol)`：
1. 组装变量向量 `x = [tau; xi]`。
2. 设置 L-BFGS 参数：
   ```cpp
   lbfgs_params.mem_size = 256;
   lbfgs_params.past = 3;
   lbfgs_params.delta = relCostTol;
   ```
3. 调 `lbfgs_optimize`：需要提供 `costFunctional` 计算代价与梯度。
4. 成功后：
   - `forwardT(tau, times)` 恢复时间。
   - `forwardP(xi, ...)` 恢复内点。
   - `minco.setParameters(points, times)` -> `minco.getTrajectory(traj)`。
5. 失败时：清空 `traj` 并打印错误信息。

### 5.4 `costFunctional` 结构（重点）

源码：`gcopter/include/gcopter/gcopter.hpp:472-511`

流程：
1. 将 `tau/xi` 映射回实际 `times/points`。
2. 调用 MINCO：
   - `minco.setParameters(points, times)`
   - `minco.getEnergy(cost)` 获得 `J_snap`
   - `minco.getEnergyPartialGradByCoeffs/Times(...)` 获得能量梯度
3. 加上罚函数：
   ```cpp
   attachPenaltyFunctional(times, coeffs, ...,
                           smoothEps, integralRes,
                           magnitudeBd, penaltyWt, flatmap,
                           cost, partialGradByTimes, partialGradByCoeffs);
   ```
4. 通过 MINCO 的 `propogateGrad` 将系数梯度传到内点/时间梯度。
5. 加时间正则项：`cost += rho * times.sum();`。
6. 将梯度映射回 `tau/xi` 空间：`backwardGradT` 和 `backwardGradP`。
7. `normRestrictionLayer`：限制 `xi` 在线段范围内（防止出界）。

> 对计算机专业读者，可以把 `costFunctional` 看成典型的“管线式”自动微分：前半段 `forward` 计算 cost，后半段逐层 `backward`。区别在于很多层（MINCO、平坦性）都手写了解析梯度，效率与稳定性更好。

---

## 6. MINCO `MINCO_S3NU` 深度解析

源码：`gcopter/include/gcopter/minco.hpp:396-645`

### 6.1 状态量

- `A`：带状矩阵（banded system），维度 `6N×6N`（`N`=片段数）。
- `b`：右侧矩阵，大小 `6N×3`（三维空间分别求解）。
- `T1..T5`：`T`, `T^2`, `T^3`, `T^4`, `T^5`。
- `headPVA`, `tailPVA`：起终点 PVA。

### 6.2 `setParameters`

关键结构：
```cpp
// 约束每段末端的系数，保证连续
A(6*i + 5, 6*i + 0~5) = [1, T, T^2, T^3, T^4, T^5]
A(6*i + 6, 6*i + 0~6) = ... // 位置连续
A(6*i + 7, ...)             // 速度连续
A(6*i + 8, ...)             // 加速度连续
```

- 第 0~2 行：设置起点位置/速度/加速度等于 `headPVA`。
- 第 `6N-3`~`6N-1` 行：设置终点 PVA。
- 中间行：连接等式与内点约束。

### 6.3 解线性系统

`A.factorizeLU()` + `A.solve(b)`：
- `factorizeLU()` 只做一次，复杂度 `O(N)`。
- `solve(b)` 用于方程 `Ax=b`，结果直接写回 `b`。

### 6.4 得到轨迹

`getTrajectory(Trajectory<5> &traj)` 遍历片段，取 `b` 中每 6 行的系数，转置/翻转（适配 `Trajectory` 的列序）。

### 6.5 导出梯度

- `getEnergy(...)`：计算 `J_snap`。
- `getEnergyPartialGradByCoeffs/Times`：给出对系数和时间的梯度。
- `propogateGrad`：使用 `solveAdj`（对偶系统）把梯度传给内点/时间。

> 这种“解析梯度 + 稀疏线性系统”组合，是 MINCO 能够实时解高阶轨迹的关键。与数值微分相比，速度和稳定性都更好。

---

## 7. 平坦性映射 `flatness::FlatnessMap`

源码：`gcopter/include/gcopter/flatness.hpp`

### 7.1 输入输出关系

`forward(vel, acc, jer, psi, dpsi, thr, quat, omg)`：
- 输入：速度 `v`、加速度 `a`、加加速度 `jer`、无人机 yaw `ψ` 与其导数。
- 输出：推力 `thr`、姿态四元数 `quat`、机体角速度 `ω`。

`backward(...)` 将梯度从 `thr`, `quat`, `ω` 回传到 `pos/vel/acc/jer`。优化器需要它才能对约束求导。

### 7.2 阻力建模

- 水平阻力：`d_h`
- 垂直阻力：`d_v`
- 寄生阻力：`c_p`
- 为避免 `‖v‖` 为零时的数值问题，引入 `speed_smooth_factor (ϵ)`。

### 7.3 常用调试技巧

- 如需忽略阻力，将 YAML 中 `HorizDrag/VertDrag/ParasDrag` 置 0。
- 如果姿态/推力经常越界，可先观察 `vel/acc`，可能是 `WeightT` 太小导致飞太快。

---

## 8. 罚函数 `attachPenaltyFunctional`

源码：`gcopter/include/gcopter/gcopter.hpp:340-469`

### 8.1 数值积分

- 每段轨迹按 `integralResolution` 切成 `integralResolution` 个子区间，每个采样点采用梯形积分权重 `node` (`0.5` 或 `1.0`)。
- `beta0..beta4` 是 0~4 阶多项式基函数，用于快速计算 `pos/vel/acc/jer/snap`。

### 8.2 罚项一览

1. **位置约束**：要求每个采样点都处于对应多面体内。
2. **速度上限**：`‖vel‖^2 <= v_max^2`。
3. **角速度上限**：`‖ω‖^2 <= ω_max^2`。
4. **倾角上限**：`θ <= θ_max`，`θ = acos(1 - 2(q_x^2 + q_y^2))`。
5. **推力上下限**：借助平均值 `T_mean` 和半径 `T_radius` 实现 `T_min <= T <= T_max`。

每项调用 `smoothedL1(violation, smoothFactor, pena, penaGrad)`：
- 若 `violation ≤ 0`（未违反） → 不计代价。
- 若 `0 < violation < smoothFactor` → 代价二次上升。
- 若 `violation ≥ smoothFactor` → 代价线性上升。

### 8.3 梯度回传

1. 组合 `gradPos/gradVel/...`。
2. 调 `flatMap.backward` 计算 `totalGradPos/Vel/Acc/Jer`。
3. 按照积分权重累加至 `gradC`（对应 MINCO 系数）和 `gradT`（时间变量）。

---

## 9. 轨迹播放与可视化

源码：
- `GlobalPlanner::process` (`gcopter/src/global_planning.cpp:259-304`)
- `Visualizer` (`gcopter/include/misc/visualizer.hpp`)

### 9.1 轨迹回放逻辑

```cpp
if (traj.getPieceNum() > 0) {
    delta = now - trajStamp;
    if (0 < delta < traj.totalDuration()) {
        // 取轨迹状态
        pos = traj.getPos(delta);
        vel = traj.getVel(delta);
        acc = traj.getAcc(delta);
        jer = traj.getJer(delta);

        // 映射到推力/姿态/角速度
        flatmap.forward(..., thr, quat, omg);

        // 发布速度/推力/倾角/角速度
        speedPub.publish(...);
        // ...

        // 画蓝色安全球指示当前飞行位置
        visualizer.visualizeSphere(pos, radius);
    }
}
```

### 9.2 可视化话题

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/visualizer/route` | `visualization_msgs/Marker` (LINE_LIST) | OMPL 找到的粗路径 |
| `/visualizer/trajectory` | Marker (LINE_LIST) | MINCO 优化后的轨迹 |
| `/visualizer/mesh` | Marker (TRIANGLE_LIST) | SFC 多面体网格 |
| `/visualizer/spheres` | Marker (SPHERE_LIST) | 当前飞行位置/安全球 |
| `/visualizer/*` (Float64) | 推力、倾角、角速度、速度 | rqt_plot 监控 |

### 9.3 小技巧

- 如果轨迹看起来“折线”很多，可能 `IntegralIntervs` 太小或 `TimeOutRRT` 太短。
- 若希望导出轨迹，可调用 `Trajectory` 类提供的 `getPositions()` / `getCoeffs()`，自己写文件。

---

## 10. 参数映射（YAML ↔ 代码）

| YAML Key | `Config` 字段 | 使用位置 | 影响 |
| --- | --- | --- | --- |
| `MapTopic` | `mapTopic` | `nh.subscribe(mapTopic, ...)` | 地图话题名 |
| `TargetTopic` | `targetTopic` | `nh.subscribe(targetTopic, ...)` | RViz 目标话题 |
| `DilateRadius` | `dilateRadius` | `voxelMap.dilate`, `zGoal` | 安全半径 |
| `VoxelWidth` | `voxelWidth` | 初始化 VoxelMap 尺寸 | 地图分辨率 |
| `MapBound` | `mapBound` | 初始化、`targetCallBack` | 规划空间边界 |
| `TimeoutRRT` | `timeoutRRT` | `sfc_gen::planPath` | RRT* 求解时间 |
| `MaxVelMag...` | `maxVelMag ...` | `magnitudeBounds` | 各类约束阈值 |
| `VehicleMass` | `vehicleMass` | `physicalParams` | 估计推力 |
| `HorizDrag...` | `horizDrag...` | `flatmap.reset` | 阻力模型 |
| `ChiVec` | `chiVec` | `penaltyWeights` | 罚权重 |
| `WeightT` | `weightT` | 时间代价权重 `rho` |
| `SmoothingEps` | `smoothingEps` | `smoothedL1` 过渡 | 罚函数平滑 |
| `IntegralIntervs` | `integralIntervs` | 数值积分采样数 | 精度/耗时 |
| `RelCostTol` | `relCostTol` | L-BFGS 收敛阈值 | 结束条件 |

配置改动后需 `catkin_make`? → **不需要**，YAML 参数由 ROS param server 读取，修改后重新 `roslaunch` 即可。

---

## 11. 常用操作示例

1. **运行示例**
   ```bash
   roslaunch gcopter global_planning.launch
   # RViz 弹出后，选择 "2D Nav Goal" 连续点两次
   ```
2. **调整最大速度**（比如想要 6 m/s）：
   - 修改 `config/global_planning.yaml` 中 `MaxVelMag: 6.0`
   - 重新 `roslaunch`
   - 在 rqt_plot 观察 `/visualizer/speed` 是否触顶（若频繁到 6，说明确实达到上限）。
3. **打开多面体 Mesh**：
   - RViz 左侧勾选 `/visualizer/mesh`
   - 若颜色太深，可调整 Marker 显示属性。

---

## 12. 调试与排错（常见问题 8 连问）

1. **`Optimization Failed`**
   - 原因：L-BFGS 无法找到下降方向，多半是走廊过窄或初始解太劣。
   - 解决：增大 `DilateRadius`、`TimeoutRRT` 或减小 `WeightT`。
2. **轨迹有明显折点**
   - 检查 `IntegralIntervs` 是否太小（默认 16，必要时提到 32）。
   - 检查罚函数权重 `ChiVec` 是否偏小。
3. **推力/倾角常撞上限**
   - 提高对应 `ChiVec` 权重；或放宽 `MaxTiltAngle/MaxThrust`。
4. **轨迹“穿墙”**
   - 观察多面体 Mesh 是否存在裂缝。必要时增大 `range`（`convexCover` 调用处 3.0）。
5. **跑太慢**
   - 降低 `ChiVec` 中速度罚，或调大 `WeightT` 增强时间压缩。
6. **运行时卡顿**
   - `IntegralIntervs`/`pieceN` 太大时会增加优化时间。可减少地图范围或 `range`。
7. **更换地图**
   - 确保点云与 `MapBound` 一致；可先在 RViz 观察 `/voxel_map` 是否覆盖整个区域。
8. **想输出离线轨迹**
   - 在 `optimize` 成功后，使用 `traj.getPositions()`/`getCoeffs()` 写文件即可。

---

## 13. 进阶：从 MINCO 到 Point-Mass

README_old.md 提到若只需点质点模型，可修改罚函数：

- **最小改动**：YAML 中将 `HorizDrag/VertDrag/ParasDrag = 0`，`ChiVec` 中倾角/推力/角速度罚设为 0。
- **深入改动**：在 `attachPenaltyFunctional` 内直接注释掉 `flatMap.forward/backward`，改成针对 `acc`、`jer` 的限制。此时 cost 完全退化成位置空间的 MINCO + 速度限制，计算更快但忽略动力学细节。

---

## 14. 延伸阅读与参考

- [T-RO 2022 论文](https://arxiv.org/abs/2103.00190)：完整的 GCOPTER 方法推导。
- 仓库内 `misc/flatness.pdf`：非线性阻力下的平坦性公式。
- `Thesis - ZhepeiWang ...`：作者硕士论文，包含 FIRI、SE(3) 扩展等高级内容。
- `gcopter/include/gcopter/firi.hpp`：FIRI 算法实现详情，适合对几何算法感兴趣的读者。

---

> 建议阅读方式：
> 1. 打开本文 + 对应源码文件；
> 2. 跑一次 `roslaunch gcopter global_planning.launch`，在 RViz/rqt_plot 中边看边对照；
> 3. 修改 YAML 参数观察反馈；
> 4. 若想深入算法，可配合论文公式逐行对照解析代码。
