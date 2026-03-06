# ShooterSim (FRC 2D Shooter Solver)

ShooterSim 是一个用于 FRC 射击系统前期建模与求解验证的桌面工具。  
它把“弹道模型 + 约束判断 + 最优化搜索 + 可视化 + 导出”放在同一个 C++20 项目里，方便在 robot code 之外快速迭代。

## 背景与目标

在射击机构开发中，先用桌面仿真把数学逻辑跑通，通常比直接上车调参更快、更可控。  
这个项目的目标是：

- 验证 2D 弹道和几何模型是否正确
- 在约束下搜索每个距离的最优 `pitch + velocity`
- 以“速度容忍区间最大（deltaV 最大）”作为核心优化目标
- 生成可移植到机器人代码的 lookup 数据

当前版本优先实现解析无阻力模型，并预留二次阻力数值模型接口。

## 主要功能

- 2D 几何模型：release point、目标窗口、有效窗口收缩
- 两种目标窗口输入方式：
- `centerDistance + openingDepth`
- `xFront/xBack` 偏移模式
- 弹道求解：
- `Analytic No Drag`（解析）
- `Numeric Quadratic Drag`（数值积分）
- 完整约束判断：
- pitch/速度范围
- 天花板高度
- 必须有下降段交点
- 必须命中有效窗口
- 可选入射角/飞行时间约束
- 单点最优化（Single Solve）
- 区间批量求解（Sweep）
- 结果导出：CSV / JSON / C++ header
- 基础单元测试（ballistics / constraints / search）

## 当前默认常量

按当前代码默认值（已更新）：

- Pitch 范围：`44.57 deg ~ 70.53 deg`
- 飞轮理论速度范围：`0 ~ 20 m/s`
- 高度：`releaseHeight = 0.66 m`, `zTarget = 1.82 m`
- `dz = zTarget - releaseHeight + kTargetOffsetZ = 1.16 m`（默认 `kTargetOffsetZ = 0`）
- 距离 sweep 默认范围：`1.0 ~ 6.1 m`
- 距离 sweep 默认步长：`0.01 m`
- pitch 搜索默认步长：`0.01 deg`
- 天花板约束默认值：`zCeilingMax = 3.3 m`
- HUB 顶部开口直径（2D 简化 openingDepth）：`1.06 m`
- FUEL 直径：`0.15 m`（球半径 `0.075 m`）
- 默认窗口边距：`frontMargin = 0`, `backMargin = 0`
- 默认 nominal 策略：`Speed interval bias`，`speedBias = 0.5`（速度区间对称鲁棒）

## 工程结构

```text
ShooterSim/
  CMakeLists.txt
  README.md
  src/
    main.cpp
    app/
      App.h
      App.cpp
    model/
      Types.h
      Geometry.h / .cpp
      Ballistics.h / .cpp
      Constraints.h / .cpp
      Search.h / .cpp
      Sweep.h / .cpp
      Export.h / .cpp
    gui/
      Panels.h / .cpp
      PlotHelpers.h / .cpp
    util/
      MathUtil.h / .cpp
      CsvUtil.h / .cpp
      JsonUtil.h / .cpp
  tests/
    TestFramework.h
    TestMain.cpp
    test_ballistics.cpp
    test_constraints.cpp
    test_search.cpp
```

## 命令行编译与运行（Windows CMD）

在项目根目录执行：

```cmd
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DSHOOTERSIM_BUILD_GUI=ON -DSHOOTERSIM_BUILD_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
build\Release\ShooterSim.exe
```

如果你用 VS2022：

```cmd
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DSHOOTERSIM_BUILD_GUI=ON -DSHOOTERSIM_BUILD_TESTS=ON
```

## GUI 使用说明

启动后默认会看到 6 个固定面板：

- `Parameters`
- `Single Solve`
- `Trajectory`
- `Heatmap & Curves`
- `Sweep`
- `Status`

`Auto Solve` 默认开启，改参数会自动刷新单点求解和轨迹。

### Single Solve

输入一个 `distance`，立即得到该距离下最优解：

- `bestTheta`
- `bestVNominal`
- `bestVLow`, `bestVHigh`
- `bestDeltaV`
- 入射点/入射角/apex/失败原因

优化目标是：在约束满足时，最大化 `deltaV = vHigh - vLow`。

### Sweep

设置 `distance_min / distance_max / distance_step` 后执行 `Run Sweep`，得到整段距离曲线。  
Sweep 本质是对每个距离调用同一套单点搜索器。

### Trajectory

显示 2D 弹道：

- `v_low`
- `v_nominal`
- `v_high`
- raw window / effective window
- entry point
- apex

### Heatmap

`theta-v` 可行性热力图，表示在“当前 single distance”下哪些 `(theta, v)` 组合满足所有约束。

### Status

显示运行状态和日志：

- 内部 smoke test 结果
- 最近求解耗时
- sweep 耗时
- 导入/导出/缓存命中信息

## GUI 参数说明（核心）

- `releaseHeight`, `zTarget`  
定义发射点和目标平面高度。
- `windowInputMode`  
窗口输入模式，支持中心+深度或前后偏移。
- `openingDepth`  
目标窗口在 x 方向的开口长度（2D 模型）。
- `ballRadius`  
球半径，用于有效窗口收缩。
- `frontMargin`, `backMargin`  
对有效窗口做额外保守收缩（当前默认都为 0）。
- `thetaMin/Max`, `vMin/Max`  
机械/速度约束边界。
- `zCeilingMax`  
抛物线最高点约束。
- `thetaStep`, `vStep`  
搜索离散精度，越小越精细但耗时越高。
- `targetBias`, `speedBias`  
nominal 选取策略偏置参数。

## 导出与缓存

- `Export CSV`：导出 sweep 全距离结果（含 `has_solution` 与诊断字段）
- `Export JSON`：导出配置+结果，可用于后续自动加载
- `Export C++ Header`：生成机器人可直接引用的 lookup 表

默认文件名：

- `ShooterLookup.csv`
- `ShooterLookup.json`
- `GeneratedShooterLookup.h`

程序启动时会尝试读取项目根目录的 `ShooterLookup.json`。  
当 JSON 中配置与当前默认参数匹配时，会自动载入 sweep 结果，避免每次重跑。

## 核心接口索引

建议从这些头文件开始阅读：

- 几何：`src/model/Geometry.h`
- 弹道：`src/model/Ballistics.h`
- 约束：`src/model/Constraints.h`
- 单点最优：`src/model/Search.h`
- 批量 sweep：`src/model/Sweep.h`
- 导出：`src/model/Export.h`
- 配置与结果 JSON：`src/util/JsonUtil.h`

关键函数：

- `SolveForDistance(const SimulationConfig&, double distance)`
- `RunSweep(const SimulationConfig&, const SweepRequest&, ...)`
- `EvaluateConstraints(...)`
- `ComputeIntersectionAtHeight(...)`
- `ComputeApex(...)`
- `BuildTrajectory(...)`

## 移植到 FRC Robot Code 的建议

建议迁移：

- `src/model/*`
- `src/util/MathUtil.*`

通常不迁移：

- `src/main.cpp`
- `src/app/*`
- `src/gui/*`

实践上推荐流程：

1. 场外用 Sweep 生成 lookup
2. 导出为 header 或 CSV/JSON
3. 在 robot code 查表 + 插值
4. 通过实机标定做误差修正

## 测试

测试命令：

```cmd
ctest --test-dir build -C Release --output-on-failure
```

当前测试覆盖：

- 弹道解析正确性
- 约束判定（含仅接受下降段交点）
- 搜索结果的有效性与稳定性
