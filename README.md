# ShooterSim (FRC 2026 Fuel Shooter Desktop Simulator)

ShooterSim 是一个独立于 robot 主程序的 C++20 桌面工具，用于在第一阶段快速验证 2D 射击数学模型、约束判定与最优解搜索。

## 功能概览

- 2D 几何模型（release point + target opening）
- 无阻力解析抛体完整实现（并预留数值 drag 模式）
- 约束判定（机械角度、速度、ceiling、下降段交点、有效窗口、可选入射角/飞行时间）
- 最优解搜索目标：最大速度容忍区间 `delta_v = v_high - v_low`
- nominal 策略：
  - 模式 A：按窗口位置偏置（targetBias）
  - 模式 B：按速度区间偏置（speedBias）
- 单距离求解 + 批量距离 sweep
- GUI：参数调节、轨迹图、theta-v 热力图、distance 曲线、状态日志
- 导出：CSV / JSON / C++ constexpr Header
- 配置 JSON 保存/加载
- sweep 结果缓存（参数不变直接命中缓存）
- 轻量测试（ballistics / constraints / search）

## 单位约定

- 内部统一 SI：`m`, `s`, `rad`, `m/s`
- GUI 角度输入显示为 degree，内部转换到 rad

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

## 构建

> 需要 CMake 3.22+、C++20 编译器、Git（FetchContent 拉取依赖）

```powershell
cmake -S . -B build -DSHOOTERSIM_BUILD_GUI=ON -DSHOOTERSIM_BUILD_TESTS=ON
cmake --build build --config Release
```

依赖由 FetchContent 自动拉取：

- GLFW
- Dear ImGui
- ImPlot
- nlohmann/json

## 运行

GUI：

```powershell
.\build\Release\ShooterSim.exe
```

测试：

```powershell
.\build\Release\ShooterSimTests.exe
```

## GUI 主要区域

- `Parameters`：全部物理/几何/搜索/约束参数，支持配置 JSON 保存/加载
- `Single Solve`：单距离求解，显示最优解、窗口、约束判定和失败原因
- `Trajectory`：显示 `v_low / v_nominal / v_high` 三条轨迹与窗口
- `Heatmap & Curves`：theta-v 有效性热力图 + distance 曲线
- `Sweep`：批量求解、进度、导出
- `Status`：最近状态、耗时、内部 smoke test 状态

## 导出

- CSV：用于快速分析
- JSON：用于记录 sweep 与配置快照
- Header：直接用于机器人代码

生成头文件示例（默认文件名 `GeneratedShooterLookup.h`）：

```cpp
struct ShooterPoint { double distance; double thetaDeg; double vNominal; double vLow; double vHigh; };
constexpr std::array<ShooterPoint, N> kShooterLookup = { ... };
```

## 向 robot code 迁移建议

1. 将 `src/model` 作为纯数学模块迁移到 robot 工程（不带 GUI）。
2. 保持单位一致（rad、m/s）并在输入层做 degree 转换。
3. 使用导出的 `GeneratedShooterLookup.h` 做第一版查表。
4. 第二阶段将理论 `v_nominal` 映射到电机目标速度（RPM）并做实机标定。
5. 逐步启用 drag 模式或加入实测修正项（偏置/分段拟合）。

## 说明

- 当前重点是 `kAnalyticNoDrag` 完整正确性。
- `kNumericQuadraticDrag` 为预留/可选模式，便于后续迭代到更高保真模型。
