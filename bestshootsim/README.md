# BestShooterSim (Stage-1)

`bestshootsim` 是第一阶段桌面仿真工具，用于验证：

- 2D 抛体解析/数值模型
- 命中约束判定
- 单距离最优解（最大速度容忍区间）
- 距离 sweep 与导出（CSV/JSON/C++ header）

## 构建（从仓库根目录）

```cmd
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DBUILD_BESTSHOOTSIM=ON -DBUILD_CALLI=OFF
cmake --build build --config Release --target ShooterSim ShooterSimTests
ctest --test-dir build -C Release -R ShooterSimTests --output-on-failure
```

可执行文件：

```text
build\bestshootsim\Release\ShooterSim.exe
```

## 主要源码入口

- GUI 主入口: `bestshootsim/src/main.cpp`
- App 逻辑: `bestshootsim/src/app/App.h/.cpp`
- 核心数学模型:
  - `bestshootsim/src/model/Ballistics.*`
  - `bestshootsim/src/model/Constraints.*`
  - `bestshootsim/src/model/Search.*`
  - `bestshootsim/src/model/Sweep.*`

## 导出给第二阶段

第一阶段推荐导出 CSV（如 `ShooterLookup.csv`），然后交给 `calli` 做第二阶段标定规划与拟合。

