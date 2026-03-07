# Shooter Calibration Tool (Phase 2)

`calli` 是第二阶段离线标定工具，用来把第一阶段的理论射击结果转换成可用于机器人代码的：

- 标定点推荐
- 实测数据聚合
- `theory_vel_mps -> motor_target_rpm` 映射拟合
- 分段建议
- 机器人可直接使用的导出头文件

## Build

从仓库根目录执行：

```cmd
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DBUILD_BESTSHOOTSIM=OFF -DBUILD_CALLI=ON
cmake --build build --config Release --target ShooterCalibrationTool ShooterCalibrationTests
ctest --test-dir build -C Release -R ShooterCalibrationTests --output-on-failure
```

可执行文件路径：

```text
build\calli\Release\ShooterCalibrationTool.exe
```

## Input And Output

### 1) 理论 CSV 输入

`calli` 的理论输入，默认就是第一阶段 `bestshootsim` 导出的原生 CSV，例如：

```text
bestshootsim\ShooterLookup.csv
```

这个文件只要包含这些列，就满足 `calli` 的理论输入格式：

```csv
distance_m,pitch_deg,theory_vel_mps
```

如果还包含这些附加列，会被一起利用：

```csv
v_low_mps,v_high_mps,delta_v_mps
```

也就是说，通常不需要额外改格式，直接把 `bestshootsim\ShooterLookup.csv` 传给 `calli` 即可。

### 2) Planner 输出

`plan` 会读取理论 CSV，然后输出推荐标定点表。

如果你这样运行：

```cmd
build\calli\Release\ShooterCalibrationTool.exe plan ^
  --input bestshootsim\ShooterLookup.csv ^
  --output calli\plan_output\recommended_points.csv ^
  --mode sensitivity ^
  --target-count 9 ^
  --error-threshold 0.05 ^
  --use-delta-v true ^
  --verbose
```

那么主要输出会在：

```text
calli\plan_output\recommended_points.csv
calli\plan_output\recommended_points_all_scores.csv
calli\plan_output\recommended_points_summary.json
calli\plan_output\recommended_points_summary.md
```

如果你改用 `--out-dir calli\plan_output`，则默认输出名是：

```text
calli\plan_output\recommended_calibration_points.csv
calli\plan_output\all_scores.csv
calli\plan_output\planner_summary.json
calli\plan_output\calibration_report.md
```

### 3) 推荐点 CSV 的作用

`recommended_points.csv` 是面向现场标定的工作表。列顺序已经调整成“先标定输入，后解释信息”：

```csv
distance_m,pitch_deg,theory_vel_mps,motor_target_rpm,measured_motor_rpm,hit,shot_index,lane_index,notes,delta_v_mps,final_score,reason_tags,original_index
```

其中前半部分是你现场直接填写的：

- `distance_m`
- `pitch_deg`
- `theory_vel_mps`
- `motor_target_rpm`
- `measured_motor_rpm`
- `hit`
- `shot_index`
- `lane_index`
- `notes`

后半部分是算法解释字段：

- `delta_v_mps`
- `final_score`
- `reason_tags`
- `original_index`

### 4) 标定 CSV 输入

第二阶段拟合时，`--calib` 可以直接使用你从 `plan` 导出的推荐点 CSV，只要你已经回填了 `motor_target_rpm`。

也就是这个流程：

1. `bestshootsim\ShooterLookup.csv` -> `calli plan`
2. 得到 `calli\plan_output\recommended_points.csv`
3. 现场在这张表里填写 `motor_target_rpm`
4. 把这张表直接作为 `--calib` 输入给 `fit` 或 `full`

## Typical Commands

### Plan

```cmd
build\calli\Release\ShooterCalibrationTool.exe plan ^
  --input bestshootsim\ShooterLookup.csv ^
  --output calli\plan_output\recommended_points.csv ^
  --mode sensitivity ^
  --target-count 9 ^
  --error-threshold 0.05 ^
  --use-delta-v true ^
  --verbose
```

### Full analysis

```cmd
build\calli\Release\ShooterCalibrationTool.exe full ^
  --theory bestshootsim\ShooterLookup.csv ^
  --calib calli\plan_output\recommended_points.csv ^
  --out-dir calli\full_output ^
  --target-count 9 ^
  --mode sensitivity
```

### Fit only

```cmd
build\calli\Release\ShooterCalibrationTool.exe fit ^
  --theory bestshootsim\ShooterLookup.csv ^
  --calib calli\plan_output\recommended_points.csv ^
  --out-dir calli\fit_output
```

## Output Files

`fit` / `full` 常见输出目录内容：

- `aggregated_calibration_points.csv`
- `model_report.csv`
- `fit_curve.csv`
- `fit_residuals.csv`
- `calibration_result.json`
- `calibration_report.md`
- `GeneratedShooterCalibration.h`

其中：

- `fit_curve.csv`：拟合曲线采样点，可直接拿去画图
- `fit_residuals.csv`：每个实测点的残差，可检查分段是否合理

## Notes

- `calli` 默认优先推荐稳定、单调、适合 robot runtime 的模型。
- 默认基线是 lookup/线性插值；如果分段线性明显更好，会给出分段建议。
- 第一阶段和第二阶段现在是平级目录：
  - `bestshootsim/`
  - `calli/`

