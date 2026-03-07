# FRC Shooter Tools Workspace

This repository now uses a parallel layout:

- `bestshootsim/` : Stage-1 desktop ballistic simulation + search tool
- `calli/` : Stage-2 calibration planning + fitting tool

## Build From Workspace Root (CMD)

```cmd
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DBUILD_BESTSHOOTSIM=ON -DBUILD_CALLI=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Build Only Stage-1

```cmd
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DBUILD_BESTSHOOTSIM=ON -DBUILD_CALLI=OFF
cmake --build build --config Release --target ShooterSim ShooterSimTests
```

## Build Only Stage-2

```cmd
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DBUILD_BESTSHOOTSIM=OFF -DBUILD_CALLI=ON
cmake --build build --config Release --target ShooterCalibrationTool ShooterCalibrationTests
```

