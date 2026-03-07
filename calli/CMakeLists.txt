cmake_minimum_required(VERSION 3.22)
project(ShooterCalibrationTool VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SHOOTER_CALIBRATION_BUILD_TESTS "Build calibration tests" ON)

add_library(calibration_core STATIC
  src/MathUtil.cpp
  src/CsvUtil.cpp
  src/TheoryDataLoader.cpp
  src/CalibrationDataLoader.cpp
  src/CalibrationPlanner.cpp
  src/CalibrationAggregator.cpp
  src/RuntimeModel.cpp
  src/ModelFitter.cpp
  src/SegmentAnalyzer.cpp
  src/Exporter.cpp
  src/CalibrationApp.cpp
)

target_include_directories(calibration_core PUBLIC src)

if(MSVC)
  target_compile_options(calibration_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(calibration_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(ShooterCalibrationTool src/main.cpp)
target_link_libraries(ShooterCalibrationTool PRIVATE calibration_core)

if(SHOOTER_CALIBRATION_BUILD_TESTS)
  enable_testing()
  add_executable(ShooterCalibrationTests
    tests/TestMain.cpp
    tests/test_planner.cpp
    tests/test_aggregator.cpp
    tests/test_fitter.cpp
    tests/test_segment.cpp
    tests/test_export.cpp
  )
  target_link_libraries(ShooterCalibrationTests PRIVATE calibration_core)

  if(MSVC)
    target_compile_options(ShooterCalibrationTests PRIVATE /W4 /permissive-)
  else()
    target_compile_options(ShooterCalibrationTests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  add_test(NAME ShooterCalibrationTests COMMAND $<TARGET_FILE:ShooterCalibrationTests>)
endif()
