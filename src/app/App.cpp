#include "app/App.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>

#include <imgui.h>
#include <implot.h>

#include "gui/Panels.h"
#include "gui/PlotHelpers.h"
#include "model/Ballistics.h"
#include "model/Export.h"
#include "model/Geometry.h"
#include "model/Search.h"
#include "model/Sweep.h"
#include "util/JsonUtil.h"
#include "util/MathUtil.h"

namespace shootersim::app {

App::App() {
  std::snprintf(configPath_, sizeof(configPath_), "%s", "ShooterSimConfig.json");
  std::snprintf(exportCsvPath_, sizeof(exportCsvPath_), "%s", "ShooterLookup.csv");
  std::snprintf(exportJsonPath_, sizeof(exportJsonPath_), "%s", "ShooterLookup.json");
  std::snprintf(exportHeaderPath_, sizeof(exportHeaderPath_), "%s", "GeneratedShooterLookup.h");

  sweepRequest_.distanceMin = 1.5;
  sweepRequest_.distanceMax = 6.0;
  sweepRequest_.distanceStep = 0.1;

  internalSelfTestPassed_ = RunInternalSelfTest();
  PushLog(internalSelfTestPassed_ ? "Internal math smoke test passed"
                                  : "Internal math smoke test failed");

  SolveSingle();
}

App::~App() { JoinSweepThread(); }

void App::Render() {
  FinalizeSweepIfDone();

  ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

  const bool parameterChanged = DrawParameterPanel();
  const bool singleChanged = DrawSingleSolvePanel();

  if (autoSolve_ && (parameterChanged || singleChanged || solveDirty_)) {
    SolveSingle();
  }

  DrawTrajectoryPanel();
  DrawHeatmapPanel();
  DrawSweepPanel();
  DrawLogPanel();
}

bool App::DrawParameterPanel() {
  bool changed = false;

  ImGui::Begin("Parameters");

  changed |= ImGui::Checkbox("Auto Solve", &autoSolve_);

  int physicsMode = static_cast<int>(config_.physicsMode);
  if (ImGui::Combo("Physics Mode", &physicsMode,
                   "Analytic No Drag\0Numeric Quadratic Drag\0")) {
    config_.physicsMode = static_cast<PhysicsMode>(physicsMode);
    changed = true;
  }

  changed |= ImGui::DragDouble("Gravity (m/s^2)", &config_.gravity, 0.01, 1.0, 30.0, "%.5f");
  changed |= ImGui::DragDouble("Release Height (m)", &config_.releaseHeight, 0.005, 0.0, 3.0,
                               "%.3f");
  changed |= ImGui::DragDouble("Target Height z_target (m)", &config_.zTarget, 0.005, 0.1, 5.0,
                               "%.3f");

  if (config_.physicsMode == PhysicsMode::kNumericQuadraticDrag) {
    changed |= ImGui::DragDouble("Drag Coeff (1/m)", &config_.dragCoefficient, 0.0001, 0.0, 2.0,
                                 "%.4f");
    changed |= ImGui::DragDouble("Drag dt (s)", &config_.dragDt, 0.0001, 0.0001, 0.02, "%.4f");
    changed |=
        ImGui::DragDouble("Drag max time (s)", &config_.dragMaxTime, 0.01, 0.1, 10.0, "%.2f");
  }

  ImGui::Separator();
  int windowMode = static_cast<int>(config_.windowInputMode);
  if (ImGui::Combo("Window Input Mode", &windowMode,
                   "Direct front/back offsets\0Center distance + opening depth\0")) {
    config_.windowInputMode = static_cast<WindowInputMode>(windowMode);
    changed = true;
  }

  if (config_.windowInputMode == WindowInputMode::kDirectFrontBackOffset) {
    ImGui::TextWrapped(
        "Window = [distance + xFrontOffset, distance + xBackOffset] at z_target.");
    changed |= ImGui::DragDouble("xFrontOffset (m)", &config_.xFrontOffset, 0.002, -2.0, 2.0,
                                 "%.3f");
    changed |=
        ImGui::DragDouble("xBackOffset (m)", &config_.xBackOffset, 0.002, -2.0, 2.0, "%.3f");
  } else {
    ImGui::TextWrapped("Window centered on distance with openingDepth.");
    changed |= ImGui::DragDouble("Opening Depth (m)", &config_.openingDepth, 0.002, 0.01, 2.0,
                                 "%.3f");
  }

  changed |= ImGui::DragDouble("Ball Radius (m)", &config_.ballRadius, 0.001, 0.0, 0.4, "%.3f");
  changed |=
      ImGui::DragDouble("Front Margin (m)", &config_.frontMargin, 0.001, 0.0, 0.3, "%.3f");
  changed |= ImGui::DragDouble("Back Margin (m)", &config_.backMargin, 0.001, 0.0, 0.3, "%.3f");

  ImGui::Separator();
  changed |= gui::DragAngleDeg("Theta Min", &config_.thetaMinRad, 0.1, 0.0, 89.0);
  changed |= gui::DragAngleDeg("Theta Max", &config_.thetaMaxRad, 0.1, 0.0, 89.0);
  changed |= ImGui::DragDouble("v Min (m/s)", &config_.vMin, 0.01, 0.1, 60.0, "%.3f");
  changed |= ImGui::DragDouble("v Max (m/s)", &config_.vMax, 0.01, 0.1, 60.0, "%.3f");
  changed |= ImGui::DragDouble("z Ceiling Max (m)", &config_.zCeilingMax, 0.01, 0.1, 20.0,
                               "%.3f");
  changed |= gui::DragAngleDeg("Theta Step", &config_.thetaStepRad, 0.02, 0.01, 5.0);
  changed |= ImGui::DragDouble("v Step (m/s)", &config_.vStep, 0.005, 0.005, 1.0, "%.3f");

  changed |= ImGui::Checkbox("Enable monotonic boundary search", &config_.enableMonotonicBoundarySearch);

  ImGui::Separator();
  changed |= ImGui::Checkbox("Enable entry-angle constraint", &config_.enableEntryAngleConstraint);
  if (config_.enableEntryAngleConstraint) {
    changed |= gui::DragAngleDeg("Min |entry angle|", &config_.minAbsEntryAngleRad, 0.1, 0.0, 89.0);
  }

  changed |= ImGui::Checkbox("Enable flight-time constraint", &config_.enableFlightTimeConstraint);
  if (config_.enableFlightTimeConstraint) {
    changed |=
        ImGui::DragDouble("Max flight time (s)", &config_.maxFlightTime, 0.01, 0.01, 10.0, "%.3f");
  }

  ImGui::Separator();
  int nominalMode = static_cast<int>(config_.nominalMode);
  if (ImGui::Combo("Nominal Mode", &nominalMode,
                   "Window bias (x target)\0Speed interval bias\0")) {
    config_.nominalMode = static_cast<NominalSelectionMode>(nominalMode);
    changed = true;
  }
  changed |= ImGui::SliderDouble("targetBias", &config_.targetBias, 0.0, 1.0, "%.3f");
  changed |= ImGui::SliderDouble("speedBias", &config_.speedBias, 0.0, 1.0, "%.3f");

  ImGui::Separator();
  changed |= ImGui::DragInt("Trajectory Samples", &config_.trajectorySamples, 1.0f, 20, 1000);
  changed |= ImGui::Checkbox("Show trajectory points", &config_.showTrajectoryPoints);
  changed |= ImGui::Checkbox("Show entry point", &config_.showEntryPoint);
  changed |= ImGui::Checkbox("Show apex point", &config_.showApexPoint);
  changed |= ImGui::Checkbox("Show raw/effective window", &config_.showWindowComparison);

  ImGui::Separator();
  ImGui::InputText("Config JSON", configPath_, sizeof(configPath_));
  if (ImGui::Button("Save Config")) {
    std::string err;
    if (util::SaveConfigJson(configPath_, config_, &err)) {
      PushLog(std::string("Saved config: ") + configPath_);
    } else {
      PushLog(std::string("Save config failed: ") + err);
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Load Config")) {
    std::string err;
    SimulationConfig loaded = config_;
    if (util::LoadConfigJson(configPath_, &loaded, &err)) {
      config_ = loaded;
      solveDirty_ = true;
      PushLog(std::string("Loaded config: ") + configPath_);
    } else {
      PushLog(std::string("Load config failed: ") + err);
    }
  }

  if (changed) {
    solveDirty_ = true;
    config_.targetBias = util::Clamp(config_.targetBias, 0.0, 1.0);
    config_.speedBias = util::Clamp(config_.speedBias, 0.0, 1.0);

    if (config_.thetaMinRad > config_.thetaMaxRad) {
      std::swap(config_.thetaMinRad, config_.thetaMaxRad);
    }
    if (config_.vMin > config_.vMax) {
      std::swap(config_.vMin, config_.vMax);
    }
  }

  ImGui::End();
  return changed;
}

bool App::DrawSingleSolvePanel() {
  bool changed = false;

  ImGui::Begin("Single Solve");

  changed |= ImGui::DragDouble("Distance (m)", &singleDistance_, 0.005, 0.1, 20.0, "%.3f");

  if (ImGui::Button("Solve")) {
    SolveSingle();
  }

  ImGui::Separator();
  gui::DrawSolveSummary(singleResult_);

  const auto rawWindow = model::ComputeRawWindow(config_, singleDistance_);
  const auto effWindow = model::ComputeEffectiveWindow(config_, rawWindow);
  ImGui::Separator();
  ImGui::Text("Raw window: [%.3f, %.3f]", rawWindow.xFront, rawWindow.xBack);
  ImGui::Text("Effective window: [%.3f, %.3f]", effWindow.xFrontEff, effWindow.xBackEff);
  if (!effWindow.valid) {
    ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), "%s", effWindow.error.c_str());
  }

  if (singleResult_.hasSolution) {
    ImGui::Separator();
    gui::DrawConstraintStatus(singleResult_.nominalConstraint);
  }

  ImGui::End();
  return changed;
}

void App::DrawTrajectoryPanel() const {
  ImGui::Begin("Trajectory");
  gui::PlotTrajectory(config_, singleResult_);
  ImGui::End();
}

void App::DrawHeatmapPanel() const {
  ImGui::Begin("Heatmap & Curves");
  gui::PlotThetaVHeatmap(singleResult_);
  ImGui::Separator();
  gui::PlotSweepCurves(sweepResult_);
  ImGui::End();
}

void App::DrawSweepPanel() {
  ImGui::Begin("Sweep");

  ImGui::DragDouble("distance_min", &sweepRequest_.distanceMin, 0.01, 0.1, 20.0, "%.3f");
  ImGui::DragDouble("distance_max", &sweepRequest_.distanceMax, 0.01, 0.1, 20.0, "%.3f");
  ImGui::DragDouble("distance_step", &sweepRequest_.distanceStep, 0.005, 0.005, 2.0, "%.3f");

  if (sweepRequest_.distanceMin > sweepRequest_.distanceMax) {
    std::swap(sweepRequest_.distanceMin, sweepRequest_.distanceMax);
  }

  if (sweepRunning_) {
    ImGui::BeginDisabled();
    ImGui::Button("Sweep Running...");
    ImGui::EndDisabled();
  } else if (ImGui::Button("Run Sweep")) {
    StartSweepAsync();
  }

  const float progress = static_cast<float>(sweepProgress_.load());
  ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f));

  if (sweepRunning_) {
    ImGui::Text("Sweeping...");
  } else if (!sweepResult_.points.empty()) {
    ImGui::Text("Sweep points: %d", static_cast<int>(sweepResult_.points.size()));
    ImGui::Text("Sweep total time: %.3f ms", sweepResult_.totalTimeMs);
    ImGui::Text("Cache hit: %s", sweepResult_.fromCache ? "YES" : "NO");
  }

  ImGui::Separator();
  ImGui::InputText("Export CSV", exportCsvPath_, sizeof(exportCsvPath_));
  if (ImGui::Button("Export CSV")) {
    std::string err;
    if (model::ExportSweepToCsv(exportCsvPath_, sweepResult_, &err)) {
      PushLog(std::string("Exported CSV: ") + exportCsvPath_);
    } else {
      PushLog(std::string("CSV export failed: ") + err);
    }
  }

  ImGui::InputText("Export JSON", exportJsonPath_, sizeof(exportJsonPath_));
  if (ImGui::Button("Export JSON")) {
    std::string err;
    if (model::ExportSweepToJson(exportJsonPath_, config_, sweepResult_, &err)) {
      PushLog(std::string("Exported JSON: ") + exportJsonPath_);
    } else {
      PushLog(std::string("JSON export failed: ") + err);
    }
  }

  ImGui::InputText("Export Header", exportHeaderPath_, sizeof(exportHeaderPath_));
  if (ImGui::Button("Export C++ Header")) {
    std::string err;
    if (model::ExportSweepToCppHeader(exportHeaderPath_, sweepResult_, &err)) {
      PushLog(std::string("Exported Header: ") + exportHeaderPath_);
    } else {
      PushLog(std::string("Header export failed: ") + err);
    }
  }

  ImGui::End();
}

void App::DrawLogPanel() const {
  ImGui::Begin("Status");
  ImGui::Text("Internal tests: %s", internalSelfTestPassed_ ? "PASS" : "FAIL");
  ImGui::Text("Single solve time: %.3f ms", singleResult_.solveTimeMs);
  if (!sweepResult_.points.empty()) {
    ImGui::Text("Last sweep time: %.3f ms", sweepResult_.totalTimeMs);
  }

  ImGui::Separator();
  for (const auto& line : logs_) {
    ImGui::TextWrapped("%s", line.c_str());
  }
  ImGui::End();
}

void App::SolveSingle() {
  solveDirty_ = false;
  singleResult_ = model::SolveForDistance(config_, singleDistance_);
}

void App::StartSweepAsync() {
  if (sweepRunning_) {
    return;
  }

  JoinSweepThread();
  sweepProgress_.store(0.0);
  sweepDone_.store(false);
  sweepRunning_ = true;

  const SimulationConfig config = config_;
  const SweepRequest request = sweepRequest_;

  sweepThread_ = std::thread([this, config, request]() {
    const auto result = model::RunSweep(config, request, [this](double p) { sweepProgress_.store(p); });
    {
      std::lock_guard<std::mutex> lock(sweepMutex_);
      sweepThreadResult_ = result;
    }
    sweepDone_.store(true);
  });

  PushLog("Sweep started");
}

void App::FinalizeSweepIfDone() {
  if (!sweepRunning_ || !sweepDone_.load()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(sweepMutex_);
    sweepResult_ = sweepThreadResult_;
  }
  sweepRunning_ = false;
  sweepDone_.store(false);

  JoinSweepThread();
  PushLog("Sweep finished");
}

void App::JoinSweepThread() {
  if (sweepThread_.joinable()) {
    sweepThread_.join();
  }
}

void App::PushLog(const std::string& message) {
  logs_.push_back(message);
  if (logs_.size() > 50U) {
    logs_.erase(logs_.begin());
  }
}

bool App::RunInternalSelfTest() {
  SimulationConfig cfg;
  cfg.physicsMode = PhysicsMode::kAnalyticNoDrag;
  const double theta = util::DegToRad(45.0);
  const double v = 10.0;

  const auto sample = model::EvaluateAtTime(cfg, theta, v, 1.0);
  const double expectedX = 10.0 * std::cos(theta);
  const double expectedZ = cfg.releaseHeight + 10.0 * std::sin(theta) - 0.5 * cfg.gravity;

  if (!util::NearlyEqual(sample.x, expectedX, 1e-6)) {
    return false;
  }
  if (!util::NearlyEqual(sample.z, expectedZ, 1e-6)) {
    return false;
  }

  const auto apex = model::ComputeApex(cfg, theta, v);
  if (!apex.valid || apex.zApex < cfg.releaseHeight) {
    return false;
  }

  return true;
}

}  // namespace shootersim::app
