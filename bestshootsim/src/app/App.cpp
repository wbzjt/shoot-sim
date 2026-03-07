#include "app/App.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
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
namespace {

struct PanelLayout {
  ImVec2 pos;
  ImVec2 size;
};

bool DragDoubleCompat(const char* label,
                      double* value,
                      float speed,
                      double minValue,
                      double maxValue,
                      const char* format) {
  float v = static_cast<float>(*value);
  const bool changed =
      ImGui::DragFloat(label, &v, speed, static_cast<float>(minValue),
                       static_cast<float>(maxValue), format);
  if (changed) {
    *value = static_cast<double>(v);
  }
  return changed;
}

bool SliderDoubleCompat(const char* label,
                        double* value,
                        double minValue,
                        double maxValue,
                        const char* format) {
  float v = static_cast<float>(*value);
  const bool changed = ImGui::SliderFloat(label, &v, static_cast<float>(minValue),
                                          static_cast<float>(maxValue), format);
  if (changed) {
    *value = static_cast<double>(v);
  }
  return changed;
}

void ApplyPanelLayout(const PanelLayout& layout) {
  ImGui::SetNextWindowPos(layout.pos, ImGuiCond_Always);
  ImGui::SetNextWindowSize(layout.size, ImGuiCond_Always);
}

PanelLayout MakePanel(float x, float y, float w, float h) {
  PanelLayout out;
  out.pos = ImVec2(x, y);
  out.size = ImVec2(std::max(120.0f, w), std::max(100.0f, h));
  return out;
}

}  // namespace

App::App() {
  std::snprintf(configPath_, sizeof(configPath_), "%s", "ShooterSimConfig.json");
  std::snprintf(exportCsvPath_, sizeof(exportCsvPath_), "%s", "ShooterLookup.csv");
  std::snprintf(exportJsonPath_, sizeof(exportJsonPath_), "%s", "ShooterLookup.json");
  std::snprintf(exportHeaderPath_, sizeof(exportHeaderPath_), "%s", "GeneratedShooterLookup.h");

  sweepRequest_.distanceMin = 1.0;
  sweepRequest_.distanceMax = 6.1;
  sweepRequest_.distanceStep = 0.01;

  internalSelfTestPassed_ = RunInternalSelfTest();
  PushLog(internalSelfTestPassed_ ? "Internal math smoke test passed"
                                  : "Internal math smoke test failed");

  SolveSingle();

  if (std::filesystem::exists(exportJsonPath_)) {
    SimulationConfig loadedConfig;
    SweepResult loadedSweep;
    std::string err;
    if (util::LoadSweepJson(exportJsonPath_, &loadedConfig, &loadedSweep, &err)) {
      const std::string keyCurrent = model::BuildSweepCacheKey(config_, sweepRequest_);
      const std::string keyLoaded =
          model::BuildSweepCacheKey(loadedConfig, loadedSweep.request);
      if (keyCurrent == keyLoaded) {
        sweepResult_ = loadedSweep;
        PushLog(std::string("Loaded cached sweep: ") + exportJsonPath_);
      } else {
        PushLog(std::string("Cached sweep config mismatch, ignored: ") + exportJsonPath_);
      }
    } else {
      PushLog(std::string("Failed to load cached sweep: ") + err);
    }
  }
}

App::~App() { JoinSweepThread(); }

void App::Render() {
  FinalizeSweepIfDone();

  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const ImVec2 workPos = viewport->WorkPos;
  const ImVec2 workSize = viewport->WorkSize;
  const float gap = 10.0f;

  const float leftWidth = 390.0f;
  const float rightX = workPos.x + leftWidth + gap;
  const float rightWidth = std::max(320.0f, workSize.x - leftWidth - gap);
  const float leftHeight = workSize.y;

  const float paramHeight = leftHeight * 0.58f;
  const float singleHeight = leftHeight * 0.24f;
  const float statusHeight = leftHeight - paramHeight - singleHeight - gap * 2.0f;

  const float trajHeight = workSize.y * 0.46f;
  const float heatHeight = workSize.y * 0.30f;
  const float sweepHeight = workSize.y - trajHeight - heatHeight - gap * 2.0f;

  ApplyPanelLayout(MakePanel(workPos.x, workPos.y, leftWidth, paramHeight));

  const bool parameterChanged = DrawParameterPanel();

  ApplyPanelLayout(MakePanel(workPos.x, workPos.y + paramHeight + gap, leftWidth,
                             singleHeight));
  const bool singleChanged = DrawSingleSolvePanel();

  if (autoSolve_ && (parameterChanged || singleChanged || solveDirty_)) {
    SolveSingle();
  }

  ApplyPanelLayout(MakePanel(rightX, workPos.y, rightWidth, trajHeight));
  DrawTrajectoryPanel();
  ApplyPanelLayout(
      MakePanel(rightX, workPos.y + trajHeight + gap, rightWidth, heatHeight));
  DrawHeatmapPanel();
  ApplyPanelLayout(MakePanel(rightX, workPos.y + trajHeight + gap + heatHeight + gap,
                             rightWidth, sweepHeight));
  DrawSweepPanel();
  ApplyPanelLayout(MakePanel(workPos.x,
                             workPos.y + paramHeight + gap + singleHeight + gap,
                             leftWidth, statusHeight));
  DrawLogPanel();
}

bool App::DrawParameterPanel() {
  bool changed = false;

  ImGui::Begin("Parameters", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  changed |= ImGui::Checkbox("Auto Solve", &autoSolve_);

  int physicsMode = static_cast<int>(config_.physicsMode);
  if (ImGui::Combo("Physics Mode", &physicsMode,
                   "Analytic No Drag\0Numeric Quadratic Drag\0")) {
    config_.physicsMode = static_cast<PhysicsMode>(physicsMode);
    changed = true;
  }

  changed |=
      DragDoubleCompat("Gravity (m/s^2)", &config_.gravity, 0.01f, 1.0, 30.0, "%.5f");
  changed |= DragDoubleCompat("Release Height (m)", &config_.releaseHeight, 0.005f, 0.0,
                              3.0, "%.3f");
  changed |= DragDoubleCompat("Target Height z_target (m)", &config_.zTarget, 0.005f,
                              0.1, 5.0, "%.3f");

  if (config_.physicsMode == PhysicsMode::kNumericQuadraticDrag) {
    changed |= DragDoubleCompat("Drag Coeff (1/m)", &config_.dragCoefficient, 0.0001f,
                                0.0, 2.0, "%.4f");
    changed |= DragDoubleCompat("Drag dt (s)", &config_.dragDt, 0.0001f, 0.0001, 0.02,
                                "%.4f");
    changed |= DragDoubleCompat("Drag max time (s)", &config_.dragMaxTime, 0.01f, 0.1,
                                10.0, "%.2f");
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
    changed |= DragDoubleCompat("xFrontOffset (m)", &config_.xFrontOffset, 0.002f, -2.0,
                                2.0, "%.3f");
    changed |= DragDoubleCompat("xBackOffset (m)", &config_.xBackOffset, 0.002f, -2.0,
                                2.0, "%.3f");
  } else {
    ImGui::TextWrapped("Window centered on distance with openingDepth.");
    changed |= DragDoubleCompat("Opening Depth (m)", &config_.openingDepth, 0.002f, 0.01,
                                2.0, "%.3f");
  }

  changed |= DragDoubleCompat("Ball Radius (m)", &config_.ballRadius, 0.001f, 0.0, 0.4,
                              "%.3f");
  changed |= DragDoubleCompat("Front Margin (m)", &config_.frontMargin, 0.001f, 0.0, 0.3,
                              "%.3f");
  changed |= DragDoubleCompat("Back Margin (m)", &config_.backMargin, 0.001f, 0.0, 0.3,
                              "%.3f");

  ImGui::Separator();
  changed |= gui::DragAngleDeg("Theta Min", &config_.thetaMinRad, 0.1, 0.0, 89.0);
  changed |= gui::DragAngleDeg("Theta Max", &config_.thetaMaxRad, 0.1, 0.0, 89.0);
  changed |= DragDoubleCompat("v Min (m/s)", &config_.vMin, 0.01f, 0.0, 60.0, "%.3f");
  changed |= DragDoubleCompat("v Max (m/s)", &config_.vMax, 0.01f, 0.0, 60.0, "%.3f");
  changed |= DragDoubleCompat("z Ceiling Max (m)", &config_.zCeilingMax, 0.01f, 0.1,
                              20.0, "%.3f");
  changed |= gui::DragAngleDeg("Theta Step", &config_.thetaStepRad, 0.02, 0.01, 5.0);
  changed |= DragDoubleCompat("v Step (m/s)", &config_.vStep, 0.005f, 0.005, 1.0, "%.3f");

  changed |= ImGui::Checkbox("Enable monotonic boundary search", &config_.enableMonotonicBoundarySearch);

  ImGui::Separator();
  changed |= ImGui::Checkbox("Enable entry-angle constraint", &config_.enableEntryAngleConstraint);
  if (config_.enableEntryAngleConstraint) {
    changed |= gui::DragAngleDeg("Min |entry angle|", &config_.minAbsEntryAngleRad, 0.1, 0.0, 89.0);
  }

  changed |= ImGui::Checkbox("Enable flight-time constraint", &config_.enableFlightTimeConstraint);
  if (config_.enableFlightTimeConstraint) {
    changed |= DragDoubleCompat("Max flight time (s)", &config_.maxFlightTime, 0.01f,
                                0.01, 10.0, "%.3f");
  }

  ImGui::Separator();
  int nominalMode = static_cast<int>(config_.nominalMode);
  if (ImGui::Combo("Nominal Mode", &nominalMode,
                   "Window bias (x target)\0Speed interval bias\0")) {
    config_.nominalMode = static_cast<NominalSelectionMode>(nominalMode);
    changed = true;
  }
  changed |= SliderDoubleCompat("targetBias", &config_.targetBias, 0.0, 1.0, "%.3f");
  changed |= SliderDoubleCompat("speedBias", &config_.speedBias, 0.0, 1.0, "%.3f");

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

  ImGui::Begin("Single Solve", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  changed |= DragDoubleCompat("Distance (m)", &singleDistance_, 0.005f, 0.1, 20.0, "%.3f");

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
  ImGui::Begin("Trajectory", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
  gui::PlotTrajectory(config_, singleResult_);
  ImGui::End();
}

void App::DrawHeatmapPanel() const {
  ImGui::Begin("Heatmap & Curves", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
  gui::PlotThetaVHeatmap(singleResult_);
  ImGui::Separator();
  gui::PlotSweepCurves(sweepResult_);
  ImGui::End();
}

void App::DrawSweepPanel() {
  ImGui::Begin("Sweep", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  DragDoubleCompat("distance_min", &sweepRequest_.distanceMin, 0.01f, 0.1, 20.0, "%.3f");
  DragDoubleCompat("distance_max", &sweepRequest_.distanceMax, 0.01f, 0.1, 20.0, "%.3f");
  DragDoubleCompat("distance_step", &sweepRequest_.distanceStep, 0.005f, 0.005, 2.0,
                   "%.3f");

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
  ImGui::TextUnformatted("CSV Export");
  ImGui::InputText("Path##csv_path", exportCsvPath_, sizeof(exportCsvPath_));
  if (ImGui::Button("Export CSV##csv_btn")) {
    if (sweepResult_.points.empty()) {
      PushLog("CSV export skipped: no sweep result. Run Sweep first.");
    } else {
      std::string err;
      if (model::ExportSweepToCsv(exportCsvPath_, sweepResult_, &err)) {
        if (err.empty()) {
          PushLog(std::string("Exported CSV: ") + exportCsvPath_);
        } else {
          PushLog(err);
        }
      } else {
        PushLog(std::string("CSV export failed: ") + err);
      }
    }
  }

  ImGui::TextUnformatted("JSON Export");
  ImGui::InputText("Path##json_path", exportJsonPath_, sizeof(exportJsonPath_));
  if (ImGui::Button("Export JSON##json_btn")) {
    if (sweepResult_.points.empty()) {
      PushLog("JSON export skipped: no sweep result. Run Sweep first.");
    } else {
      std::string err;
      if (model::ExportSweepToJson(exportJsonPath_, config_, sweepResult_, &err)) {
        PushLog(std::string("Exported JSON: ") + exportJsonPath_);
      } else {
        PushLog(std::string("JSON export failed: ") + err);
      }
    }
  }

  ImGui::TextUnformatted("Header Export");
  ImGui::InputText("Path##header_path", exportHeaderPath_, sizeof(exportHeaderPath_));
  if (ImGui::Button("Export C++ Header##header_btn")) {
    if (sweepResult_.points.empty()) {
      PushLog("Header export skipped: no sweep result. Run Sweep first.");
    } else {
      std::string err;
      if (model::ExportSweepToCppHeader(exportHeaderPath_, sweepResult_, &err)) {
        PushLog(std::string("Exported Header: ") + exportHeaderPath_);
      } else {
        PushLog(std::string("Header export failed: ") + err);
      }
    }
  }

  ImGui::TextWrapped("Tip: use absolute paths if you are unsure about working directory.");

  ImGui::End();
}

void App::DrawLogPanel() const {
  ImGui::Begin("Status", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
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
