#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "model/Types.h"

namespace shootersim::app {

class App {
 public:
  App();
  ~App();

  void Render();

 private:
  bool DrawParameterPanel();
  bool DrawSingleSolvePanel();
  void DrawTrajectoryPanel() const;
  void DrawHeatmapPanel() const;
  void DrawSweepPanel();
  void DrawLogPanel() const;

  void SolveSingle();

  void StartSweepAsync();
  void FinalizeSweepIfDone();
  void JoinSweepThread();

  void PushLog(const std::string& message);
  bool RunInternalSelfTest();

 private:
  shootersim::SimulationConfig config_;

  double singleDistance_ = 3.0;
  bool autoSolve_ = true;
  bool solveDirty_ = true;

  shootersim::SolveResult singleResult_;

  shootersim::SweepRequest sweepRequest_;
  shootersim::SweepResult sweepResult_;

  std::atomic<double> sweepProgress_ = 0.0;
  std::atomic<bool> sweepDone_ = false;
  bool sweepRunning_ = false;

  std::thread sweepThread_;
  std::mutex sweepMutex_;
  shootersim::SweepResult sweepThreadResult_;

  char configPath_[260] = {};
  char exportCsvPath_[260] = {};
  char exportJsonPath_[260] = {};
  char exportHeaderPath_[260] = {};

  bool internalSelfTestPassed_ = false;
  std::vector<std::string> logs_;
};

}  // namespace shootersim::app
