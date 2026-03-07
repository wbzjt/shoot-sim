#include "TestFramework.h"

#include <cmath>
#include <vector>

#include "CalibrationPlanner.h"

TEST_CASE("Planner covers boundary points and target count") {
  std::vector<calibration::TheoryPoint> theory;
  for (int i = 0; i < 30; ++i) {
    calibration::TheoryPoint p;
    p.distanceM = 1.0 + 0.2 * static_cast<double>(i);
    p.pitchDeg = 45.0 + 0.6 * static_cast<double>(i);
    p.theoryVelMps = 4.0 + 0.45 * static_cast<double>(i) + 0.03 * static_cast<double>(i * i);
    p.deltaVMps = 0.8 + 0.02 * static_cast<double>(i);
    theory.push_back(p);
  }

  calibration::PlannerConfig cfg;
  cfg.recommendedCount = 9;
  cfg.mode = calibration::PlannerMode::kSensitivityFirst;
  cfg.maxInterpErrorThreshold = -1.0;

  calibration::CalibrationPlanner planner;
  std::string warn;
  calibration::PlannerSummary summary;
  std::vector<calibration::RecommendedPoint> allScores;
  const auto recommended = planner.Recommend(theory, cfg, &summary, &allScores, &warn);

  EXPECT_TRUE(warn.empty());
  EXPECT_TRUE(recommended.size() <= cfg.recommendedCount);
  EXPECT_TRUE(!recommended.empty());
  EXPECT_TRUE(summary.selectedPointCount == recommended.size());
  EXPECT_TRUE(summary.inputPointCount == theory.size());
  EXPECT_TRUE(summary.usedDeltaV);
  EXPECT_TRUE(!allScores.empty());

  bool hasFirst = false;
  bool hasLast = false;
  for (const auto& r : recommended) {
    if (std::fabs(r.point.distanceM - theory.front().distanceM) < 1e-9) {
      hasFirst = true;
    }
    if (std::fabs(r.point.distanceM - theory.back().distanceM) < 1e-9) {
      hasLast = true;
    }
  }
  EXPECT_TRUE(hasFirst);
  EXPECT_TRUE(hasLast);
}

