#include "TestFramework.h"

#include <vector>

#include "ModelFitter.h"

TEST_CASE("ModelFitter runs candidate models and picks best") {
  std::vector<calibration::AggregatedPoint> points;
  for (int i = 0; i < 18; ++i) {
    calibration::AggregatedPoint p;
    p.theoryVelMps = 4.5 + 0.4 * static_cast<double>(i);
    p.recommendedMotorRpm = 1800.0 + 240.0 * p.theoryVelMps;
    p.usableForFit = true;
    p.hitRate = 0.8;
    p.sampleCount = 6;
    points.push_back(p);
  }

  calibration::FitterConfig cfg;
  cfg.enablePolynomial = true;
  cfg.enableSegmented2 = true;
  cfg.enableSegmented3 = true;

  calibration::ModelFitter fitter;
  std::string warn;
  const auto reports = fitter.Fit(points, cfg, &warn);

  EXPECT_TRUE(warn.empty());
  EXPECT_TRUE(!reports.empty());

  const auto bestSafe = calibration::ModelFitter::PickBestSafe(reports);
  const auto bestAcc = calibration::ModelFitter::PickBestAccuracy(reports);
  EXPECT_TRUE(bestSafe.has_value());
  EXPECT_TRUE(bestAcc.has_value());
  EXPECT_TRUE(bestSafe->metrics.monotonic);
}
