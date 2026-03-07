#include "TestFramework.h"

#include <vector>

#include "CalibrationAggregator.h"

TEST_CASE("Aggregator computes stats and outlier removal") {
  std::vector<calibration::CalibrationSample> samples;
  for (int i = 0; i < 12; ++i) {
    calibration::CalibrationSample s;
    s.distanceM = 3.0;
    s.pitchDeg = 52.0;
    s.theoryVelMps = 9.5;
    s.motorTargetRpm = 3200.0 + (i % 2 == 0 ? 0.0 : 20.0);
    s.measuredMotorRpm = 3190.0 + static_cast<double>(i % 3) * 5.0;
    s.hit = (i < 10);
    s.score = 0.8;
    samples.push_back(s);
  }
  // Intentional outlier.
  calibration::CalibrationSample outlier = samples.front();
  outlier.measuredMotorRpm = 5000.0;
  samples.push_back(outlier);

  calibration::AggregationConfig cfg;
  cfg.removeOutliers = true;
  cfg.useMadOutlier = true;
  cfg.minSamplesPerPoint = 3;
  cfg.minHitRateForFit = 0.5;

  calibration::CalibrationAggregator agg;
  std::string warn;
  const auto out = agg.Aggregate(samples, cfg, &warn);

  EXPECT_TRUE(warn.empty());
  EXPECT_TRUE(out.size() == 1U);
  EXPECT_TRUE(out.front().outlierCount >= 1);
  EXPECT_TRUE(out.front().hitRate > 0.5);
  EXPECT_TRUE(out.front().usableForFit);
}
