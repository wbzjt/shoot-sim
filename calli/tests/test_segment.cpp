#include "TestFramework.h"

#include <vector>

#include "ModelFitter.h"
#include "SegmentAnalyzer.h"

TEST_CASE("SegmentAnalyzer recommends segmentation on bent curve") {
  std::vector<calibration::AggregatedPoint> points;
  for (int i = 0; i < 24; ++i) {
    const double x = 4.0 + 0.3 * static_cast<double>(i);
    calibration::AggregatedPoint p;
    p.theoryVelMps = x;
    p.recommendedMotorRpm = (x < 7.0) ? (1800.0 + 200.0 * x) : (1200.0 + 300.0 * x);
    p.usableForFit = true;
    p.sampleCount = 6;
    p.hitRate = 0.8;
    points.push_back(p);
  }

  calibration::ModelFitter fitter;
  calibration::FitterConfig fitCfg;
  fitCfg.enableSegmented2 = true;
  fitCfg.enableSegmented3 = true;
  fitCfg.enablePolynomial = false;

  std::string warn;
  const auto reports = fitter.Fit(points, fitCfg, &warn);

  calibration::SegmentAnalyzer analyzer;
  calibration::SegmentAnalyzerConfig segCfg;
  segCfg.rmseImproveThresholdRatio = 0.08;

  const auto rec = analyzer.Analyze(calibration::ModelFitter::BuildFitDataset(points), reports, segCfg);

  EXPECT_TRUE(rec.decision == calibration::SegmentationDecision::kRecommend2Segments ||
              rec.decision == calibration::SegmentationDecision::kRecommend3Segments);
}
