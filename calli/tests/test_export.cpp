#include "TestFramework.h"

#include <filesystem>

#include "Exporter.h"

TEST_CASE("Exporter writes csv json md and header") {
  calibration::AnalysisOutput out;
  calibration::TheoryPoint t;
  t.distanceM = 2.0;
  t.pitchDeg = 50.0;
  t.theoryVelMps = 8.0;
  out.theoryPoints.push_back(t);

  calibration::RecommendedPoint rp;
  rp.originalIndex = 0;
  rp.point = t;
  rp.finalScore = 1.0;
  rp.reasonTags = {"ANCHOR_POINT", "EDGE_DISTANCE_MIN"};
  rp.reason = "ANCHOR_POINT|EDGE_DISTANCE_MIN";
  out.recommendedPoints.push_back(rp);

  calibration::AggregatedPoint ap;
  ap.distanceM = 2.0;
  ap.pitchDeg = 50.0;
  ap.theoryVelMps = 8.0;
  ap.recommendedMotorRpm = 3100.0;
  ap.meanMeasuredRpm = 3095.0;
  ap.medianMeasuredRpm = 3090.0;
  ap.stddevMeasuredRpm = 10.0;
  ap.hitRate = 0.9;
  ap.sampleCount = 5;
  ap.usableForFit = true;
  out.aggregatedPoints.push_back(ap);

  calibration::ModelReport mr;
  mr.kind = calibration::ModelKind::kLookupLinear;
  mr.metrics.rmse = 5.0;
  mr.metrics.mae = 4.0;
  mr.metrics.maxError = 9.0;
  mr.metrics.p95Error = 8.0;
  mr.metrics.loocvRmse = 6.0;
  mr.metrics.monotonic = true;
  mr.safeScore = 12.0;
  mr.accuracyScore = 11.0;
  out.modelReports.push_back(mr);
  out.bestSafeModel = mr;
  out.recommendedRuntimeModel = mr;

  calibration::ExportPaths paths;
  paths.outDir = "calli/test_out";

  calibration::Exporter exporter;
  calibration::CalibrationConfig cfg;
  std::string err;
  EXPECT_TRUE(exporter.ExportAll(out, cfg, paths, &err));

  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(paths.outDir) / paths.recommendedPointsCsv));
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(paths.outDir) / paths.aggregatedCsv));
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(paths.outDir) / paths.modelReportCsv));
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(paths.outDir) / paths.analysisJson));
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(paths.outDir) / paths.markdownReport));
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(paths.outDir) / paths.generatedHeader));
}
