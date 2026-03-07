#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace calibration {

enum class PlannerMode {
  kUniformCoverage = 0,
  kSensitivityFirst = 1,
};

enum class PlannerStopReason {
  kReachedTargetCount = 0,
  kInterpolationErrorBelowThreshold = 1,
  kInsufficientCandidatePoints = 2,
  kInputTooSmall = 3,
};

enum class AggregationPreference {
  kHitRateThenMinRpm = 0,
  kHitRateThenScore = 1,
};

enum class ModelKind {
  kLookupLinear = 0,
  kSegmentedLinear2 = 1,
  kSegmentedLinear3 = 2,
  kPolynomial2 = 3,
  kPolynomial3 = 4,
  kPolynomial4 = 5,
};

enum class SegmentationDecision {
  kNoSegmentationNeeded = 0,
  kRecommend2Segments = 1,
  kRecommend3Segments = 2,
  kInsufficientData = 3,
};

struct TheoryPoint {
  double distanceM = 0.0;
  double pitchDeg = 0.0;
  double theoryVelMps = 0.0;
  std::optional<double> vLowMps;
  std::optional<double> vHighMps;
  std::optional<double> deltaVMps;
};

struct CalibrationSample {
  double distanceM = 0.0;
  double pitchDeg = 0.0;
  double theoryVelMps = 0.0;
  double motorTargetRpm = 0.0;
  double measuredMotorRpm = 0.0;
  int shotIndex = -1;
  int laneIndex = 0;
  bool hit = false;
  std::optional<double> score;
  std::optional<double> steadyStateErrorRpm;
  std::optional<double> recoveryTimeMs;
  std::optional<double> batteryV;
  std::string notes;
};

struct RecommendedPoint {
  std::size_t originalIndex = 0;
  TheoryPoint point;
  double finalScore = 0.0;
  bool selected = true;
  std::vector<std::string> reasonTags;
  std::string reason;
};

struct PlannerSummary {
  std::size_t inputPointCount = 0;
  std::size_t selectedPointCount = 0;
  PlannerStopReason stopReason = PlannerStopReason::kInsufficientCandidatePoints;
  double minDistanceM = 0.0;
  double maxDistanceM = 0.0;
  double minTheoryVelMps = 0.0;
  double maxTheoryVelMps = 0.0;
  double maxInterpolationError = 0.0;
  double avgInterpolationError = 0.0;
  bool usedDeltaV = false;
  PlannerMode mode = PlannerMode::kSensitivityFirst;
  std::string stopReasonText;
};

struct AggregatedPoint {
  double distanceM = 0.0;
  double pitchDeg = 0.0;
  double theoryVelMps = 0.0;
  int laneIndex = 0;

  double meanMeasuredRpm = 0.0;
  double medianMeasuredRpm = 0.0;
  double stddevMeasuredRpm = 0.0;
  double hitRate = 0.0;

  double recommendedMotorRpm = 0.0;
  double confidence = 0.0;

  int sampleCount = 0;
  int outlierCount = 0;
  bool usableForFit = false;
  bool uncertain = false;
};

struct FitDatasetPoint {
  double xTheoryVelMps = 0.0;
  double yMotorRpm = 0.0;
};

struct RuntimeModelData {
  ModelKind kind = ModelKind::kLookupLinear;
  std::vector<double> knotsX;
  std::vector<double> knotsY;
  std::vector<double> polyCoefficients;
  std::vector<double> segmentBreakpoints;
  bool clamp = true;
};

struct ModelMetrics {
  double rmse = 0.0;
  double mae = 0.0;
  double maxError = 0.0;
  double p95Error = 0.0;
  double loocvRmse = 0.0;
  bool monotonic = true;
  bool localReverse = false;
  double complexityPenalty = 0.0;
};

struct ModelReport {
  ModelKind kind = ModelKind::kLookupLinear;
  RuntimeModelData model;
  ModelMetrics metrics;
  std::string notes;
  double safeScore = 0.0;
  double accuracyScore = 0.0;
};

struct SegmentRecommendation {
  SegmentationDecision decision = SegmentationDecision::kNoSegmentationNeeded;
  std::vector<double> suggestedBreakpoints;
  std::vector<std::string> reasons;
};

struct PlannerConfig {
  std::size_t recommendedCount = 9;
  PlannerMode mode = PlannerMode::kSensitivityFirst;
  double minVelocitySpacingMps = 0.12;
  double maxInterpErrorThreshold = -1.0;
  bool useDeltaV = true;
  bool verbose = false;
};

struct AggregationConfig {
  AggregationPreference preference = AggregationPreference::kHitRateThenMinRpm;
  bool removeOutliers = true;
  double zScoreThreshold = 2.5;
  bool useMadOutlier = true;
  int minSamplesPerPoint = 3;
  double minHitRateForFit = 0.45;
  bool splitByLane = false;
};

struct FitterConfig {
  bool enablePolynomial = true;
  bool enableSegmented2 = true;
  bool enableSegmented3 = true;
  int loocvMinPoints = 6;
  bool requireMonotonicForRecommended = true;
};

struct SegmentAnalyzerConfig {
  double maxErrorThresholdRpm = 120.0;
  double rmseImproveThresholdRatio = 0.12;
  std::size_t minPointsFor2Segments = 8;
  std::size_t minPointsFor3Segments = 14;
};

struct CalibrationConfig {
  PlannerConfig planner;
  AggregationConfig aggregation;
  FitterConfig fitter;
  SegmentAnalyzerConfig segment;
};

struct AnalysisOutput {
  std::vector<TheoryPoint> theoryPoints;
  std::vector<RecommendedPoint> recommendedPoints;
  std::vector<RecommendedPoint> plannerAllScores;
  std::optional<PlannerSummary> plannerSummary;
  std::vector<CalibrationSample> rawSamples;
  std::vector<AggregatedPoint> aggregatedPoints;
  std::vector<ModelReport> modelReports;
  SegmentRecommendation segmentRecommendation;
  std::optional<ModelReport> bestSafeModel;
  std::optional<ModelReport> bestAccuracyModel;
  std::optional<ModelReport> recommendedRuntimeModel;
};

struct ExportPaths {
  std::string outDir;
  std::string recommendedPointsCsv = "recommended_calibration_points.csv";
  std::string allScoresCsv = "all_scores.csv";
  std::string plannerSummaryJson = "planner_summary.json";
  std::string aggregatedCsv = "aggregated_calibration_points.csv";
  std::string modelReportCsv = "model_report.csv";
  std::string fitCurveCsv = "fit_curve.csv";
  std::string fitResidualCsv = "fit_residuals.csv";
  std::string analysisJson = "calibration_result.json";
  std::string markdownReport = "calibration_report.md";
  std::string generatedHeader = "GeneratedShooterCalibration.h";
};

}  // namespace calibration
