#include "Exporter.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include "CsvUtil.h"
#include "MathUtil.h"
#include "RuntimeModel.h"

namespace calibration {
namespace {

std::string JoinPath(const std::string& dir, const std::string& file) {
  return (std::filesystem::path(dir) / std::filesystem::path(file)).string();
}

std::string ModelKindName(ModelKind kind) {
  switch (kind) {
    case ModelKind::kLookupLinear:
      return "lookup_linear";
    case ModelKind::kSegmentedLinear2:
      return "segmented_linear_2";
    case ModelKind::kSegmentedLinear3:
      return "segmented_linear_3";
    case ModelKind::kPolynomial2:
      return "polynomial_2";
    case ModelKind::kPolynomial3:
      return "polynomial_3";
    case ModelKind::kPolynomial4:
      return "polynomial_4";
  }
  return "unknown";
}

std::string SegDecisionName(SegmentationDecision d) {
  switch (d) {
    case SegmentationDecision::kNoSegmentationNeeded:
      return "no segmentation needed";
    case SegmentationDecision::kRecommend2Segments:
      return "recommend 2 segments";
    case SegmentationDecision::kRecommend3Segments:
      return "recommend 3 segments";
    case SegmentationDecision::kInsufficientData:
      return "insufficient data";
  }
  return "unknown";
}

std::string PlannerStopReasonName(PlannerStopReason r) {
  switch (r) {
    case PlannerStopReason::kReachedTargetCount:
      return "reached target count";
    case PlannerStopReason::kInterpolationErrorBelowThreshold:
      return "interpolation error below threshold";
    case PlannerStopReason::kInsufficientCandidatePoints:
      return "insufficient candidate points";
    case PlannerStopReason::kInputTooSmall:
      return "input points too small";
  }
  return "unknown";
}

std::vector<FitDatasetPoint> BuildLookupPoints(const AnalysisOutput& output) {
  std::vector<FitDatasetPoint> points;
  points.reserve(output.aggregatedPoints.size());
  for (const auto& p : output.aggregatedPoints) {
    if (p.usableForFit) {
      points.push_back({p.theoryVelMps, p.recommendedMotorRpm});
    }
  }
  std::sort(points.begin(), points.end(), [](const FitDatasetPoint& a, const FitDatasetPoint& b) {
    return a.xTheoryVelMps < b.xTheoryVelMps;
  });
  return points;
}

bool WriteTextFile(const std::string& path, const std::string& content, std::string* error) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open output file: " + path;
    }
    return false;
  }
  file << content;
  if (!file.good()) {
    if (error != nullptr) {
      *error = "Failed to write output file: " + path;
    }
    return false;
  }
  return true;
}

std::string EscapeJson(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8U);
  for (char c : s) {
    if (c == '"') {
      out += "\\\"";
    } else if (c == '\\') {
      out += "\\\\";
    } else if (c == '\n') {
      out += "\\n";
    } else {
      out.push_back(c);
    }
  }
  return out;
}

}  // namespace

bool Exporter::ExportAll(const AnalysisOutput& output,
                         const CalibrationConfig& config,
                         const ExportPaths& paths,
                         std::string* error) const {
  try {
    std::filesystem::create_directories(paths.outDir);
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }

  if (!ExportRecommendedPointsCsv(output,
                                  JoinPath(paths.outDir, paths.recommendedPointsCsv),
                                  error)) {
    return false;
  }
  if (!output.plannerAllScores.empty()) {
    if (!ExportAllScoresCsv(output, JoinPath(paths.outDir, paths.allScoresCsv), error)) {
      return false;
    }
  }
  if (output.plannerSummary.has_value()) {
    if (!ExportPlannerSummaryJson(output, JoinPath(paths.outDir, paths.plannerSummaryJson), error)) {
      return false;
    }
  }
  if (!ExportAggregatedCsv(output, JoinPath(paths.outDir, paths.aggregatedCsv), error)) {
    return false;
  }
  if (!ExportModelReportCsv(output, JoinPath(paths.outDir, paths.modelReportCsv), error)) {
    return false;
  }
  if (!output.aggregatedPoints.empty() && output.recommendedRuntimeModel.has_value()) {
    if (!ExportFitCurveCsv(output, JoinPath(paths.outDir, paths.fitCurveCsv), error)) {
      return false;
    }
    if (!ExportFitResidualCsv(output, JoinPath(paths.outDir, paths.fitResidualCsv), error)) {
      return false;
    }
  }
  if (!ExportJson(output, config, JoinPath(paths.outDir, paths.analysisJson), error)) {
    return false;
  }
  if (!ExportMarkdown(output, JoinPath(paths.outDir, paths.markdownReport), error)) {
    return false;
  }

  bool hasUsablePoints = false;
  for (const auto& p : output.aggregatedPoints) {
    if (p.usableForFit) {
      hasUsablePoints = true;
      break;
    }
  }
  if (hasUsablePoints) {
    if (!ExportHeader(output, JoinPath(paths.outDir, paths.generatedHeader), error)) {
      return false;
    }
  }

  return true;
}

bool Exporter::ExportRecommendedPointsCsv(const AnalysisOutput& output,
                                          const std::string& path,
                                          std::string* error) const {
  std::vector<std::vector<std::string>> rows;
  rows.reserve(output.recommendedPoints.size());
  for (const auto& r : output.recommendedPoints) {
    const std::string delta = r.point.deltaVMps.has_value() ? std::to_string(*r.point.deltaVMps) : "";
    rows.push_back({
        std::to_string(r.point.distanceM),
        std::to_string(r.point.pitchDeg),
        std::to_string(r.point.theoryVelMps),
        "",   // motor_target_rpm (template for stage-2 field filling)
        "",   // measured_motor_rpm (optional template)
        "",   // hit (optional template)
        "",   // shot_index
        "",   // lane_index
        "",   // notes
        delta,
        std::to_string(r.finalScore),
        r.reason,
        std::to_string(r.originalIndex),
    });
  }
  return csv::WriteCsv(path,
                       {"distance_m", "pitch_deg", "theory_vel_mps", "motor_target_rpm",
                        "measured_motor_rpm", "hit", "shot_index", "lane_index", "notes",
                        "delta_v_mps", "final_score", "reason_tags", "original_index"},
                       rows,
                       error);
}

bool Exporter::ExportAllScoresCsv(const AnalysisOutput& output,
                                  const std::string& path,
                                  std::string* error) const {
  std::vector<std::vector<std::string>> rows;
  rows.reserve(output.plannerAllScores.size());
  for (const auto& r : output.plannerAllScores) {
    const std::string delta = r.point.deltaVMps.has_value() ? std::to_string(*r.point.deltaVMps) : "";
    rows.push_back({
        std::to_string(r.originalIndex),
        std::to_string(r.point.distanceM),
        std::to_string(r.point.pitchDeg),
        std::to_string(r.point.theoryVelMps),
        delta,
        std::to_string(r.finalScore),
        r.selected ? "1" : "0",
        r.reason,
    });
  }
  return csv::WriteCsv(path,
                       {"original_index", "distance_m", "pitch_deg", "theory_vel_mps",
                        "delta_v_mps", "final_score", "selected", "reason_tags"},
                       rows,
                       error);
}

bool Exporter::ExportPlannerSummaryJson(const AnalysisOutput& output,
                                        const std::string& path,
                                        std::string* error) const {
  if (!output.plannerSummary.has_value()) {
    return true;
  }
  const auto& s = *output.plannerSummary;
  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"inputPointCount\": " << s.inputPointCount << ",\n";
  oss << "  \"selectedPointCount\": " << s.selectedPointCount << ",\n";
  oss << "  \"stopReason\": \"" << EscapeJson(PlannerStopReasonName(s.stopReason)) << "\",\n";
  oss << "  \"mode\": \"" << (s.mode == PlannerMode::kUniformCoverage ? "uniform" : "sensitivity") << "\",\n";
  oss << "  \"usedDeltaV\": " << (s.usedDeltaV ? "true" : "false") << ",\n";
  oss << "  \"minDistanceM\": " << s.minDistanceM << ",\n";
  oss << "  \"maxDistanceM\": " << s.maxDistanceM << ",\n";
  oss << "  \"minTheoryVelMps\": " << s.minTheoryVelMps << ",\n";
  oss << "  \"maxTheoryVelMps\": " << s.maxTheoryVelMps << ",\n";
  oss << "  \"maxInterpolationError\": " << s.maxInterpolationError << ",\n";
  oss << "  \"avgInterpolationError\": " << s.avgInterpolationError << "\n";
  oss << "}\n";
  return WriteTextFile(path, oss.str(), error);
}

bool Exporter::ExportAggregatedCsv(const AnalysisOutput& output,
                                   const std::string& path,
                                   std::string* error) const {
  std::vector<std::vector<std::string>> rows;
  rows.reserve(output.aggregatedPoints.size());
  for (const auto& p : output.aggregatedPoints) {
    rows.push_back({
        std::to_string(p.distanceM),
        std::to_string(p.pitchDeg),
        std::to_string(p.theoryVelMps),
        std::to_string(p.laneIndex),
        std::to_string(p.recommendedMotorRpm),
        std::to_string(p.hitRate),
        std::to_string(p.stddevMeasuredRpm),
        std::to_string(p.confidence),
        std::to_string(p.sampleCount),
        std::to_string(p.outlierCount),
        p.usableForFit ? "1" : "0",
        p.uncertain ? "1" : "0",
    });
  }

  return csv::WriteCsv(path,
                       {
                           "distance_m", "pitch_deg", "theory_vel_mps", "lane_index",
                           "recommended_motor_rpm", "hit_rate", "stddev_rpm", "confidence",
                           "sample_count", "outlier_count", "usable_for_fit", "uncertain",
                       },
                       rows,
                       error);
}

bool Exporter::ExportModelReportCsv(const AnalysisOutput& output,
                                    const std::string& path,
                                    std::string* error) const {
  std::vector<std::vector<std::string>> rows;
  rows.reserve(output.modelReports.size());
  for (const auto& r : output.modelReports) {
    rows.push_back({
        ModelKindName(r.kind),
        std::to_string(r.metrics.rmse),
        std::to_string(r.metrics.mae),
        std::to_string(r.metrics.maxError),
        std::to_string(r.metrics.p95Error),
        std::to_string(r.metrics.loocvRmse),
        r.metrics.monotonic ? "1" : "0",
        r.metrics.localReverse ? "1" : "0",
        std::to_string(r.metrics.complexityPenalty),
        std::to_string(r.safeScore),
        std::to_string(r.accuracyScore),
        r.notes,
    });
  }

  return csv::WriteCsv(path,
                       {
                           "model", "rmse", "mae", "max_error", "p95_error", "loocv_rmse",
                           "monotonic", "local_reverse", "complexity_penalty", "safe_score",
                           "accuracy_score", "notes",
                       },
                       rows,
                       error);
}

bool Exporter::ExportFitCurveCsv(const AnalysisOutput& output,
                                 const std::string& path,
                                 std::string* error) const {
  if (!output.recommendedRuntimeModel.has_value()) {
    return true;
  }
  double xMin = std::numeric_limits<double>::infinity();
  double xMax = -std::numeric_limits<double>::infinity();
  for (const auto& p : output.aggregatedPoints) {
    if (!p.usableForFit) {
      continue;
    }
    xMin = std::min(xMin, p.theoryVelMps);
    xMax = std::max(xMax, p.theoryVelMps);
  }
  if (!std::isfinite(xMin) || !std::isfinite(xMax) || xMax <= xMin) {
    return true;
  }

  std::vector<std::vector<std::string>> rows;
  constexpr int kSamples = 200;
  rows.reserve(static_cast<std::size_t>(kSamples));
  for (int i = 0; i < kSamples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);
    const double x = math::Lerp(xMin, xMax, t);
    const double y = runtime::EvaluateModel(output.recommendedRuntimeModel->model, x);
    rows.push_back({std::to_string(x), std::to_string(y)});
  }

  return csv::WriteCsv(path, {"theory_vel_mps", "predicted_motor_rpm"}, rows, error);
}

bool Exporter::ExportFitResidualCsv(const AnalysisOutput& output,
                                    const std::string& path,
                                    std::string* error) const {
  if (!output.recommendedRuntimeModel.has_value()) {
    return true;
  }
  std::vector<std::vector<std::string>> rows;
  rows.reserve(output.aggregatedPoints.size());
  for (const auto& p : output.aggregatedPoints) {
    if (!p.usableForFit) {
      continue;
    }
    const double pred = runtime::EvaluateModel(output.recommendedRuntimeModel->model, p.theoryVelMps);
    const double residual = pred - p.recommendedMotorRpm;
    rows.push_back({
        std::to_string(p.distanceM),
        std::to_string(p.pitchDeg),
        std::to_string(p.theoryVelMps),
        std::to_string(p.recommendedMotorRpm),
        std::to_string(pred),
        std::to_string(residual),
    });
  }
  return csv::WriteCsv(path,
                       {"distance_m", "pitch_deg", "theory_vel_mps", "observed_motor_rpm",
                        "predicted_motor_rpm", "residual_rpm"},
                       rows,
                       error);
}

bool Exporter::ExportJson(const AnalysisOutput& output,
                          const CalibrationConfig& config,
                          const std::string& path,
                          std::string* error) const {
  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"config\": {\n";
  oss << "    \"plannerCount\": " << config.planner.recommendedCount << ",\n";
  oss << "    \"plannerMode\": \"" << (config.planner.mode == PlannerMode::kUniformCoverage ? "uniform" : "sensitivity") << "\",\n";
  oss << "    \"minVelocitySpacingMps\": " << config.planner.minVelocitySpacingMps << ",\n";
  oss << "    \"maxInterpErrorThreshold\": " << config.planner.maxInterpErrorThreshold << ",\n";
  oss << "    \"useDeltaV\": " << (config.planner.useDeltaV ? "true" : "false") << "\n";
  oss << "  },\n";

  if (output.plannerSummary.has_value()) {
    const auto& s = *output.plannerSummary;
    oss << "  \"plannerSummary\": {\n";
    oss << "    \"inputPointCount\": " << s.inputPointCount << ",\n";
    oss << "    \"selectedPointCount\": " << s.selectedPointCount << ",\n";
    oss << "    \"stopReason\": \"" << EscapeJson(PlannerStopReasonName(s.stopReason)) << "\",\n";
    oss << "    \"maxInterpolationError\": " << s.maxInterpolationError << ",\n";
    oss << "    \"avgInterpolationError\": " << s.avgInterpolationError << ",\n";
    oss << "    \"usedDeltaV\": " << (s.usedDeltaV ? "true" : "false") << "\n";
    oss << "  },\n";
  }

  oss << "  \"modelReports\": [\n";
  for (std::size_t i = 0; i < output.modelReports.size(); ++i) {
    const auto& r = output.modelReports[i];
    oss << "    {\n";
    oss << "      \"model\": \"" << ModelKindName(r.kind) << "\",\n";
    oss << "      \"rmse\": " << r.metrics.rmse << ",\n";
    oss << "      \"mae\": " << r.metrics.mae << ",\n";
    oss << "      \"maxError\": " << r.metrics.maxError << ",\n";
    oss << "      \"p95Error\": " << r.metrics.p95Error << ",\n";
    oss << "      \"loocvRmse\": " << r.metrics.loocvRmse << ",\n";
    oss << "      \"monotonic\": " << (r.metrics.monotonic ? "true" : "false") << ",\n";
    oss << "      \"safeScore\": " << r.safeScore << ",\n";
    oss << "      \"accuracyScore\": " << r.accuracyScore << "\n";
    oss << "    }" << (i + 1U == output.modelReports.size() ? "\n" : ",\n");
  }
  oss << "  ],\n";

  oss << "  \"segmentRecommendation\": {\n";
  oss << "    \"decision\": \"" << SegDecisionName(output.segmentRecommendation.decision) << "\",\n";
  oss << "    \"breakpoints\": [";
  for (std::size_t i = 0; i < output.segmentRecommendation.suggestedBreakpoints.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << output.segmentRecommendation.suggestedBreakpoints[i];
  }
  oss << "],\n";
  oss << "    \"reasons\": [";
  for (std::size_t i = 0; i < output.segmentRecommendation.reasons.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << "\"" << EscapeJson(output.segmentRecommendation.reasons[i]) << "\"";
  }
  oss << "]\n";
  oss << "  }\n";

  oss << "}\n";

  return WriteTextFile(path, oss.str(), error);
}

bool Exporter::ExportMarkdown(const AnalysisOutput& output,
                              const std::string& path,
                              std::string* error) const {
  std::ostringstream oss;
  oss << "# Shooter Calibration Report\n\n";
  oss << "## Theory Summary\n\n";
  oss << "- Theory points: " << output.theoryPoints.size() << "\n";
  oss << "- Recommended calibration points: " << output.recommendedPoints.size() << "\n\n";
  if (output.plannerSummary.has_value()) {
    const auto& s = *output.plannerSummary;
    oss << "### Planner Selection Summary\n\n";
    oss << "- Stop reason: " << PlannerStopReasonName(s.stopReason) << "\n";
    oss << "- Distance coverage (m): " << s.minDistanceM << " ~ " << s.maxDistanceM << "\n";
    oss << "- Theory velocity coverage (m/s): " << s.minTheoryVelMps << " ~ " << s.maxTheoryVelMps << "\n";
    oss << "- Max interpolation error (normalized): " << s.maxInterpolationError << "\n";
    oss << "- Avg interpolation error (normalized): " << s.avgInterpolationError << "\n";
    oss << "- Used delta_v weighting: " << (s.usedDeltaV ? "yes" : "no") << "\n\n";
  }

  oss << "## Field Calibration Summary\n\n";
  oss << "- Raw samples: " << output.rawSamples.size() << "\n";
  oss << "- Aggregated points: " << output.aggregatedPoints.size() << "\n\n";

  oss << "## Model Comparison\n\n";
  oss << "| Model | RMSE | MAE | Max Error | P95 | LOOCV RMSE | Monotonic |\n";
  oss << "|---|---:|---:|---:|---:|---:|---|\n";
  for (const auto& r : output.modelReports) {
    oss << "| " << ModelKindName(r.kind)
        << " | " << std::fixed << std::setprecision(3) << r.metrics.rmse
        << " | " << r.metrics.mae
        << " | " << r.metrics.maxError
        << " | " << r.metrics.p95Error
        << " | " << r.metrics.loocvRmse
        << " | " << (r.metrics.monotonic ? "yes" : "no") << " |\n";
  }
  oss << "\n";

  oss << "## Segmentation Recommendation\n\n";
  oss << "- Decision: " << SegDecisionName(output.segmentRecommendation.decision) << "\n";
  if (!output.segmentRecommendation.suggestedBreakpoints.empty()) {
    oss << "- Suggested breakpoints (theory vel m/s): ";
    for (std::size_t i = 0; i < output.segmentRecommendation.suggestedBreakpoints.size(); ++i) {
      if (i > 0U) {
        oss << ", ";
      }
      oss << output.segmentRecommendation.suggestedBreakpoints[i];
    }
    oss << "\n";
  }
  for (const auto& reason : output.segmentRecommendation.reasons) {
    oss << "- " << reason << "\n";
  }

  oss << "\n## Final Recommendation\n\n";
  if (output.recommendedRuntimeModel.has_value()) {
    oss << "- Recommended robot runtime model: `"
        << ModelKindName(output.recommendedRuntimeModel->kind) << "`\n";
  } else {
    oss << "- Recommended robot runtime model: not available\n";
  }

  return WriteTextFile(path, oss.str(), error);
}

bool Exporter::ExportHeader(const AnalysisOutput& output,
                            const std::string& path,
                            std::string* error) const {
  const std::vector<FitDatasetPoint> lookup = BuildLookupPoints(output);
  if (lookup.empty()) {
    if (error != nullptr) {
      *error = "No usable calibration points for header export";
    }
    return false;
  }

  std::optional<ModelReport> runtimeModel = output.recommendedRuntimeModel;
  if (!runtimeModel.has_value() && output.bestSafeModel.has_value()) {
    runtimeModel = output.bestSafeModel;
  }

  std::ostringstream oss;
  oss << "#pragma once\n\n";
  oss << "#include <array>\n";
  oss << "#include <algorithm>\n\n";
  oss << "namespace shooter_calibration_generated {\n\n";

  oss << "struct CalibrationPoint { double theoryVelMps; double motorRpm; };\n\n";

  oss << "constexpr std::array<CalibrationPoint, " << lookup.size() << "> kLookup = {{\n";
  for (const auto& p : lookup) {
    oss << "  CalibrationPoint{" << std::fixed << std::setprecision(6)
        << p.xTheoryVelMps << ", " << p.yMotorRpm << "},\n";
  }
  oss << "}};\n\n";

  oss << "inline double LookupLinear(double theoryVelMps) {\n";
  oss << "  if (theoryVelMps <= kLookup.front().theoryVelMps) return kLookup.front().motorRpm;\n";
  oss << "  if (theoryVelMps >= kLookup.back().theoryVelMps) return kLookup.back().motorRpm;\n";
  oss << "  for (std::size_t i = 1; i < kLookup.size(); ++i) {\n";
  oss << "    if (theoryVelMps <= kLookup[i].theoryVelMps) {\n";
  oss << "      const auto& a = kLookup[i - 1];\n";
  oss << "      const auto& b = kLookup[i];\n";
  oss << "      const double t = (theoryVelMps - a.theoryVelMps) / (b.theoryVelMps - a.theoryVelMps);\n";
  oss << "      return a.motorRpm + (b.motorRpm - a.motorRpm) * t;\n";
  oss << "    }\n";
  oss << "  }\n";
  oss << "  return kLookup.back().motorRpm;\n";
  oss << "}\n\n";

  if (runtimeModel.has_value()) {
    oss << "// Recommended runtime model: " << ModelKindName(runtimeModel->kind) << "\n";
    if (runtimeModel->kind == ModelKind::kPolynomial2 ||
        runtimeModel->kind == ModelKind::kPolynomial3 ||
        runtimeModel->kind == ModelKind::kPolynomial4) {
      oss << "constexpr std::array<double, " << runtimeModel->model.polyCoefficients.size()
          << "> kPoly = {";
      for (std::size_t i = 0; i < runtimeModel->model.polyCoefficients.size(); ++i) {
        if (i > 0U) {
          oss << ", ";
        }
        oss << runtimeModel->model.polyCoefficients[i];
      }
      oss << "};\n\n";
      oss << "inline double RecommendedFit(double theoryVelMps) {\n";
      oss << "  double y = 0.0;\n";
      oss << "  double p = 1.0;\n";
      oss << "  for (double c : kPoly) { y += c * p; p *= theoryVelMps; }\n";
      oss << "  return y;\n";
      oss << "}\n\n";
    } else {
      oss << "inline double RecommendedFit(double theoryVelMps) {\n";
      oss << "  return LookupLinear(theoryVelMps);\n";
      oss << "}\n\n";
    }
  } else {
    oss << "inline double RecommendedFit(double theoryVelMps) { return LookupLinear(theoryVelMps); }\n\n";
  }

  oss << "inline double GetMotorRpmForTheoreticalVelocity(double theoryVelMps) {\n";
  oss << "  return LookupLinear(theoryVelMps);\n";
  oss << "}\n\n";

  oss << "inline double GetMotorRpmForTheoreticalVelocitySegmented(double theoryVelMps) {\n";
  oss << "  return RecommendedFit(theoryVelMps);\n";
  oss << "}\n\n";

  oss << "}  // namespace shooter_calibration_generated\n";

  return WriteTextFile(path, oss.str(), error);
}

}  // namespace calibration
