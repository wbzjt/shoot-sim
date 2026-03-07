#include "SegmentAnalyzer.h"

#include <algorithm>
#include <cmath>

namespace calibration {
namespace {

struct LinearBaselineMetrics {
  double rmse = 0.0;
  double maxError = 0.0;
  bool valid = false;
};

double CompareErrorMetric(const ModelMetrics& metrics) {
  // Lookup/interpolation can overfit training points, so prefer LOOCV if available.
  if (metrics.loocvRmse > 1e-9) {
    return metrics.loocvRmse;
  }
  return metrics.rmse;
}

LinearBaselineMetrics ComputeLinearBaseline(const std::vector<FitDatasetPoint>& dataset) {
  LinearBaselineMetrics out;
  if (dataset.size() < 2U) {
    return out;
  }

  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  for (const auto& p : dataset) {
    sx += p.xTheoryVelMps;
    sy += p.yMotorRpm;
    sxx += p.xTheoryVelMps * p.xTheoryVelMps;
    sxy += p.xTheoryVelMps * p.yMotorRpm;
  }

  const double n = static_cast<double>(dataset.size());
  const double denom = n * sxx - sx * sx;
  if (std::fabs(denom) < 1e-12) {
    return out;
  }

  const double b = (n * sxy - sx * sy) / denom;
  const double a = (sy - b * sx) / n;

  double mse = 0.0;
  double maxErr = 0.0;
  for (const auto& p : dataset) {
    const double pred = a + b * p.xTheoryVelMps;
    const double err = std::fabs(pred - p.yMotorRpm);
    mse += err * err;
    maxErr = std::max(maxErr, err);
  }

  out.rmse = std::sqrt(mse / n);
  out.maxError = maxErr;
  out.valid = true;
  return out;
}

}  // namespace

SegmentRecommendation SegmentAnalyzer::Analyze(
    const std::vector<FitDatasetPoint>& dataset,
    const std::vector<ModelReport>& reports,
    const SegmentAnalyzerConfig& config) const {
  SegmentRecommendation out;

  if (dataset.size() < config.minPointsFor2Segments) {
    out.decision = SegmentationDecision::kInsufficientData;
    out.reasons.push_back("Data points too few for segmentation");
    return out;
  }

  const ModelReport* bestGlobal = nullptr;
  const ModelReport* bestSeg2 = nullptr;
  const ModelReport* bestSeg3 = nullptr;

  for (const auto& r : reports) {
    if (r.kind == ModelKind::kPolynomial2 ||
        r.kind == ModelKind::kPolynomial3 ||
        r.kind == ModelKind::kPolynomial4) {
      if (bestGlobal == nullptr || r.safeScore < bestGlobal->safeScore) {
        bestGlobal = &r;
      }
    }
    if (r.kind == ModelKind::kSegmentedLinear2) {
      if (bestSeg2 == nullptr || r.safeScore < bestSeg2->safeScore) {
        bestSeg2 = &r;
      }
    }
    if (r.kind == ModelKind::kSegmentedLinear3) {
      if (bestSeg3 == nullptr || r.safeScore < bestSeg3->safeScore) {
        bestSeg3 = &r;
      }
    }
  }

  const LinearBaselineMetrics linearBaseline = ComputeLinearBaseline(dataset);

  double globalErrorMetric = 0.0;
  double globalMaxError = 0.0;
  if (bestGlobal != nullptr) {
    globalErrorMetric = CompareErrorMetric(bestGlobal->metrics);
    globalMaxError = bestGlobal->metrics.maxError;
  } else if (linearBaseline.valid) {
    globalErrorMetric = linearBaseline.rmse;
    globalMaxError = linearBaseline.maxError;
    out.reasons.push_back("Using single-line global baseline for segmentation decision");
  } else {
    out.decision = SegmentationDecision::kInsufficientData;
    out.reasons.push_back("No baseline global model available");
    return out;
  }

  if (globalMaxError > config.maxErrorThresholdRpm) {
    out.reasons.push_back("Global model max error exceeds threshold");
  }

  if (bestSeg2 != nullptr) {
    const double seg2Error = CompareErrorMetric(bestSeg2->metrics);
    const double improve = (globalErrorMetric - seg2Error) / std::max(1e-9, globalErrorMetric);
    if (improve >= config.rmseImproveThresholdRatio && bestSeg2->metrics.monotonic) {
      out.decision = SegmentationDecision::kRecommend2Segments;
      out.suggestedBreakpoints = bestSeg2->model.segmentBreakpoints;
      out.reasons.push_back("2-segment model significantly improves generalization error");
      globalErrorMetric = seg2Error;
    }
  }

  if (bestSeg3 != nullptr && dataset.size() >= config.minPointsFor3Segments) {
    const double seg3Error = CompareErrorMetric(bestSeg3->metrics);
    const double improve = (globalErrorMetric - seg3Error) / std::max(1e-9, globalErrorMetric);
    if (improve >= config.rmseImproveThresholdRatio && bestSeg3->metrics.monotonic) {
      out.decision = SegmentationDecision::kRecommend3Segments;
      out.suggestedBreakpoints = bestSeg3->model.segmentBreakpoints;
      out.reasons.push_back("3-segment model adds significant extra improvement");
    }
  }

  if (out.decision == SegmentationDecision::kNoSegmentationNeeded) {
    out.reasons.push_back("Global model error is acceptable; segmentation not required");
  }

  return out;
}

}  // namespace calibration

