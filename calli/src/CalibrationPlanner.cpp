#include "CalibrationPlanner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>

#include "MathUtil.h"

namespace calibration {
namespace {

struct PlannerWeights {
  double wFit = 0.45;
  double wSpace = 0.20;
  double wCurv = 0.20;
  double wSens = 0.10;
  double wEdge = 0.05;
  double alpha = 0.70;
  double beta = 0.70;
  double gamma = 0.70;
  double edgeBonus = 1.0;
};

struct PreparedPoint {
  TheoryPoint point;
  std::size_t originalIndex = 0;
  bool edgeDistanceMin = false;
  bool edgeDistanceMax = false;
  bool edgeVelocityMin = false;
  bool edgeVelocityMax = false;
  double curvatureVel = 0.0;
  double curvaturePitch = 0.0;
  double sensitivity = 0.0;
};

struct EvalTerm {
  double fit = 0.0;
  double space = 0.0;
  double curv = 0.0;
  double sens = 0.0;
  double edge = 0.0;
  double score = 0.0;
};

struct ErrorStats {
  double maxError = 0.0;
  double avgError = 0.0;
};

std::string JoinTags(const std::vector<std::string>& tags) {
  if (tags.empty()) {
    return "";
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < tags.size(); ++i) {
    if (i > 0U) {
      oss << '|';
    }
    oss << tags[i];
  }
  return oss.str();
}

PlannerWeights BuildWeights(PlannerMode mode) {
  PlannerWeights w;
  if (mode == PlannerMode::kUniformCoverage) {
    w.wFit = 0.30;
    w.wSpace = 0.45;
    w.wCurv = 0.15;
    w.wSens = 0.05;
    w.wEdge = 0.05;
  } else {
    w.wFit = 0.45;
    w.wSpace = 0.20;
    w.wCurv = 0.20;
    w.wSens = 0.10;
    w.wEdge = 0.05;
  }
  return w;
}

double SafeRange(double lo, double hi) {
  return std::max(1e-9, hi - lo);
}

double CurvatureSecondDiff(double x0, double x1, double x2, double y0, double y1, double y2) {
  const double h0 = x1 - x0;
  const double h1 = x2 - x1;
  const double h = x2 - x0;
  if (std::fabs(h0) < 1e-9 || std::fabs(h1) < 1e-9 || std::fabs(h) < 1e-9) {
    return 0.0;
  }
  const double d0 = (y1 - y0) / h0;
  const double d1 = (y2 - y1) / h1;
  return std::fabs(2.0 * (d1 - d0) / h);
}

std::vector<PreparedPoint> Preprocess(const std::vector<TheoryPoint>& theory, bool* hasAnyDelta) {
  struct RawPoint {
    std::size_t index = 0;
    TheoryPoint p;
  };

  std::vector<RawPoint> valid;
  valid.reserve(theory.size());
  for (std::size_t i = 0; i < theory.size(); ++i) {
    const auto& p = theory[i];
    if (!std::isfinite(p.distanceM) || !std::isfinite(p.pitchDeg) || !std::isfinite(p.theoryVelMps)) {
      continue;
    }
    valid.push_back({i, p});
  }

  std::sort(valid.begin(), valid.end(), [](const RawPoint& a, const RawPoint& b) {
    if (a.p.distanceM == b.p.distanceM) {
      return a.index < b.index;
    }
    return a.p.distanceM < b.p.distanceM;
  });

  std::vector<PreparedPoint> out;
  out.reserve(valid.size());
  bool useDelta = false;

  for (std::size_t i = 0; i < valid.size(); ++i) {
    if (out.empty() || std::fabs(valid[i].p.distanceM - out.back().point.distanceM) > 1e-9) {
      PreparedPoint pp;
      pp.originalIndex = valid[i].index;
      pp.point = valid[i].p;
      out.push_back(pp);
      if (valid[i].p.deltaVMps.has_value()) {
        useDelta = true;
      }
      continue;
    }

    // Merge duplicate distance points by averaging pitch / theory velocity / delta-v.
    auto& m = out.back();
    const double n = static_cast<double>(i + 1U);
    m.point.pitchDeg = math::Lerp(m.point.pitchDeg, valid[i].p.pitchDeg, 1.0 / n);
    m.point.theoryVelMps = math::Lerp(m.point.theoryVelMps, valid[i].p.theoryVelMps, 1.0 / n);
    if (m.point.deltaVMps.has_value() && valid[i].p.deltaVMps.has_value()) {
      m.point.deltaVMps = math::Lerp(*m.point.deltaVMps, *valid[i].p.deltaVMps, 1.0 / n);
      useDelta = true;
    } else if (!m.point.deltaVMps.has_value() && valid[i].p.deltaVMps.has_value()) {
      m.point.deltaVMps = valid[i].p.deltaVMps;
      useDelta = true;
    }
  }

  if (hasAnyDelta != nullptr) {
    *hasAnyDelta = useDelta;
  }
  return out;
}

void MarkEdgesAndFeatures(std::vector<PreparedPoint>* points, bool useDelta) {
  if (points == nullptr || points->empty()) {
    return;
  }
  auto& p = *points;

  p.front().edgeDistanceMin = true;
  p.back().edgeDistanceMax = true;

  std::size_t velMinIdx = 0U;
  std::size_t velMaxIdx = 0U;
  for (std::size_t i = 1U; i < p.size(); ++i) {
    if (p[i].point.theoryVelMps < p[velMinIdx].point.theoryVelMps) {
      velMinIdx = i;
    }
    if (p[i].point.theoryVelMps > p[velMaxIdx].point.theoryVelMps) {
      velMaxIdx = i;
    }
  }
  p[velMinIdx].edgeVelocityMin = true;
  p[velMaxIdx].edgeVelocityMax = true;

  std::vector<double> curvVelRaw(p.size(), 0.0);
  std::vector<double> curvPitchRaw(p.size(), 0.0);
  for (std::size_t i = 1U; i + 1U < p.size(); ++i) {
    curvVelRaw[i] = CurvatureSecondDiff(
        p[i - 1U].point.distanceM, p[i].point.distanceM, p[i + 1U].point.distanceM,
        p[i - 1U].point.theoryVelMps, p[i].point.theoryVelMps, p[i + 1U].point.theoryVelMps);
    curvPitchRaw[i] = CurvatureSecondDiff(
        p[i - 1U].point.distanceM, p[i].point.distanceM, p[i + 1U].point.distanceM,
        p[i - 1U].point.pitchDeg, p[i].point.pitchDeg, p[i + 1U].point.pitchDeg);
  }
  const double curvVelMax = std::max(1e-9, *std::max_element(curvVelRaw.begin(), curvVelRaw.end()));
  const double curvPitchMax = std::max(1e-9, *std::max_element(curvPitchRaw.begin(), curvPitchRaw.end()));
  for (std::size_t i = 0; i < p.size(); ++i) {
    p[i].curvatureVel = curvVelRaw[i] / curvVelMax;
    p[i].curvaturePitch = curvPitchRaw[i] / curvPitchMax;
  }

  if (!useDelta) {
    return;
  }

  double deltaMin = std::numeric_limits<double>::infinity();
  double deltaMax = -std::numeric_limits<double>::infinity();
  for (const auto& x : p) {
    if (!x.point.deltaVMps.has_value()) {
      continue;
    }
    deltaMin = std::min(deltaMin, *x.point.deltaVMps);
    deltaMax = std::max(deltaMax, *x.point.deltaVMps);
  }
  if (!std::isfinite(deltaMin) || !std::isfinite(deltaMax)) {
    return;
  }
  const double deltaRange = SafeRange(deltaMin, deltaMax);
  for (auto& x : p) {
    if (!x.point.deltaVMps.has_value()) {
      x.sensitivity = 0.0;
      continue;
    }
    const double normalized = (*x.point.deltaVMps - deltaMin) / deltaRange;
    x.sensitivity = 1.0 - math::Clamp(normalized, 0.0, 1.0);
  }
}

void BuildSelectedInterpolator(const std::vector<PreparedPoint>& points,
                               const std::vector<std::size_t>& selected,
                               std::vector<double>* xs,
                               std::vector<double>* velYs,
                               std::vector<double>* pitchYs) {
  xs->clear();
  velYs->clear();
  pitchYs->clear();
  xs->reserve(selected.size());
  velYs->reserve(selected.size());
  pitchYs->reserve(selected.size());
  for (std::size_t idx : selected) {
    xs->push_back(points[idx].point.distanceM);
    velYs->push_back(points[idx].point.theoryVelMps);
    pitchYs->push_back(points[idx].point.pitchDeg);
  }
}

double InterpErrorAtIndex(const std::vector<PreparedPoint>& points,
                          std::size_t i,
                          const std::vector<double>& selDist,
                          const std::vector<double>& selVel,
                          const std::vector<double>& selPitch,
                          double velRange,
                          double pitchRange,
                          double alpha) {
  if (selDist.empty()) {
    return 1.0;
  }
  const double d = points[i].point.distanceM;
  const double vHat = math::LinearInterpolate(selDist, selVel, d, true);
  const double pHat = math::LinearInterpolate(selDist, selPitch, d, true);
  const double vErr = std::fabs(points[i].point.theoryVelMps - vHat) / velRange;
  const double pErr = std::fabs(points[i].point.pitchDeg - pHat) / pitchRange;
  return alpha * vErr + (1.0 - alpha) * pErr;
}

ErrorStats ComputeErrorStats(const std::vector<PreparedPoint>& points,
                             const std::vector<std::size_t>& selected,
                             double velRange,
                             double pitchRange,
                             double alpha) {
  std::vector<double> d;
  std::vector<double> v;
  std::vector<double> p;
  BuildSelectedInterpolator(points, selected, &d, &v, &p);
  ErrorStats out;
  if (points.empty()) {
    return out;
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double err = InterpErrorAtIndex(points, i, d, v, p, velRange, pitchRange, alpha);
    out.maxError = std::max(out.maxError, err);
    sum += err;
  }
  out.avgError = sum / static_cast<double>(points.size());
  return out;
}

EvalTerm EvaluateCandidate(const std::vector<PreparedPoint>& points,
                           std::size_t i,
                           const std::vector<std::size_t>& selected,
                           double distRange,
                           double velRange,
                           double pitchRange,
                           const PlannerConfig& config,
                           const PlannerWeights& weights) {
  EvalTerm t;
  if (selected.empty()) {
    return t;
  }

  std::vector<double> selDist;
  std::vector<double> selVel;
  std::vector<double> selPitch;
  BuildSelectedInterpolator(points, selected, &selDist, &selVel, &selPitch);

  t.fit = InterpErrorAtIndex(points, i, selDist, selVel, selPitch, velRange, pitchRange, weights.alpha);

  double minVelDiff = std::numeric_limits<double>::infinity();
  double minDistDiff = std::numeric_limits<double>::infinity();
  for (std::size_t j : selected) {
    minVelDiff = std::min(minVelDiff, std::fabs(points[i].point.theoryVelMps - points[j].point.theoryVelMps));
    minDistDiff = std::min(minDistDiff, std::fabs(points[i].point.distanceM - points[j].point.distanceM));
  }
  if (!std::isfinite(minVelDiff)) {
    minVelDiff = velRange;
  }
  if (!std::isfinite(minDistDiff)) {
    minDistDiff = distRange;
  }

  const double velSpace = math::Clamp(minVelDiff / velRange, 0.0, 1.0);
  const double distSpace = math::Clamp(minDistDiff / distRange, 0.0, 1.0);
  t.space = weights.beta * velSpace + (1.0 - weights.beta) * distSpace;

  t.curv = weights.gamma * points[i].curvatureVel + (1.0 - weights.gamma) * points[i].curvaturePitch;
  t.sens = config.useDeltaV ? points[i].sensitivity : 0.0;
  t.edge = (points[i].edgeDistanceMin || points[i].edgeDistanceMax ||
            points[i].edgeVelocityMin || points[i].edgeVelocityMax) ? weights.edgeBonus : 0.0;

  t.score = weights.wFit * t.fit + weights.wSpace * t.space + weights.wCurv * t.curv +
            weights.wSens * t.sens + weights.wEdge * t.edge;

  if (config.minVelocitySpacingMps > 1e-9) {
    const double spacingScale = math::Clamp(minVelDiff / config.minVelocitySpacingMps, 0.0, 1.0);
    t.score *= spacingScale;
  }

  return t;
}

std::vector<std::string> BuildReasonTags(const PreparedPoint& p, const EvalTerm& term, bool anchor) {
  std::vector<std::string> tags;
  if (anchor) {
    tags.push_back("ANCHOR_POINT");
  }
  if (p.edgeDistanceMin) {
    tags.push_back("EDGE_DISTANCE_MIN");
  }
  if (p.edgeDistanceMax) {
    tags.push_back("EDGE_DISTANCE_MAX");
  }
  if (p.edgeVelocityMin) {
    tags.push_back("EDGE_VELOCITY_MIN");
  }
  if (p.edgeVelocityMax) {
    tags.push_back("EDGE_VELOCITY_MAX");
  }
  if (term.curv >= 0.60) {
    tags.push_back("HIGH_CURVATURE");
  }
  if (term.sens >= 0.60) {
    tags.push_back("HIGH_SENSITIVITY");
  }
  if (term.fit >= 0.50) {
    tags.push_back("LARGE_INTERP_ERROR");
  }
  if (term.space >= 0.55) {
    tags.push_back("SPACE_FILLING");
  }
  if (tags.empty()) {
    tags.push_back("REPRESENTATIVE_POINT");
  }
  return tags;
}

std::size_t FindMedianVelocityPoint(const std::vector<PreparedPoint>& points,
                                    const std::set<std::size_t>& selected) {
  std::vector<double> values;
  values.reserve(points.size());
  for (const auto& p : points) {
    values.push_back(p.point.theoryVelMps);
  }
  const double median = math::Median(values);

  std::size_t best = 0U;
  double bestDist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (selected.find(i) != selected.end()) {
      continue;
    }
    const double dv = std::fabs(points[i].point.theoryVelMps - median);
    if (dv < bestDist) {
      bestDist = dv;
      best = i;
    }
  }
  return best;
}

std::string StopReasonText(PlannerStopReason reason) {
  switch (reason) {
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

}  // namespace

std::vector<RecommendedPoint> CalibrationPlanner::Recommend(const std::vector<TheoryPoint>& theory,
                                                            const PlannerConfig& config,
                                                            PlannerSummary* summary,
                                                            std::vector<RecommendedPoint>* allScores,
                                                            std::string* warning) const {
  std::vector<RecommendedPoint> out;
  if (warning != nullptr) {
    warning->clear();
  }
  if (allScores != nullptr) {
    allScores->clear();
  }
  if (summary != nullptr) {
    *summary = {};
    summary->mode = config.mode;
    summary->inputPointCount = theory.size();
  }

  if (theory.empty()) {
    if (warning != nullptr) {
      *warning = "No theory data for planner";
    }
    if (summary != nullptr) {
      summary->stopReason = PlannerStopReason::kInputTooSmall;
      summary->stopReasonText = StopReasonText(summary->stopReason);
    }
    return out;
  }

  bool hasAnyDelta = false;
  std::vector<PreparedPoint> points = Preprocess(theory, &hasAnyDelta);
  if (points.empty()) {
    if (warning != nullptr) {
      *warning = "No valid points after filtering invalid rows";
    }
    if (summary != nullptr) {
      summary->stopReason = PlannerStopReason::kInputTooSmall;
      summary->stopReasonText = StopReasonText(summary->stopReason);
    }
    return out;
  }

  const PlannerWeights weights = BuildWeights(config.mode);
  const std::size_t targetCount = std::max<std::size_t>(1U, config.recommendedCount);

  MarkEdgesAndFeatures(&points, config.useDeltaV && hasAnyDelta);

  const double distMin = points.front().point.distanceM;
  const double distMax = points.back().point.distanceM;
  double velMin = points.front().point.theoryVelMps;
  double velMax = points.front().point.theoryVelMps;
  double pitchMin = points.front().point.pitchDeg;
  double pitchMax = points.front().point.pitchDeg;
  for (const auto& p : points) {
    velMin = std::min(velMin, p.point.theoryVelMps);
    velMax = std::max(velMax, p.point.theoryVelMps);
    pitchMin = std::min(pitchMin, p.point.pitchDeg);
    pitchMax = std::max(pitchMax, p.point.pitchDeg);
  }
  const double distRange = SafeRange(distMin, distMax);
  const double velRange = SafeRange(velMin, velMax);
  const double pitchRange = SafeRange(pitchMin, pitchMax);

  std::set<std::size_t> selectedSet;
  std::set<std::size_t> medianRepresentativeSet;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (points[i].edgeDistanceMin || points[i].edgeDistanceMax ||
        points[i].edgeVelocityMin || points[i].edgeVelocityMax) {
      selectedSet.insert(i);
    }
  }

  if (selectedSet.size() < std::min<std::size_t>(3U, targetCount)) {
    const std::size_t medianIdx = FindMedianVelocityPoint(points, selectedSet);
    selectedSet.insert(medianIdx);
    medianRepresentativeSet.insert(medianIdx);
  }

  std::vector<std::size_t> selected(selectedSet.begin(), selectedSet.end());
  std::sort(selected.begin(), selected.end());
  std::vector<double> selectedScore(points.size(), -1.0);
  for (std::size_t idx : selected) {
    if (points[idx].edgeDistanceMin || points[idx].edgeDistanceMax ||
        points[idx].edgeVelocityMin || points[idx].edgeVelocityMax) {
      selectedScore[idx] = 1.0;
    } else if (medianRepresentativeSet.find(idx) != medianRepresentativeSet.end()) {
      selectedScore[idx] = 0.75;
    } else {
      selectedScore[idx] = 0.50;
    }
  }

  PlannerStopReason stopReason = PlannerStopReason::kInsufficientCandidatePoints;
  while (selected.size() < targetCount) {
    if (config.maxInterpErrorThreshold > 0.0) {
      const ErrorStats stats = ComputeErrorStats(points, selected, velRange, pitchRange, weights.alpha);
      if (stats.maxError <= config.maxInterpErrorThreshold) {
        stopReason = PlannerStopReason::kInterpolationErrorBelowThreshold;
        break;
      }
    }

    double bestScore = -1.0;
    std::size_t bestIdx = static_cast<std::size_t>(-1);
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (selectedSet.find(i) != selectedSet.end()) {
        continue;
      }
      const EvalTerm term = EvaluateCandidate(points, i, selected, distRange, velRange, pitchRange, config, weights);
      if (term.score > bestScore) {
        bestScore = term.score;
        bestIdx = i;
      }
    }

    if (bestIdx == static_cast<std::size_t>(-1) || bestScore < 0.0) {
      stopReason = PlannerStopReason::kInsufficientCandidatePoints;
      break;
    }

    selectedSet.insert(bestIdx);
    selected.push_back(bestIdx);
    selectedScore[bestIdx] = bestScore;
    std::sort(selected.begin(), selected.end());
  }

  if (selected.size() >= targetCount) {
    stopReason = PlannerStopReason::kReachedTargetCount;
  } else if (stopReason == PlannerStopReason::kInsufficientCandidatePoints &&
             config.maxInterpErrorThreshold > 0.0) {
    const ErrorStats stats = ComputeErrorStats(points, selected, velRange, pitchRange, weights.alpha);
    if (stats.maxError <= config.maxInterpErrorThreshold) {
      stopReason = PlannerStopReason::kInterpolationErrorBelowThreshold;
    }
  }

  out.reserve(selected.size());
  for (std::size_t idx : selected) {
    const bool isAnchor = points[idx].edgeDistanceMin || points[idx].edgeDistanceMax ||
                          points[idx].edgeVelocityMin || points[idx].edgeVelocityMax;
    EvalTerm term = EvaluateCandidate(points, idx, selected, distRange, velRange, pitchRange, config, weights);
    if (selectedScore[idx] >= 0.0) {
      term.score = selectedScore[idx];
    }
    RecommendedPoint rp;
    rp.originalIndex = points[idx].originalIndex;
    rp.point = points[idx].point;
    rp.finalScore = term.score;
    rp.selected = true;
    rp.reasonTags = BuildReasonTags(points[idx], term, isAnchor);
    if (medianRepresentativeSet.find(idx) != medianRepresentativeSet.end()) {
      rp.reasonTags.push_back("MEDIAN_REPRESENTATIVE");
    }
    rp.reason = JoinTags(rp.reasonTags);
    out.push_back(rp);
  }

  std::sort(out.begin(), out.end(), [](const RecommendedPoint& a, const RecommendedPoint& b) {
    return a.point.distanceM < b.point.distanceM;
  });

  if (allScores != nullptr) {
    allScores->reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      const bool selectedFlag = selectedSet.find(i) != selectedSet.end();
      EvalTerm term = EvaluateCandidate(points, i, selected, distRange, velRange, pitchRange, config, weights);
      if (selectedFlag && selectedScore[i] >= 0.0) {
        term.score = selectedScore[i];
      }
      RecommendedPoint row;
      row.originalIndex = points[i].originalIndex;
      row.point = points[i].point;
      row.finalScore = term.score;
      row.selected = selectedFlag;
      row.reasonTags = BuildReasonTags(points[i], term,
                                       points[i].edgeDistanceMin || points[i].edgeDistanceMax ||
                                           points[i].edgeVelocityMin || points[i].edgeVelocityMax);
      if (!selectedFlag) {
        row.reasonTags.push_back("UNSELECTED");
      }
      row.reason = JoinTags(row.reasonTags);
      allScores->push_back(row);
    }
    std::sort(allScores->begin(), allScores->end(), [](const RecommendedPoint& a, const RecommendedPoint& b) {
      if (a.selected != b.selected) {
        return a.selected && !b.selected;
      }
      return a.point.distanceM < b.point.distanceM;
    });
  }

  if (summary != nullptr) {
    const ErrorStats stats = ComputeErrorStats(points, selected, velRange, pitchRange, weights.alpha);
    summary->selectedPointCount = out.size();
    summary->minDistanceM = distMin;
    summary->maxDistanceM = distMax;
    summary->minTheoryVelMps = velMin;
    summary->maxTheoryVelMps = velMax;
    summary->maxInterpolationError = stats.maxError;
    summary->avgInterpolationError = stats.avgError;
    summary->usedDeltaV = config.useDeltaV && hasAnyDelta;
    summary->stopReason = stopReason;
    summary->stopReasonText = StopReasonText(stopReason);
  }

  if (config.verbose && warning != nullptr && summary != nullptr) {
    *warning = "Planner stop reason: " + summary->stopReasonText;
  }

  return out;
}

}  // namespace calibration
