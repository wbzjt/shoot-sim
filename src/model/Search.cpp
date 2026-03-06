#include "model/Search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>

#include "model/Ballistics.h"
#include "model/Constraints.h"
#include "model/Geometry.h"
#include "util/MathUtil.h"

namespace shootersim::model {
namespace {

constexpr int kBoundaryRefineIterations = 22;
constexpr double kTieEpsilon = 1e-6;

using ValidityFn = std::function<bool(double)>;

bool PassNonWindowConstraints(const ConstraintResult& r) {
  return r.pitchInRange && r.speedInRange && r.effectiveWindowValid && r.ceilingOk &&
         r.hasDescendingIntersection && r.entryAngleOk && r.flightTimeOk;
}

double RefineLowBoundary(double invalidV, double validV, const ValidityFn& isValid) {
  double lo = invalidV;
  double hi = validV;
  for (int i = 0; i < kBoundaryRefineIterations; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (isValid(mid)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

double RefineHighBoundary(double validV, double invalidV, const ValidityFn& isValid) {
  double lo = validV;
  double hi = invalidV;
  for (int i = 0; i < kBoundaryRefineIterations; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (isValid(mid)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

std::vector<ValidInterval> BuildIntervalsFromScan(const std::vector<double>& vSamples,
                                                  const std::vector<uint8_t>& validRow,
                                                  const ValidityFn& isValid) {
  std::vector<ValidInterval> intervals;
  const std::size_t n = vSamples.size();
  if (n == 0U || validRow.size() != n) {
    return intervals;
  }

  bool inRun = false;
  std::size_t runStart = 0;

  for (std::size_t i = 0; i < n; ++i) {
    const bool valid = (validRow[i] != 0U);
    if (valid && !inRun) {
      runStart = i;
      inRun = true;
    }

    const bool runEnds = inRun && (!valid || i + 1U == n);
    if (!runEnds) {
      continue;
    }

    const std::size_t runEnd = valid ? i : (i - 1U);
    double vLow = vSamples[runStart];
    double vHigh = vSamples[runEnd];

    if (runStart > 0U) {
      const double invalidBelow = vSamples[runStart - 1U];
      vLow = RefineLowBoundary(invalidBelow, vLow, isValid);
    }
    if (runEnd + 1U < n) {
      const double invalidAbove = vSamples[runEnd + 1U];
      vHigh = RefineHighBoundary(vHigh, invalidAbove, isValid);
    }

    ValidInterval interval;
    interval.vLow = vLow;
    interval.vHigh = vHigh;
    interval.deltaV = std::max(0.0, vHigh - vLow);
    interval.refined = true;
    if (interval.deltaV > 0.0) {
      intervals.push_back(interval);
    }

    inRun = false;
  }

  return intervals;
}

bool IsMonotonicXEntryOnInterval(const SimulationConfig& config,
                                 double distance,
                                 double thetaRad,
                                 double vLow,
                                 double vHigh,
                                 bool* increasing) {
  const int kProbeCount = 10;
  double prevX = 0.0;
  int sign = 0;

  for (int i = 0; i < kProbeCount; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(kProbeCount - 1);
    const double v = vLow + alpha * (vHigh - vLow);
    const auto eval = EvaluateConstraints(config, distance, thetaRad, v);
    if (!PassNonWindowConstraints(eval.result)) {
      return false;
    }
    const double x = eval.intersection.xEntry;
    if (i > 0) {
      const double diff = x - prevX;
      if (std::fabs(diff) > 1e-7) {
        const int curSign = (diff > 0.0) ? 1 : -1;
        if (sign == 0) {
          sign = curSign;
        } else if (curSign != sign) {
          return false;
        }
      }
    }
    prevX = x;
  }

  if (sign == 0) {
    return false;
  }
  *increasing = (sign > 0);
  return true;
}

std::optional<double> SolveForVelocityAtX(const SimulationConfig& config,
                                          double distance,
                                          double thetaRad,
                                          double xTarget,
                                          double vLow,
                                          double vHigh,
                                          bool increasing) {
  auto xAt = [&](double v) -> std::optional<double> {
    const auto eval = EvaluateConstraints(config, distance, thetaRad, v);
    if (!PassNonWindowConstraints(eval.result)) {
      return std::nullopt;
    }
    return eval.intersection.xEntry;
  };

  auto xLo = xAt(vLow);
  auto xHi = xAt(vHigh);
  if (!xLo.has_value() || !xHi.has_value()) {
    return std::nullopt;
  }

  double lo = vLow;
  double hi = vHigh;
  const double dir = increasing ? 1.0 : -1.0;
  double fLo = (*xLo - xTarget) * dir;
  double fHi = (*xHi - xTarget) * dir;

  if (fLo * fHi > 0.0) {
    return std::nullopt;
  }

  for (int i = 0; i < 32; ++i) {
    const double mid = 0.5 * (lo + hi);
    const auto xMid = xAt(mid);
    if (!xMid.has_value()) {
      return std::nullopt;
    }
    const double fMid = *xMid - xTarget;
    if (fLo * fMid <= 0.0) {
      hi = mid;
      fHi = fMid;
    } else {
      lo = mid;
      fLo = fMid;
    }
  }

  const double root = 0.5 * (lo + hi);
  return root;
}

ValidInterval RefineIntervalByMonotonicX(const SimulationConfig& config,
                                         double distance,
                                         double thetaRad,
                                         const EffectiveWindow& window,
                                         const ValidInterval& coarse,
                                         const ValidityFn& isValid) {
  ValidInterval out = coarse;

  bool increasing = false;
  if (!IsMonotonicXEntryOnInterval(config, distance, thetaRad, coarse.vLow, coarse.vHigh,
                                   &increasing)) {
    return out;
  }

  const auto vFront = SolveForVelocityAtX(config, distance, thetaRad, window.xFrontEff,
                                          coarse.vLow, coarse.vHigh, increasing);
  const auto vBack = SolveForVelocityAtX(config, distance, thetaRad, window.xBackEff,
                                         coarse.vLow, coarse.vHigh, increasing);
  if (!vFront.has_value() || !vBack.has_value()) {
    return out;
  }

  double vLow = std::min(*vFront, *vBack);
  double vHigh = std::max(*vFront, *vBack);

  if (!isValid(vLow) || !isValid(vHigh)) {
    return out;
  }

  const double lowRefined = RefineLowBoundary(std::max(config.vMin, vLow - config.vStep), vLow, isValid);
  const double highRefined = RefineHighBoundary(vHigh, std::min(config.vMax, vHigh + config.vStep), isValid);

  if (highRefined > lowRefined) {
    out.vLow = lowRefined;
    out.vHigh = highRefined;
    out.deltaV = highRefined - lowRefined;
    out.refined = true;
  }
  return out;
}

double SelectNominalSpeedByWindowBias(const SimulationConfig& config,
                                      double distance,
                                      double thetaRad,
                                      const EffectiveWindow& window,
                                      const ValidInterval& interval) {
  const double xTarget = window.xFrontEff +
                         config.targetBias * (window.xBackEff - window.xFrontEff);

  auto evaluateX = [&](double v) -> std::optional<double> {
    const auto eval = EvaluateConstraints(config, distance, thetaRad, v);
    if (!eval.result.overallValid) {
      return std::nullopt;
    }
    return eval.intersection.xEntry;
  };

  const auto xLow = evaluateX(interval.vLow);
  const auto xHigh = evaluateX(interval.vHigh);

  if (xLow.has_value() && xHigh.has_value()) {
    const double fLow = *xLow - xTarget;
    const double fHigh = *xHigh - xTarget;
    if (fLow * fHigh <= 0.0) {
      double lo = interval.vLow;
      double hi = interval.vHigh;
      double gLo = fLow;
      for (int i = 0; i < 32; ++i) {
        const double mid = 0.5 * (lo + hi);
        const auto xMid = evaluateX(mid);
        if (!xMid.has_value()) {
          break;
        }
        const double gMid = *xMid - xTarget;
        if (gLo * gMid <= 0.0) {
          hi = mid;
        } else {
          lo = mid;
          gLo = gMid;
        }
      }
      return 0.5 * (lo + hi);
    }
  }

  // Fallback robust scan for non-monotonic cases.
  const int kSamples = 100;
  double bestV = interval.vLow;
  double bestCost = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= kSamples; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(kSamples);
    const double v = interval.vLow + alpha * interval.deltaV;
    const auto x = evaluateX(v);
    if (!x.has_value()) {
      continue;
    }
    const double cost = std::fabs(*x - xTarget);
    if (cost < bestCost) {
      bestCost = cost;
      bestV = v;
    }
  }
  return bestV;
}

double SelectNominalSpeed(const SimulationConfig& config,
                          double distance,
                          double thetaRad,
                          const EffectiveWindow& window,
                          const ValidInterval& interval) {
  if (config.nominalMode == NominalSelectionMode::kSpeedBias) {
    const double bias = util::Clamp(config.speedBias, 0.0, 1.0);
    return interval.vLow + bias * interval.deltaV;
  }
  return SelectNominalSpeedByWindowBias(config, distance, thetaRad, window, interval);
}

bool IsBetterInterval(const ValidInterval& lhs,
                      double lhsNominal,
                      const ValidInterval& rhs,
                      double rhsNominal) {
  if (lhs.deltaV > rhs.deltaV + kTieEpsilon) {
    return true;
  }
  if (std::fabs(lhs.deltaV - rhs.deltaV) <= kTieEpsilon) {
    return lhsNominal < rhsNominal;
  }
  return false;
}

}  // namespace

SolveResult SolveForDistance(const SimulationConfig& config, double distance) {
  const auto t0 = std::chrono::steady_clock::now();

  SolveResult out;
  out.distance = distance;
  out.rawWindow = ComputeRawWindow(config, distance);
  out.effectiveWindow = ComputeEffectiveWindow(config, out.rawWindow);

  if (!out.effectiveWindow.valid) {
    out.failureReason = out.effectiveWindow.error;
    return out;
  }

  out.thetaSamplesRad = util::Linspace(config.thetaMinRad, config.thetaMaxRad, config.thetaStepRad);
  out.velocitySamples = util::Linspace(config.vMin, config.vMax, config.vStep);

  const std::size_t thetaCount = out.thetaSamplesRad.size();
  const std::size_t vCount = out.velocitySamples.size();
  out.validityGrid.assign(thetaCount * vCount, 0U);
  out.thetaScan.reserve(thetaCount);

  bool hasGlobalBest = false;
  double globalTheta = 0.0;
  ValidInterval globalInterval;
  double globalNominal = 0.0;

  for (std::size_t thetaIdx = 0; thetaIdx < thetaCount; ++thetaIdx) {
    const double theta = out.thetaSamplesRad[thetaIdx];

    std::vector<uint8_t> row(vCount, 0U);
    auto isValid = [&](double v) {
      return IsShotValid(config, distance, theta, v);
    };

    for (std::size_t vIdx = 0; vIdx < vCount; ++vIdx) {
      const bool valid = isValid(out.velocitySamples[vIdx]);
      row[vIdx] = valid ? 1U : 0U;
      out.validityGrid[thetaIdx * vCount + vIdx] = row[vIdx];
    }

    std::vector<ValidInterval> intervals = BuildIntervalsFromScan(out.velocitySamples, row, isValid);
    if (config.enableMonotonicBoundarySearch) {
      for (auto& interval : intervals) {
        interval = RefineIntervalByMonotonicX(config, distance, theta, out.effectiveWindow,
                                              interval, isValid);
      }
    }

    ThetaScanResult thetaResult;
    thetaResult.thetaRad = theta;

    if (intervals.empty()) {
      out.thetaScan.push_back(thetaResult);
      continue;
    }

    bool hasThetaBest = false;
    ValidInterval thetaBestInterval;
    double thetaBestNominal = 0.0;

    for (const auto& interval : intervals) {
      const double vNominal = SelectNominalSpeed(config, distance, theta, out.effectiveWindow,
                                                 interval);
      if (!hasThetaBest || IsBetterInterval(interval, vNominal, thetaBestInterval, thetaBestNominal)) {
        hasThetaBest = true;
        thetaBestInterval = interval;
        thetaBestNominal = vNominal;
      }
    }

    if (hasThetaBest) {
      thetaResult.hasValidInterval = true;
      thetaResult.bestInterval = thetaBestInterval;
      thetaResult.vNominal = thetaBestNominal;

      if (!hasGlobalBest ||
          IsBetterInterval(thetaBestInterval, thetaBestNominal, globalInterval, globalNominal)) {
        hasGlobalBest = true;
        globalTheta = theta;
        globalInterval = thetaBestInterval;
        globalNominal = thetaBestNominal;
      }
    }

    out.thetaScan.push_back(thetaResult);
  }

  if (!hasGlobalBest) {
    out.failureReason = "No feasible solution found under current constraints";
    const auto t1 = std::chrono::steady_clock::now();
    out.solveTimeMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return out;
  }

  out.hasSolution = true;
  out.bestThetaRad = globalTheta;
  out.bestVLow = globalInterval.vLow;
  out.bestVHigh = globalInterval.vHigh;
  out.bestDeltaV = globalInterval.deltaV;
  out.bestVNominal = globalNominal;

  const auto nominalEval = EvaluateConstraints(config, distance, out.bestThetaRad, out.bestVNominal);
  out.nominalConstraint = nominalEval.result;
  out.nominalIntersection = nominalEval.intersection;
  out.nominalApex = nominalEval.apex;
  out.lowIntersection =
      EvaluateConstraints(config, distance, out.bestThetaRad, out.bestVLow).intersection;
  out.highIntersection =
      EvaluateConstraints(config, distance, out.bestThetaRad, out.bestVHigh).intersection;

  const auto t1 = std::chrono::steady_clock::now();
  out.solveTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return out;
}

}  // namespace shootersim::model
