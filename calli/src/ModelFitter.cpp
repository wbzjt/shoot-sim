#include "ModelFitter.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MathUtil.h"
#include "RuntimeModel.h"

namespace calibration {
namespace {

struct LineFit {
  double a = 0.0;
  double b = 0.0;
  bool ok = false;
};

LineFit FitLine(const std::vector<double>& x, const std::vector<double>& y, std::size_t begin, std::size_t end) {
  LineFit out;
  if (end <= begin || end - begin < 2U) {
    return out;
  }
  const std::size_t n = end - begin;
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    sx += x[i];
    sy += y[i];
    sxx += x[i] * x[i];
    sxy += x[i] * y[i];
  }
  const double denom = static_cast<double>(n) * sxx - sx * sx;
  if (std::fabs(denom) < 1e-12) {
    return out;
  }
  out.b = (static_cast<double>(n) * sxy - sx * sy) / denom;
  out.a = (sy - out.b * sx) / static_cast<double>(n);
  out.ok = true;
  return out;
}

double EvalLine(const LineFit& line, double x) {
  return line.a + line.b * x;
}

RuntimeModelData BuildLookupModel(const std::vector<FitDatasetPoint>& data) {
  RuntimeModelData model;
  model.kind = ModelKind::kLookupLinear;
  model.clamp = true;
  model.knotsX.reserve(data.size());
  model.knotsY.reserve(data.size());
  for (const auto& p : data) {
    model.knotsX.push_back(p.xTheoryVelMps);
    model.knotsY.push_back(p.yMotorRpm);
  }
  return model;
}

RuntimeModelData BuildPolynomialModel(const std::vector<FitDatasetPoint>& data, int degree, bool* ok) {
  RuntimeModelData model;
  model.kind = (degree == 2) ? ModelKind::kPolynomial2 :
               (degree == 3) ? ModelKind::kPolynomial3 : ModelKind::kPolynomial4;
  model.clamp = true;
  if (ok != nullptr) {
    *ok = false;
  }

  if (data.size() < static_cast<std::size_t>(degree + 1)) {
    return model;
  }

  const int n = degree + 1;
  std::vector<std::vector<double>> a(static_cast<std::size_t>(n), std::vector<double>(static_cast<std::size_t>(n), 0.0));
  std::vector<double> b(static_cast<std::size_t>(n), 0.0);

  for (const auto& p : data) {
    std::vector<double> xp(static_cast<std::size_t>(2 * degree + 1), 1.0);
    for (int k = 1; k <= 2 * degree; ++k) {
      xp[static_cast<std::size_t>(k)] = xp[static_cast<std::size_t>(k - 1)] * p.xTheoryVelMps;
    }
    for (int r = 0; r < n; ++r) {
      for (int c = 0; c < n; ++c) {
        a[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] += xp[static_cast<std::size_t>(r + c)];
      }
      b[static_cast<std::size_t>(r)] += p.yMotorRpm * xp[static_cast<std::size_t>(r)];
    }
  }

  bool solved = false;
  const std::vector<double> coeff = math::SolveLinearSystem(a, b, &solved);
  if (!solved || coeff.empty()) {
    return model;
  }

  model.polyCoefficients = coeff;
  if (ok != nullptr) {
    *ok = true;
  }
  return model;
}

RuntimeModelData BuildSegmentedLinearModel(const std::vector<FitDatasetPoint>& data,
                                           std::size_t segmentCount,
                                           bool* ok,
                                           std::vector<double>* outBreaks) {
  RuntimeModelData best;
  best.kind = (segmentCount == 2U) ? ModelKind::kSegmentedLinear2 : ModelKind::kSegmentedLinear3;
  best.clamp = true;
  if (ok != nullptr) {
    *ok = false;
  }
  if (outBreaks != nullptr) {
    outBreaks->clear();
  }

  if (data.size() < segmentCount * 2U) {
    return best;
  }

  std::vector<double> x;
  std::vector<double> y;
  x.reserve(data.size());
  y.reserve(data.size());
  for (const auto& p : data) {
    x.push_back(p.xTheoryVelMps);
    y.push_back(p.yMotorRpm);
  }

  double bestRmse = std::numeric_limits<double>::infinity();

  const std::size_t n = data.size();

  if (segmentCount == 2U) {
    for (std::size_t b = 2U; b + 2U < n; ++b) {
      const LineFit l1 = FitLine(x, y, 0U, b + 1U);
      const LineFit l2 = FitLine(x, y, b, n);
      if (!l1.ok || !l2.ok) {
        continue;
      }
      std::vector<double> pred;
      pred.reserve(n);
      for (std::size_t i = 0; i < n; ++i) {
        pred.push_back((i <= b) ? EvalLine(l1, x[i]) : EvalLine(l2, x[i]));
      }
      std::vector<double> absErr;
      absErr.reserve(n);
      double mse = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        const double e = pred[i] - y[i];
        mse += e * e;
        absErr.push_back(std::fabs(e));
      }
      const double rmse = std::sqrt(mse / static_cast<double>(n));
      if (rmse < bestRmse) {
        bestRmse = rmse;
        best.knotsX = x;
        best.knotsY = pred;
        best.segmentBreakpoints = {x[b]};
      }
    }
  } else {
    for (std::size_t b1 = 2U; b1 + 4U < n; ++b1) {
      for (std::size_t b2 = b1 + 2U; b2 + 2U < n; ++b2) {
        const LineFit l1 = FitLine(x, y, 0U, b1 + 1U);
        const LineFit l2 = FitLine(x, y, b1, b2 + 1U);
        const LineFit l3 = FitLine(x, y, b2, n);
        if (!l1.ok || !l2.ok || !l3.ok) {
          continue;
        }

        std::vector<double> pred;
        pred.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
          if (i <= b1) {
            pred.push_back(EvalLine(l1, x[i]));
          } else if (i <= b2) {
            pred.push_back(EvalLine(l2, x[i]));
          } else {
            pred.push_back(EvalLine(l3, x[i]));
          }
        }

        double mse = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
          const double e = pred[i] - y[i];
          mse += e * e;
        }
        const double rmse = std::sqrt(mse / static_cast<double>(n));
        if (rmse < bestRmse) {
          bestRmse = rmse;
          best.knotsX = x;
          best.knotsY = pred;
          best.segmentBreakpoints = {x[b1], x[b2]};
        }
      }
    }
  }

  if (!best.knotsX.empty() && ok != nullptr) {
    *ok = true;
  }
  if (ok != nullptr && *ok && outBreaks != nullptr) {
    *outBreaks = best.segmentBreakpoints;
  }
  return best;
}

std::vector<double> AbsResiduals(const RuntimeModelData& model,
                                 const std::vector<FitDatasetPoint>& data) {
  std::vector<double> absErr;
  absErr.reserve(data.size());
  for (const auto& p : data) {
    absErr.push_back(std::fabs(runtime::EvaluateModel(model, p.xTheoryVelMps) - p.yMotorRpm));
  }
  return absErr;
}

double ComputeLoocvRmse(ModelKind kind, const std::vector<FitDatasetPoint>& data) {
  if (data.size() < 3U) {
    return 0.0;
  }

  double mse = 0.0;
  int count = 0;

  for (std::size_t leave = 0; leave < data.size(); ++leave) {
    std::vector<FitDatasetPoint> train;
    train.reserve(data.size() - 1U);
    for (std::size_t i = 0; i < data.size(); ++i) {
      if (i != leave) {
        train.push_back(data[i]);
      }
    }

    bool ok = false;
    RuntimeModelData model;
    if (kind == ModelKind::kLookupLinear) {
      model = BuildLookupModel(train);
      ok = true;
    } else if (kind == ModelKind::kPolynomial2) {
      model = BuildPolynomialModel(train, 2, &ok);
    } else if (kind == ModelKind::kPolynomial3) {
      model = BuildPolynomialModel(train, 3, &ok);
    } else if (kind == ModelKind::kPolynomial4) {
      model = BuildPolynomialModel(train, 4, &ok);
    } else if (kind == ModelKind::kSegmentedLinear2) {
      model = BuildSegmentedLinearModel(train, 2U, &ok, nullptr);
    } else if (kind == ModelKind::kSegmentedLinear3) {
      model = BuildSegmentedLinearModel(train, 3U, &ok, nullptr);
    }

    if (!ok) {
      continue;
    }

    const double pred = runtime::EvaluateModel(model, data[leave].xTheoryVelMps);
    const double e = pred - data[leave].yMotorRpm;
    mse += e * e;
    ++count;
  }

  if (count == 0) {
    return 0.0;
  }
  return std::sqrt(mse / static_cast<double>(count));
}

ModelMetrics BuildMetrics(const RuntimeModelData& model,
                          const std::vector<FitDatasetPoint>& data,
                          bool computeLoocv) {
  ModelMetrics m;
  if (data.empty()) {
    return m;
  }

  double mse = 0.0;
  std::vector<double> absErr;
  absErr.reserve(data.size());
  for (const auto& p : data) {
    const double pred = runtime::EvaluateModel(model, p.xTheoryVelMps);
    const double e = pred - p.yMotorRpm;
    mse += e * e;
    absErr.push_back(std::fabs(e));
  }

  m.rmse = std::sqrt(mse / static_cast<double>(data.size()));
  m.mae = math::Mean(absErr);
  m.maxError = *std::max_element(absErr.begin(), absErr.end());
  m.p95Error = math::Quantile(absErr, 0.95);

  const double xMin = data.front().xTheoryVelMps;
  const double xMax = data.back().xTheoryVelMps;
  m.monotonic = runtime::IsModelMonotonic(model, xMin, xMax, 200, true);
  m.localReverse = !m.monotonic;

  if (computeLoocv) {
    m.loocvRmse = ComputeLoocvRmse(model.kind, data);
  }

  switch (model.kind) {
    case ModelKind::kLookupLinear:
      m.complexityPenalty = 0.15;
      break;
    case ModelKind::kSegmentedLinear2:
      m.complexityPenalty = 0.45;
      break;
    case ModelKind::kSegmentedLinear3:
      m.complexityPenalty = 0.75;
      break;
    case ModelKind::kPolynomial2:
      m.complexityPenalty = 0.40;
      break;
    case ModelKind::kPolynomial3:
      m.complexityPenalty = 0.70;
      break;
    case ModelKind::kPolynomial4:
      m.complexityPenalty = 1.00;
      break;
  }

  return m;
}

double SafeScore(const ModelMetrics& m) {
  double score = m.rmse + 0.45 * m.maxError + 0.45 * m.loocvRmse + 20.0 * m.complexityPenalty;
  if (!m.monotonic) {
    score += 10000.0;
  }
  return score;
}

double AccuracyScore(const ModelMetrics& m) {
  double score = m.rmse + 0.25 * m.maxError + 0.5 * m.loocvRmse + 6.0 * m.complexityPenalty;
  if (!m.monotonic) {
    score += 400.0;
  }
  return score;
}

}  // namespace

std::vector<FitDatasetPoint> ModelFitter::BuildFitDataset(const std::vector<AggregatedPoint>& aggregated) {
  std::vector<FitDatasetPoint> data;
  for (const auto& p : aggregated) {
    if (!p.usableForFit) {
      continue;
    }
    data.push_back({p.theoryVelMps, p.recommendedMotorRpm});
  }

  std::sort(data.begin(), data.end(), [](const FitDatasetPoint& a, const FitDatasetPoint& b) {
    return a.xTheoryVelMps < b.xTheoryVelMps;
  });

  // Merge duplicated x values by averaging y.
  std::vector<FitDatasetPoint> merged;
  for (const auto& p : data) {
    if (!merged.empty() && std::fabs(merged.back().xTheoryVelMps - p.xTheoryVelMps) < 1e-9) {
      merged.back().yMotorRpm = 0.5 * (merged.back().yMotorRpm + p.yMotorRpm);
    } else {
      merged.push_back(p);
    }
  }

  return merged;
}

std::vector<ModelReport> ModelFitter::Fit(const std::vector<AggregatedPoint>& aggregated,
                                          const FitterConfig& config,
                                          std::string* warning) const {
  if (warning != nullptr) {
    warning->clear();
  }

  std::vector<ModelReport> reports;
  const std::vector<FitDatasetPoint> data = BuildFitDataset(aggregated);
  if (data.size() < 2U) {
    if (warning != nullptr) {
      *warning = "Not enough usable aggregated points to fit models";
    }
    return reports;
  }

  const bool doLoocv = static_cast<int>(data.size()) >= config.loocvMinPoints;

  // Lookup linear baseline.
  {
    ModelReport r;
    r.kind = ModelKind::kLookupLinear;
    r.model = BuildLookupModel(data);
    r.metrics = BuildMetrics(r.model, data, doLoocv);
    r.notes = "Lookup table + linear interpolation baseline";
    r.safeScore = SafeScore(r.metrics);
    r.accuracyScore = AccuracyScore(r.metrics);
    reports.push_back(r);
  }

  if (config.enableSegmented2 && data.size() >= 8U) {
    bool ok = false;
    RuntimeModelData model = BuildSegmentedLinearModel(data, 2U, &ok, nullptr);
    if (ok) {
      ModelReport r;
      r.kind = ModelKind::kSegmentedLinear2;
      r.model = model;
      r.metrics = BuildMetrics(r.model, data, doLoocv);
      r.notes = "2-segment linear fit";
      r.safeScore = SafeScore(r.metrics);
      r.accuracyScore = AccuracyScore(r.metrics);
      reports.push_back(r);
    }
  }

  if (config.enableSegmented3 && data.size() >= 14U) {
    bool ok = false;
    RuntimeModelData model = BuildSegmentedLinearModel(data, 3U, &ok, nullptr);
    if (ok) {
      ModelReport r;
      r.kind = ModelKind::kSegmentedLinear3;
      r.model = model;
      r.metrics = BuildMetrics(r.model, data, doLoocv);
      r.notes = "3-segment linear fit";
      r.safeScore = SafeScore(r.metrics);
      r.accuracyScore = AccuracyScore(r.metrics);
      reports.push_back(r);
    }
  }

  if (config.enablePolynomial) {
    for (int d = 2; d <= 4; ++d) {
      bool ok = false;
      RuntimeModelData model = BuildPolynomialModel(data, d, &ok);
      if (!ok) {
        continue;
      }
      ModelReport r;
      r.kind = (d == 2) ? ModelKind::kPolynomial2 : (d == 3) ? ModelKind::kPolynomial3 : ModelKind::kPolynomial4;
      r.model = model;
      r.metrics = BuildMetrics(r.model, data, doLoocv);
      r.notes = "Low-order polynomial fit";
      r.safeScore = SafeScore(r.metrics);
      r.accuracyScore = AccuracyScore(r.metrics);
      reports.push_back(r);
    }
  }

  if (config.requireMonotonicForRecommended) {
    for (auto& r : reports) {
      if (!r.metrics.monotonic) {
        r.notes += " (non-monotonic; not recommended for robot runtime)";
      }
    }
  }

  return reports;
}

std::optional<ModelReport> ModelFitter::PickBestSafe(const std::vector<ModelReport>& reports) {
  if (reports.empty()) {
    return std::nullopt;
  }
  const auto it = std::min_element(reports.begin(), reports.end(), [](const ModelReport& a, const ModelReport& b) {
    return a.safeScore < b.safeScore;
  });
  return *it;
}

std::optional<ModelReport> ModelFitter::PickBestAccuracy(const std::vector<ModelReport>& reports) {
  if (reports.empty()) {
    return std::nullopt;
  }
  const auto it = std::min_element(reports.begin(), reports.end(), [](const ModelReport& a, const ModelReport& b) {
    return a.accuracyScore < b.accuracyScore;
  });
  return *it;
}

}  // namespace calibration
