#include "CalibrationAggregator.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

#include "MathUtil.h"

namespace calibration {
namespace {

struct GroupKey {
  double theoryVel = 0.0;
  double pitchDeg = 0.0;
  double distance = 0.0;
  int lane = 0;

  bool operator<(const GroupKey& rhs) const {
    if (theoryVel != rhs.theoryVel) {
      return theoryVel < rhs.theoryVel;
    }
    if (pitchDeg != rhs.pitchDeg) {
      return pitchDeg < rhs.pitchDeg;
    }
    if (distance != rhs.distance) {
      return distance < rhs.distance;
    }
    return lane < rhs.lane;
  }
};

std::vector<std::size_t> FindOutliersZScore(const std::vector<double>& values, double threshold) {
  std::vector<std::size_t> out;
  const double mu = math::Mean(values);
  const double sd = math::StdDev(values);
  if (sd < 1e-9) {
    return out;
  }
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double z = std::fabs((values[i] - mu) / sd);
    if (z > threshold) {
      out.push_back(i);
    }
  }
  return out;
}

std::vector<std::size_t> FindOutliersMad(const std::vector<double>& values) {
  std::vector<std::size_t> out;
  if (values.size() < 4U) {
    return out;
  }
  const double med = math::Median(values);
  std::vector<double> absDev;
  absDev.reserve(values.size());
  for (double v : values) {
    absDev.push_back(std::fabs(v - med));
  }
  const double mad = math::Median(absDev);
  if (mad < 1e-9) {
    return out;
  }
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double modifiedZ = 0.6745 * std::fabs(values[i] - med) / mad;
    if (modifiedZ > 3.5) {
      out.push_back(i);
    }
  }
  return out;
}

bool ContainsIndex(const std::vector<std::size_t>& v, std::size_t idx) {
  return std::find(v.begin(), v.end(), idx) != v.end();
}

}  // namespace

std::vector<AggregatedPoint> CalibrationAggregator::Aggregate(
    const std::vector<CalibrationSample>& samples,
    const AggregationConfig& config,
    std::string* warning) const {
  if (warning != nullptr) {
    warning->clear();
  }

  std::vector<AggregatedPoint> out;
  if (samples.empty()) {
    if (warning != nullptr) {
      *warning = "No calibration samples to aggregate";
    }
    return out;
  }

  std::map<GroupKey, std::vector<const CalibrationSample*>> groups;
  for (const auto& sample : samples) {
    GroupKey key;
    key.theoryVel = math::RoundTo(sample.theoryVelMps, 0.001);
    key.pitchDeg = math::RoundTo(sample.pitchDeg, 0.1);
    key.distance = math::RoundTo(sample.distanceM, 0.01);
    key.lane = config.splitByLane ? sample.laneIndex : 0;
    groups[key].push_back(&sample);
  }

  out.reserve(groups.size());

  for (const auto& [key, group] : groups) {
    std::vector<double> measured;
    measured.reserve(group.size());
    for (const auto* s : group) {
      measured.push_back(s->measuredMotorRpm);
    }

    std::vector<std::size_t> outliers;
    if (config.removeOutliers) {
      if (config.useMadOutlier) {
        outliers = FindOutliersMad(measured);
      } else {
        outliers = FindOutliersZScore(measured, config.zScoreThreshold);
      }
    }

    std::vector<const CalibrationSample*> kept;
    kept.reserve(group.size());
    for (std::size_t i = 0; i < group.size(); ++i) {
      if (!ContainsIndex(outliers, i)) {
        kept.push_back(group[i]);
      }
    }
    if (kept.empty()) {
      continue;
    }

    std::vector<double> keptMeasured;
    std::vector<double> keptTarget;
    keptMeasured.reserve(kept.size());
    keptTarget.reserve(kept.size());

    int hitCount = 0;
    for (const auto* s : kept) {
      keptMeasured.push_back(s->measuredMotorRpm);
      keptTarget.push_back(s->motorTargetRpm);
      if (s->hit) {
        ++hitCount;
      }
    }

    // Choose recommended motor target by grouped commanded RPM policy.
    std::map<int, std::vector<const CalibrationSample*>> byCommand;
    for (const auto* s : kept) {
      byCommand[static_cast<int>(std::lround(s->motorTargetRpm))].push_back(s);
    }

    double bestRpm = math::Mean(keptTarget);
    double bestScore = -1e9;
    for (const auto& [rpmKey, bucket] : byCommand) {
      int bucketHit = 0;
      std::vector<double> scoreValues;
      for (const auto* s : bucket) {
        if (s->hit) {
          ++bucketHit;
        }
        if (s->score.has_value()) {
          scoreValues.push_back(*s->score);
        }
      }

      const double hitRate = static_cast<double>(bucketHit) / static_cast<double>(bucket.size());
      const double meanScore = scoreValues.empty() ? 0.0 : math::Mean(scoreValues);
      double ranking = 0.0;
      if (config.preference == AggregationPreference::kHitRateThenScore) {
        ranking = hitRate * 1000.0 + meanScore;
      } else {
        ranking = hitRate * 1000.0 - static_cast<double>(rpmKey) * 0.01;
      }

      if (ranking > bestScore) {
        bestScore = ranking;
        bestRpm = static_cast<double>(rpmKey);
      }
    }

    AggregatedPoint agg;
    agg.distanceM = key.distance;
    agg.pitchDeg = key.pitchDeg;
    agg.theoryVelMps = key.theoryVel;
    agg.laneIndex = key.lane;
    agg.meanMeasuredRpm = math::Mean(keptMeasured);
    agg.medianMeasuredRpm = math::Median(keptMeasured);
    agg.stddevMeasuredRpm = math::StdDev(keptMeasured);
    agg.hitRate = static_cast<double>(hitCount) / static_cast<double>(kept.size());
    agg.recommendedMotorRpm = bestRpm;
    agg.sampleCount = static_cast<int>(kept.size());
    agg.outlierCount = static_cast<int>(outliers.size());

    const double sampleFactor = math::Clamp(static_cast<double>(agg.sampleCount) / 8.0, 0.0, 1.0);
    const double spreadFactor = 1.0 - math::Clamp(agg.stddevMeasuredRpm / 250.0, 0.0, 1.0);
    agg.confidence = math::Clamp(0.6 * agg.hitRate + 0.25 * sampleFactor + 0.15 * spreadFactor,
                                 0.0, 1.0);

    agg.uncertain = (agg.hitRate < 0.4 || agg.confidence < 0.45);
    agg.usableForFit = agg.sampleCount >= config.minSamplesPerPoint &&
                       agg.hitRate >= config.minHitRateForFit &&
                       !agg.uncertain;

    out.push_back(agg);
  }

  std::sort(out.begin(), out.end(), [](const AggregatedPoint& a, const AggregatedPoint& b) {
    if (a.theoryVelMps != b.theoryVelMps) {
      return a.theoryVelMps < b.theoryVelMps;
    }
    if (a.laneIndex != b.laneIndex) {
      return a.laneIndex < b.laneIndex;
    }
    return a.distanceM < b.distanceM;
  });

  if (out.empty() && warning != nullptr) {
    *warning = "All calibration points filtered out";
  }

  return out;
}

}  // namespace calibration
