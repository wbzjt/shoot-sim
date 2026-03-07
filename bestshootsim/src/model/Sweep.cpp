#include "model/Sweep.h"

#include <chrono>
#include <mutex>
#include <unordered_map>

#include "model/Search.h"
#include "util/MathUtil.h"

namespace shootersim::model {
namespace {

std::unordered_map<std::string, SweepResult> gSweepCache;
std::mutex gSweepCacheMutex;

std::size_t HashConfig(const SimulationConfig& c) {
  std::size_t seed = 0;
  seed = util::HashCombine(seed, static_cast<std::size_t>(c.physicsMode));
  seed = util::HashDouble(seed, c.gravity);
  seed = util::HashDouble(seed, c.releaseHeight);
  seed = util::HashDouble(seed, c.zTarget);
  seed = util::HashDouble(seed, c.dragCoefficient);
  seed = util::HashDouble(seed, c.dragDt);
  seed = util::HashDouble(seed, c.dragMaxTime);
  seed = util::HashCombine(seed, static_cast<std::size_t>(c.windowInputMode));
  seed = util::HashDouble(seed, c.xFrontOffset);
  seed = util::HashDouble(seed, c.xBackOffset);
  seed = util::HashDouble(seed, c.openingDepth);
  seed = util::HashDouble(seed, c.ballRadius);
  seed = util::HashDouble(seed, c.frontMargin);
  seed = util::HashDouble(seed, c.backMargin);
  seed = util::HashDouble(seed, c.thetaMinRad);
  seed = util::HashDouble(seed, c.thetaMaxRad);
  seed = util::HashDouble(seed, c.vMin);
  seed = util::HashDouble(seed, c.vMax);
  seed = util::HashDouble(seed, c.zCeilingMax);
  seed = util::HashDouble(seed, c.thetaStepRad);
  seed = util::HashDouble(seed, c.vStep);
  seed = util::HashCombine(seed, static_cast<std::size_t>(c.enableMonotonicBoundarySearch));
  seed = util::HashCombine(seed, static_cast<std::size_t>(c.enableEntryAngleConstraint));
  seed = util::HashDouble(seed, c.minAbsEntryAngleRad);
  seed = util::HashCombine(seed, static_cast<std::size_t>(c.enableFlightTimeConstraint));
  seed = util::HashDouble(seed, c.maxFlightTime);
  seed = util::HashCombine(seed, static_cast<std::size_t>(c.nominalMode));
  seed = util::HashDouble(seed, c.targetBias);
  seed = util::HashDouble(seed, c.speedBias);
  return seed;
}

}  // namespace

std::vector<double> BuildDistanceSamples(const SweepRequest& request) {
  std::vector<double> out;
  if (request.distanceStep <= 0.0 || request.distanceMax < request.distanceMin) {
    return out;
  }

  for (double d = request.distanceMin; d <= request.distanceMax + request.distanceStep * 0.5;
       d += request.distanceStep) {
    out.push_back((d > request.distanceMax) ? request.distanceMax : d);
    if (out.back() >= request.distanceMax) {
      break;
    }
  }
  return out;
}

std::string BuildSweepCacheKey(const SimulationConfig& config, const SweepRequest& request) {
  std::size_t seed = HashConfig(config);
  seed = util::HashDouble(seed, request.distanceMin);
  seed = util::HashDouble(seed, request.distanceMax);
  seed = util::HashDouble(seed, request.distanceStep);
  return std::to_string(seed);
}

SweepResult RunSweep(const SimulationConfig& config,
                     const SweepRequest& request,
                     const std::function<void(double)>& progressCallback) {
  const auto t0 = std::chrono::steady_clock::now();

  SweepResult result;
  result.request = request;

  const std::string cacheKey = BuildSweepCacheKey(config, request);
  {
    std::lock_guard<std::mutex> lock(gSweepCacheMutex);
    auto it = gSweepCache.find(cacheKey);
    if (it != gSweepCache.end()) {
      result = it->second;
      result.fromCache = true;
      if (progressCallback) {
        progressCallback(1.0);
      }
      return result;
    }
  }

  const std::vector<double> distances = BuildDistanceSamples(request);
  if (distances.empty()) {
    result.totalTimeMs = 0.0;
    return result;
  }

  result.points.reserve(distances.size());
  for (std::size_t i = 0; i < distances.size(); ++i) {
    const double distance = distances[i];
    const SolveResult solved = SolveForDistance(config, distance);

    SweepPoint point;
    point.distance = distance;
    point.hasSolution = solved.hasSolution;

    if (solved.hasSolution) {
      point.bestThetaRad = solved.bestThetaRad;
      point.bestVNominal = solved.bestVNominal;
      point.bestVLow = solved.bestVLow;
      point.bestVHigh = solved.bestVHigh;
      point.bestDeltaV = solved.bestDeltaV;
      point.timeOfFlight = solved.nominalIntersection.tEntry;
      point.entryAngleRad = solved.nominalIntersection.entryAngleRad;
      point.zApex = solved.nominalApex.zApex;
    } else {
      point.failureReason = solved.failureReason;
    }

    result.points.push_back(point);

    if (progressCallback) {
      const double progress = static_cast<double>(i + 1U) / static_cast<double>(distances.size());
      progressCallback(progress);
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.totalTimeMs =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  {
    std::lock_guard<std::mutex> lock(gSweepCacheMutex);
    if (gSweepCache.size() > 8U) {
      gSweepCache.clear();
    }
    gSweepCache[cacheKey] = result;
  }

  return result;
}

}  // namespace shootersim::model
