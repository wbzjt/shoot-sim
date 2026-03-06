#include "model/Ballistics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace shootersim::model {
namespace {

constexpr double kEps = 1e-9;

struct NumericState {
  double t = 0.0;
  double x = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vz = 0.0;
};

NumericState InitState(const SimulationConfig& config, double thetaRad, double v0) {
  NumericState state;
  state.t = 0.0;
  state.x = 0.0;
  state.z = config.releaseHeight;
  state.vx = v0 * std::cos(thetaRad);
  state.vz = v0 * std::sin(thetaRad);
  return state;
}

NumericState StepSemiImplicitEuler(const SimulationConfig& config, const NumericState& s, double dt) {
  NumericState next = s;

  const double k = std::max(0.0, config.dragCoefficient);
  const double speed = std::sqrt(s.vx * s.vx + s.vz * s.vz);
  const double ax = -k * speed * s.vx;
  const double az = -config.gravity - k * speed * s.vz;

  next.vx = s.vx + ax * dt;
  next.vz = s.vz + az * dt;
  next.x = s.x + next.vx * dt;
  next.z = s.z + next.vz * dt;
  next.t = s.t + dt;
  return next;
}

TrajectorySample ToSample(const NumericState& state) {
  TrajectorySample s;
  s.t = state.t;
  s.x = state.x;
  s.z = state.z;
  s.vx = state.vx;
  s.vz = state.vz;
  return s;
}

IntersectionResult BuildIntersectionFromState(const NumericState& s, double zTarget) {
  IntersectionResult out;
  out.hasIntersection = true;
  out.descending = s.vz < 0.0;
  out.tEntry = s.t;
  out.xEntry = s.x;
  out.zEntry = zTarget;
  out.vxEntry = s.vx;
  out.vzEntry = s.vz;
  out.entrySpeed = std::sqrt(s.vx * s.vx + s.vz * s.vz);
  out.entryAngleRad = std::atan2(s.vz, s.vx);
  return out;
}

bool SolveLinear(double b, double c, double* t) {
  if (std::fabs(b) < kEps) {
    return false;
  }
  *t = -c / b;
  return true;
}

}  // namespace

TrajectorySample EvaluateAtTime(const SimulationConfig& config, double thetaRad, double v0, double t) {
  if (config.physicsMode == PhysicsMode::kAnalyticNoDrag) {
    TrajectorySample sample;
    sample.t = t;
    sample.x = v0 * std::cos(thetaRad) * t;
    sample.z = config.releaseHeight + v0 * std::sin(thetaRad) * t - 0.5 * config.gravity * t * t;
    sample.vx = v0 * std::cos(thetaRad);
    sample.vz = v0 * std::sin(thetaRad) - config.gravity * t;
    return sample;
  }

  const double dt = std::max(1e-4, config.dragDt);
  NumericState state = InitState(config, thetaRad, v0);
  double remaining = std::max(0.0, t);
  while (remaining > kEps) {
    const double step = std::min(dt, remaining);
    state = StepSemiImplicitEuler(config, state, step);
    remaining -= step;
  }
  return ToSample(state);
}

IntersectionResult ComputeIntersectionAtHeight(const SimulationConfig& config,
                                               double thetaRad,
                                               double v0,
                                               double zTarget) {
  if (config.physicsMode == PhysicsMode::kAnalyticNoDrag) {
    const double a = -0.5 * config.gravity;
    const double b = v0 * std::sin(thetaRad);
    const double c = config.releaseHeight - zTarget;

    if (std::fabs(a) < kEps) {
      double t = 0.0;
      if (!SolveLinear(b, c, &t) || t <= 0.0) {
        return {};
      }
      const TrajectorySample state = EvaluateAtTime(config, thetaRad, v0, t);
      if (state.vz >= 0.0) {
        return {};
      }
      IntersectionResult out;
      out.hasIntersection = true;
      out.descending = true;
      out.tEntry = t;
      out.xEntry = state.x;
      out.zEntry = zTarget;
      out.vxEntry = state.vx;
      out.vzEntry = state.vz;
      out.entrySpeed = std::sqrt(state.vx * state.vx + state.vz * state.vz);
      out.entryAngleRad = std::atan2(state.vz, state.vx);
      return out;
    }

    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return {};
    }

    const double sqrtD = std::sqrt(std::max(0.0, discriminant));
    const double denom = 2.0 * a;
    const double t1 = (-b - sqrtD) / denom;
    const double t2 = (-b + sqrtD) / denom;

    double bestT = -1.0;
    for (double tCandidate : {t1, t2}) {
      if (tCandidate <= kEps) {
        continue;
      }
      const double vz = v0 * std::sin(thetaRad) - config.gravity * tCandidate;
      if (vz < 0.0 && tCandidate > bestT) {
        bestT = tCandidate;
      }
    }

    if (bestT <= 0.0) {
      return {};
    }

    const TrajectorySample state = EvaluateAtTime(config, thetaRad, v0, bestT);
    IntersectionResult out;
    out.hasIntersection = true;
    out.descending = true;
    out.tEntry = bestT;
    out.xEntry = state.x;
    out.zEntry = zTarget;
    out.vxEntry = state.vx;
    out.vzEntry = state.vz;
    out.entrySpeed = std::sqrt(state.vx * state.vx + state.vz * state.vz);
    out.entryAngleRad = std::atan2(state.vz, state.vx);
    return out;
  }

  const double dt = std::max(1e-4, config.dragDt);
  const double tMax = std::max(0.1, config.dragMaxTime);
  NumericState prev = InitState(config, thetaRad, v0);
  NumericState cur = prev;

  while (cur.t < tMax) {
    cur = StepSemiImplicitEuler(config, prev, dt);

    const double prevDiff = prev.z - zTarget;
    const double curDiff = cur.z - zTarget;
    if (prevDiff * curDiff <= 0.0 && std::fabs(curDiff - prevDiff) > kEps) {
      const double alpha = (zTarget - prev.z) / (cur.z - prev.z);
      NumericState hit;
      hit.t = prev.t + alpha * (cur.t - prev.t);
      hit.x = prev.x + alpha * (cur.x - prev.x);
      hit.z = zTarget;
      hit.vx = prev.vx + alpha * (cur.vx - prev.vx);
      hit.vz = prev.vz + alpha * (cur.vz - prev.vz);
      if (hit.vz < 0.0) {
        return BuildIntersectionFromState(hit, zTarget);
      }
    }

    if (cur.z < -1.0 && cur.vz < 0.0) {
      break;
    }
    prev = cur;
  }

  return {};
}

ApexResult ComputeApex(const SimulationConfig& config, double thetaRad, double v0) {
  if (config.physicsMode == PhysicsMode::kAnalyticNoDrag) {
    ApexResult apex;
    const double vz0 = v0 * std::sin(thetaRad);
    if (vz0 <= 0.0) {
      apex.valid = true;
      apex.tApex = 0.0;
      apex.xApex = 0.0;
      apex.zApex = config.releaseHeight;
      return apex;
    }

    const double tApex = vz0 / config.gravity;
    const TrajectorySample state = EvaluateAtTime(config, thetaRad, v0, tApex);
    apex.valid = true;
    apex.tApex = tApex;
    apex.xApex = state.x;
    apex.zApex = state.z;
    return apex;
  }

  const double dt = std::max(1e-4, config.dragDt);
  const double tMax = std::max(0.1, config.dragMaxTime);
  NumericState state = InitState(config, thetaRad, v0);

  ApexResult apex;
  apex.valid = true;
  apex.tApex = state.t;
  apex.xApex = state.x;
  apex.zApex = state.z;

  while (state.t < tMax) {
    state = StepSemiImplicitEuler(config, state, dt);
    if (state.z > apex.zApex) {
      apex.tApex = state.t;
      apex.xApex = state.x;
      apex.zApex = state.z;
    }
    if (state.vz < 0.0 && state.z < apex.zApex - 0.5) {
      break;
    }
  }
  return apex;
}

double EstimateFlightTimeToGround(const SimulationConfig& config, double thetaRad, double v0) {
  if (config.physicsMode == PhysicsMode::kAnalyticNoDrag) {
    const IntersectionResult hitGround = ComputeIntersectionAtHeight(config, thetaRad, v0, 0.0);
    if (hitGround.hasIntersection) {
      return hitGround.tEntry;
    }
    return 2.0;
  }

  const double dt = std::max(1e-4, config.dragDt);
  const double tMax = std::max(0.1, config.dragMaxTime);
  NumericState prev = InitState(config, thetaRad, v0);
  NumericState cur = prev;

  while (cur.t < tMax) {
    cur = StepSemiImplicitEuler(config, prev, dt);
    if (prev.z >= 0.0 && cur.z <= 0.0 && std::fabs(cur.z - prev.z) > kEps) {
      const double alpha = (0.0 - prev.z) / (cur.z - prev.z);
      return prev.t + alpha * (cur.t - prev.t);
    }
    prev = cur;
  }
  return tMax;
}

std::vector<TrajectorySample> BuildTrajectory(const SimulationConfig& config,
                                              double thetaRad,
                                              double v0,
                                              int samples,
                                              double tMaxOverride) {
  std::vector<TrajectorySample> out;
  samples = std::max(2, samples);

  if (config.physicsMode == PhysicsMode::kAnalyticNoDrag) {
    const double tEnd = (tMaxOverride > 0.0)
                            ? tMaxOverride
                            : std::max(EstimateFlightTimeToGround(config, thetaRad, v0) + 0.1, 0.5);
    out.reserve(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(samples - 1);
      const double t = alpha * tEnd;
      out.push_back(EvaluateAtTime(config, thetaRad, v0, t));
    }
    return out;
  }

  const double dt = std::max(1e-4, config.dragDt);
  const double tEnd = (tMaxOverride > 0.0) ? tMaxOverride : std::max(0.5, config.dragMaxTime);
  NumericState state = InitState(config, thetaRad, v0);
  std::vector<TrajectorySample> raw;
  raw.reserve(static_cast<std::size_t>(tEnd / dt) + 4);
  raw.push_back(ToSample(state));

  while (state.t < tEnd) {
    state = StepSemiImplicitEuler(config, state, dt);
    raw.push_back(ToSample(state));
    if (state.z < -0.2 && state.vz < 0.0) {
      break;
    }
  }

  if (raw.size() <= static_cast<std::size_t>(samples)) {
    return raw;
  }

  out.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(samples - 1);
    const std::size_t idx = static_cast<std::size_t>(
        alpha * static_cast<double>(raw.size() - 1));
    out.push_back(raw[idx]);
  }
  return out;
}

}  // namespace shootersim::model
