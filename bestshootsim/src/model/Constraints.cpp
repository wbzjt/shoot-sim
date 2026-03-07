#include "model/Constraints.h"

#include <cmath>

#include "model/Ballistics.h"
#include "model/Geometry.h"

namespace shootersim::model {

ConstraintEvaluation EvaluateConstraints(const SimulationConfig& config,
                                        double distance,
                                        double thetaRad,
                                        double v0) {
  ConstraintEvaluation eval;
  eval.rawWindow = ComputeRawWindow(config, distance);
  eval.effectiveWindow = ComputeEffectiveWindow(config, eval.rawWindow);

  ConstraintResult& r = eval.result;
  r.pitchInRange = (thetaRad >= config.thetaMinRad && thetaRad <= config.thetaMaxRad);
  r.speedInRange = (v0 >= config.vMin && v0 <= config.vMax);
  r.effectiveWindowValid = eval.effectiveWindow.valid;

  if (!r.pitchInRange) {
    r.failureReasons.emplace_back("Pitch out of mechanical range");
  }
  if (!r.speedInRange) {
    r.failureReasons.emplace_back("Shooter speed out of range");
  }
  if (!r.effectiveWindowValid) {
    r.failureReasons.emplace_back(eval.effectiveWindow.error);
  }

  eval.apex = ComputeApex(config, thetaRad, v0);
  r.ceilingOk = eval.apex.valid && (eval.apex.zApex <= config.zCeilingMax);
  if (!r.ceilingOk) {
    r.failureReasons.emplace_back("Apex exceeds ceiling constraint");
  }

  eval.intersection = ComputeIntersectionAtHeight(config, thetaRad, v0, config.zTarget);
  r.hasDescendingIntersection = eval.intersection.hasIntersection && eval.intersection.descending;
  if (!r.hasDescendingIntersection) {
    r.failureReasons.emplace_back("No valid descending intersection with target plane");
  }

  if (r.hasDescendingIntersection && r.effectiveWindowValid) {
    const double x = eval.intersection.xEntry;
    r.inEffectiveWindow =
        (x >= eval.effectiveWindow.xFrontEff && x <= eval.effectiveWindow.xBackEff);
    if (!r.inEffectiveWindow) {
      r.failureReasons.emplace_back("Entry x is outside effective target window");
    }
  } else {
    r.inEffectiveWindow = false;
  }

  if (config.enableEntryAngleConstraint && r.hasDescendingIntersection) {
    r.entryAngleOk =
        std::fabs(eval.intersection.entryAngleRad) >= config.minAbsEntryAngleRad;
    if (!r.entryAngleOk) {
      r.failureReasons.emplace_back("Entry angle is too shallow");
    }
  }

  if (config.enableFlightTimeConstraint && r.hasDescendingIntersection) {
    r.flightTimeOk = eval.intersection.tEntry <= config.maxFlightTime;
    if (!r.flightTimeOk) {
      r.failureReasons.emplace_back("Flight time exceeds limit");
    }
  }

  r.overallValid = r.pitchInRange && r.speedInRange && r.effectiveWindowValid && r.ceilingOk &&
                   r.hasDescendingIntersection && r.inEffectiveWindow && r.entryAngleOk &&
                   r.flightTimeOk;

  return eval;
}

bool IsShotValid(const SimulationConfig& config, double distance, double thetaRad, double v0) {
  return EvaluateConstraints(config, distance, thetaRad, v0).result.overallValid;
}

}  // namespace shootersim::model
