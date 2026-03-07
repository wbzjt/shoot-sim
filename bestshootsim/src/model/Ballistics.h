#pragma once

#include <vector>

#include "model/Types.h"

namespace shootersim::model {

TrajectorySample EvaluateAtTime(const SimulationConfig& config, double thetaRad, double v0, double t);
IntersectionResult ComputeIntersectionAtHeight(const SimulationConfig& config,
                                               double thetaRad,
                                               double v0,
                                               double zTarget);
ApexResult ComputeApex(const SimulationConfig& config, double thetaRad, double v0);

double EstimateFlightTimeToGround(const SimulationConfig& config, double thetaRad, double v0);
std::vector<TrajectorySample> BuildTrajectory(const SimulationConfig& config,
                                              double thetaRad,
                                              double v0,
                                              int samples,
                                              double tMaxOverride = -1.0);

}  // namespace shootersim::model
