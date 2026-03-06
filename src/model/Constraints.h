#pragma once

#include "model/Types.h"

namespace shootersim::model {

ConstraintEvaluation EvaluateConstraints(const SimulationConfig& config,
                                        double distance,
                                        double thetaRad,
                                        double v0);

bool IsShotValid(const SimulationConfig& config, double distance, double thetaRad, double v0);

}  // namespace shootersim::model
