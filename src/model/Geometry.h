#pragma once

#include "model/Types.h"

namespace shootersim::model {

TargetWindow ComputeRawWindow(const SimulationConfig& config, double distance);
EffectiveWindow ComputeEffectiveWindow(const SimulationConfig& config, const TargetWindow& rawWindow);

}  // namespace shootersim::model
