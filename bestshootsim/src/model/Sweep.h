#pragma once

#include <functional>
#include <string>

#include "model/Types.h"

namespace shootersim::model {

std::vector<double> BuildDistanceSamples(const SweepRequest& request);
std::string BuildSweepCacheKey(const SimulationConfig& config, const SweepRequest& request);

SweepResult RunSweep(const SimulationConfig& config,
                     const SweepRequest& request,
                     const std::function<void(double)>& progressCallback = {});

}  // namespace shootersim::model
