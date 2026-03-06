#pragma once

#include <string>

#include "model/Types.h"

namespace shootersim::util {

bool SaveConfigJson(const std::string& path, const SimulationConfig& config, std::string* error);
bool LoadConfigJson(const std::string& path, SimulationConfig* config, std::string* error);

bool SaveSweepJson(const std::string& path,
                   const SimulationConfig& config,
                   const SweepResult& sweep,
                   std::string* error);

bool LoadSweepJson(const std::string& path,
                   SimulationConfig* config,
                   SweepResult* sweep,
                   std::string* error);

}  // namespace shootersim::util
