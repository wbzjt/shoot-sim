#pragma once

#include <string>

#include "model/Types.h"

namespace shootersim::model {

bool ExportSweepToCsv(const std::string& path, const SweepResult& sweep, std::string* error);
bool ExportSweepToJson(const std::string& path,
                       const SimulationConfig& config,
                       const SweepResult& sweep,
                       std::string* error);
bool ExportSweepToCppHeader(const std::string& path,
                            const SweepResult& sweep,
                            std::string* error,
                            const std::string& arrayName = "kShooterLookup");

}  // namespace shootersim::model
