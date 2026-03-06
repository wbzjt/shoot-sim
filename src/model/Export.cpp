#include "model/Export.h"

#include <fstream>
#include <sstream>
#include <vector>

#include "util/CsvUtil.h"
#include "util/JsonUtil.h"
#include "util/MathUtil.h"

namespace shootersim::model {

bool ExportSweepToCsv(const std::string& path, const SweepResult& sweep, std::string* error) {
  const std::vector<std::string> header = {
      "distance_m",      "has_solution", "theta_deg",     "v_nominal_mps",
      "v_low_mps",       "v_high_mps",   "delta_v_mps",   "time_of_flight_s",
      "entry_angle_deg", "z_apex_m",     "failure_reason",
  };

  std::vector<std::vector<std::string>> rows;
  rows.reserve(sweep.points.size());
  for (const auto& p : sweep.points) {
    rows.push_back({
        util::FormatDouble(p.distance, 5),
        p.hasSolution ? "1" : "0",
        util::FormatDouble(util::RadToDeg(p.bestThetaRad), 4),
        util::FormatDouble(p.bestVNominal, 5),
        util::FormatDouble(p.bestVLow, 5),
        util::FormatDouble(p.bestVHigh, 5),
        util::FormatDouble(p.bestDeltaV, 5),
        util::FormatDouble(p.timeOfFlight, 5),
        util::FormatDouble(util::RadToDeg(p.entryAngleRad), 4),
        util::FormatDouble(p.zApex, 5),
        p.failureReason,
    });
  }

  return util::WriteCsv(path, header, rows, error);
}

bool ExportSweepToJson(const std::string& path,
                       const SimulationConfig& config,
                       const SweepResult& sweep,
                       std::string* error) {
  return util::SaveSweepJson(path, config, sweep, error);
}

bool ExportSweepToCppHeader(const std::string& path,
                            const SweepResult& sweep,
                            std::string* error,
                            const std::string& arrayName) {
  std::vector<SweepPoint> validPoints;
  for (const auto& p : sweep.points) {
    if (p.hasSolution) {
      validPoints.push_back(p);
    }
  }

  std::ostringstream oss;
  oss << "#pragma once\n\n";
  oss << "#include <array>\n\n";
  oss << "namespace shootersim_generated {\n\n";
  oss << "struct ShooterPoint {\n";
  oss << "  double distance;\n";
  oss << "  double thetaDeg;\n";
  oss << "  double vNominal;\n";
  oss << "  double vLow;\n";
  oss << "  double vHigh;\n";
  oss << "};\n\n";

  oss << "constexpr std::array<ShooterPoint, " << validPoints.size() << "> " << arrayName
      << " = {\n";
  for (const auto& p : validPoints) {
    oss << "  ShooterPoint{" << util::FormatDouble(p.distance, 6) << ", "
        << util::FormatDouble(util::RadToDeg(p.bestThetaRad), 6) << ", "
        << util::FormatDouble(p.bestVNominal, 6) << ", "
        << util::FormatDouble(p.bestVLow, 6) << ", "
        << util::FormatDouble(p.bestVHigh, 6) << "},\n";
  }
  oss << "};\n\n";
  oss << "}  // namespace shootersim_generated\n";

  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open header export path: " + path;
    }
    return false;
  }
  file << oss.str();

  if (!file.good()) {
    if (error != nullptr) {
      *error = "Failed to write header export file: " + path;
    }
    return false;
  }

  return true;
}

}  // namespace shootersim::model
