#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "model/Search.h"
#include "util/MathUtil.h"

namespace {

double ParseOrDefault(const char* text, double fallback) {
  if (text == nullptr) {
    return fallback;
  }
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    return fallback;
  }
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  const double distance = (argc >= 2) ? ParseOrDefault(argv[1], 1.905) : 1.905;
  const double thetaMinDeg = (argc >= 3) ? ParseOrDefault(argv[2], -1.0) : -1.0;
  const double thetaMaxDeg = (argc >= 4) ? ParseOrDefault(argv[3], thetaMinDeg) : thetaMinDeg;

  shootersim::SimulationConfig config;
  const shootersim::SolveResult result = shootersim::model::SolveForDistance(config, distance);

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "distance_m=" << distance << "\n";
  std::cout << "has_solution=" << (result.hasSolution ? 1 : 0) << "\n";
  if (!result.hasSolution) {
    std::cout << "failure_reason=" << result.failureReason << "\n";
    return 0;
  }

  std::cout << "best_theta_deg=" << shootersim::util::RadToDeg(result.bestThetaRad) << "\n";
  std::cout << "best_v_nominal=" << result.bestVNominal << "\n";
  std::cout << "best_v_low=" << result.bestVLow << "\n";
  std::cout << "best_v_high=" << result.bestVHigh << "\n";
  std::cout << "best_delta_v=" << result.bestDeltaV << "\n";
  std::cout << "theta_count=" << result.thetaScan.size() << "\n";

  struct ThetaRow {
    double thetaDeg = 0.0;
    double vLow = 0.0;
    double vHigh = 0.0;
    double deltaV = 0.0;
    double vNominal = 0.0;
  };

  std::vector<ThetaRow> validRows;
  for (const auto& scan : result.thetaScan) {
    if (!scan.hasValidInterval) {
      continue;
    }
    validRows.push_back({
        shootersim::util::RadToDeg(scan.thetaRad),
        scan.bestInterval.vLow,
        scan.bestInterval.vHigh,
        scan.bestInterval.deltaV,
        scan.vNominal,
    });
  }

  std::sort(validRows.begin(), validRows.end(), [](const ThetaRow& a, const ThetaRow& b) {
    if (a.deltaV != b.deltaV) {
      return a.deltaV > b.deltaV;
    }
    return a.thetaDeg > b.thetaDeg;
  });

  std::cout << "valid_theta_rows=" << validRows.size() << "\n";
  std::cout << "top_valid_thetas:\n";
  const std::size_t topN = std::min<std::size_t>(12U, validRows.size());
  for (std::size_t i = 0; i < topN; ++i) {
    const auto& row = validRows[i];
    std::cout << "  theta_deg=" << row.thetaDeg
              << " v_low=" << row.vLow
              << " v_high=" << row.vHigh
              << " delta_v=" << row.deltaV
              << " v_nominal=" << row.vNominal << "\n";
  }

  if (thetaMinDeg >= 0.0 && thetaMaxDeg >= thetaMinDeg) {
    std::cout << "theta_window:\n";
    for (const auto& row : validRows) {
      if (row.thetaDeg >= thetaMinDeg && row.thetaDeg <= thetaMaxDeg) {
        std::cout << "  theta_deg=" << row.thetaDeg
                  << " v_low=" << row.vLow
                  << " v_high=" << row.vHigh
                  << " delta_v=" << row.deltaV
                  << " v_nominal=" << row.vNominal << "\n";
        }
    }
  }

  return 0;
}
