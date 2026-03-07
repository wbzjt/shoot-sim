#pragma once

#include <optional>
#include <string>
#include <vector>

#include "Types.h"

namespace calibration {

class ModelFitter {
 public:
  std::vector<ModelReport> Fit(const std::vector<AggregatedPoint>& aggregated,
                               const FitterConfig& config,
                               std::string* warning) const;

  static std::vector<FitDatasetPoint> BuildFitDataset(const std::vector<AggregatedPoint>& aggregated);

  static std::optional<ModelReport> PickBestSafe(const std::vector<ModelReport>& reports);
  static std::optional<ModelReport> PickBestAccuracy(const std::vector<ModelReport>& reports);
};

}  // namespace calibration
