#pragma once

#include <string>
#include <vector>

#include "Types.h"

namespace calibration {

class CalibrationAggregator {
 public:
  std::vector<AggregatedPoint> Aggregate(const std::vector<CalibrationSample>& samples,
                                         const AggregationConfig& config,
                                         std::string* warning) const;
};

}  // namespace calibration
