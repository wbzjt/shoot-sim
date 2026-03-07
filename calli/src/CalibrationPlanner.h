#pragma once

#include <string>
#include <vector>

#include "Types.h"

namespace calibration {

class CalibrationPlanner {
 public:
  std::vector<RecommendedPoint> Recommend(const std::vector<TheoryPoint>& theory,
                                          const PlannerConfig& config,
                                          PlannerSummary* summary,
                                          std::vector<RecommendedPoint>* allScores,
                                          std::string* warning) const;
};

}  // namespace calibration
