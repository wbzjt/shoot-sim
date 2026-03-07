#pragma once

#include <vector>

#include "Types.h"

namespace calibration {

class SegmentAnalyzer {
 public:
  SegmentRecommendation Analyze(const std::vector<FitDatasetPoint>& dataset,
                                const std::vector<ModelReport>& reports,
                                const SegmentAnalyzerConfig& config) const;
};

}  // namespace calibration
