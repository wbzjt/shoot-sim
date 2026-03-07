#pragma once

#include <string>
#include <vector>

#include "Types.h"

namespace calibration {

class CalibrationDataLoader {
 public:
  bool LoadFromCsv(const std::string& path,
                   std::vector<CalibrationSample>* out,
                   std::string* error) const;
};

}  // namespace calibration
