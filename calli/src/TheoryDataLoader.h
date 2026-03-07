#pragma once

#include <string>
#include <vector>

#include "Types.h"

namespace calibration {

class TheoryDataLoader {
 public:
  bool LoadFromCsv(const std::string& path,
                   std::vector<TheoryPoint>* out,
                   std::string* error) const;
};

}  // namespace calibration
