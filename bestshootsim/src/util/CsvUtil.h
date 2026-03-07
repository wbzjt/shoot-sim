#pragma once

#include <string>
#include <vector>

namespace shootersim::util {

bool WriteCsv(const std::string& path,
              const std::vector<std::string>& header,
              const std::vector<std::vector<std::string>>& rows,
              std::string* error);

}  // namespace shootersim::util
