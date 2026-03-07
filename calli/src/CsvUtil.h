#pragma once

#include <map>
#include <string>
#include <vector>

namespace calibration::csv {

struct Table {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
};

bool ReadCsv(const std::string& path, Table* out, std::string* error);
bool WriteCsv(const std::string& path,
              const std::vector<std::string>& header,
              const std::vector<std::vector<std::string>>& rows,
              std::string* error);

std::map<std::string, int> BuildHeaderIndex(const std::vector<std::string>& header);
int FindHeaderColumn(const std::map<std::string, int>& index,
                     const std::vector<std::string>& aliases);

}  // namespace calibration::csv
