#include "util/CsvUtil.h"

#include <fstream>
#include <sstream>

namespace shootersim::util {
namespace {

std::string EscapeCsvField(const std::string& value) {
  const bool needsQuote = value.find(',') != std::string::npos ||
                          value.find('"') != std::string::npos ||
                          value.find('\n') != std::string::npos ||
                          value.find('\r') != std::string::npos;
  if (!needsQuote) {
    return value;
  }

  std::string out;
  out.reserve(value.size() + 2U);
  out.push_back('"');
  for (char c : value) {
    if (c == '"') {
      out.push_back('"');
    }
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

std::string JoinCsvLine(const std::vector<std::string>& fields) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i > 0U) {
      oss << ',';
    }
    oss << EscapeCsvField(fields[i]);
  }
  return oss.str();
}

}  // namespace

bool WriteCsv(const std::string& path,
              const std::vector<std::string>& header,
              const std::vector<std::vector<std::string>>& rows,
              std::string* error) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open CSV path: " + path;
    }
    return false;
  }

  file << JoinCsvLine(header) << '\n';
  for (const auto& row : rows) {
    file << JoinCsvLine(row) << '\n';
  }

  if (!file.good()) {
    if (error != nullptr) {
      *error = "Failed to write CSV file: " + path;
    }
    return false;
  }

  return true;
}

}  // namespace shootersim::util
