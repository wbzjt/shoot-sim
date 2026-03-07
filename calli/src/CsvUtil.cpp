#include "CsvUtil.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "MathUtil.h"

namespace calibration::csv {
namespace {

std::vector<std::string> ParseCsvLine(const std::string& line) {
  std::vector<std::string> out;
  std::string field;
  bool inQuotes = false;

  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (inQuotes) {
      if (c == '"') {
        if (i + 1U < line.size() && line[i + 1U] == '"') {
          field.push_back('"');
          ++i;
        } else {
          inQuotes = false;
        }
      } else {
        field.push_back(c);
      }
      continue;
    }

    if (c == ',') {
      out.push_back(field);
      field.clear();
    } else if (c == '"') {
      inQuotes = true;
    } else {
      field.push_back(c);
    }
  }
  out.push_back(field);
  return out;
}

std::string EscapeCsvField(const std::string& field) {
  const bool need = field.find(',') != std::string::npos ||
                    field.find('"') != std::string::npos ||
                    field.find('\n') != std::string::npos ||
                    field.find('\r') != std::string::npos;
  if (!need) {
    return field;
  }
  std::string out;
  out.push_back('"');
  for (char c : field) {
    if (c == '"') {
      out.push_back('"');
    }
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

std::string JoinCsv(const std::vector<std::string>& row) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < row.size(); ++i) {
    if (i > 0U) {
      oss << ',';
    }
    oss << EscapeCsvField(row[i]);
  }
  return oss.str();
}

}  // namespace

bool ReadCsv(const std::string& path, Table* out, std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) {
      *error = "CSV output pointer is null";
    }
    return false;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open CSV: " + std::filesystem::absolute(path).string();
    }
    return false;
  }

  out->header.clear();
  out->rows.clear();

  std::string line;
  bool first = true;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    auto row = ParseCsvLine(line);
    if (first) {
      out->header = std::move(row);
      first = false;
      continue;
    }
    if (row.size() < out->header.size()) {
      row.resize(out->header.size());
    }
    out->rows.push_back(std::move(row));
  }

  if (out->header.empty()) {
    if (error != nullptr) {
      *error = "CSV has empty header: " + path;
    }
    return false;
  }

  return true;
}

bool WriteCsv(const std::string& path,
              const std::vector<std::string>& header,
              const std::vector<std::vector<std::string>>& rows,
              std::string* error) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open output CSV: " + path;
    }
    return false;
  }

  file << JoinCsv(header) << '\n';
  for (const auto& row : rows) {
    file << JoinCsv(row) << '\n';
  }

  if (!file.good()) {
    if (error != nullptr) {
      *error = "Failed to write CSV: " + path;
    }
    return false;
  }

  return true;
}

std::map<std::string, int> BuildHeaderIndex(const std::vector<std::string>& header) {
  std::map<std::string, int> index;
  for (std::size_t i = 0; i < header.size(); ++i) {
    const std::string key = math::ToLower(math::Trim(header[i]));
    if (!key.empty()) {
      index[key] = static_cast<int>(i);
    }
  }
  return index;
}

int FindHeaderColumn(const std::map<std::string, int>& index,
                     const std::vector<std::string>& aliases) {
  for (const auto& alias : aliases) {
    const auto it = index.find(math::ToLower(math::Trim(alias)));
    if (it != index.end()) {
      return it->second;
    }
  }
  return -1;
}

}  // namespace calibration::csv
