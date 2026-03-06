#include "util/CsvUtil.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cerrno>
#include <cstring>

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

std::string AbsolutePathOrOriginal(const std::string& path) {
  try {
    return std::filesystem::absolute(std::filesystem::path(path)).string();
  } catch (...) {
    return path;
  }
}

bool TryWriteCsvFile(const std::string& path,
                     const std::vector<std::string>& header,
                     const std::vector<std::vector<std::string>>& rows,
                     std::string* error) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open CSV path: " + AbsolutePathOrOriginal(path) +
               " (" + std::strerror(errno) + ")";
    }
    return false;
  }

  file << JoinCsvLine(header) << '\n';
  for (const auto& row : rows) {
    file << JoinCsvLine(row) << '\n';
  }

  if (!file.good()) {
    if (error != nullptr) {
      *error = "Failed to write CSV file: " + AbsolutePathOrOriginal(path) +
               " (" + std::strerror(errno) + ")";
    }
    return false;
  }

  return true;
}

std::string BuildTimestampedPath(const std::string& path) {
  const std::filesystem::path p(path);
  const std::filesystem::path parent = p.parent_path();
  const std::string stem = p.stem().string();
  const std::string ext = p.extension().string();

  const auto now = std::chrono::system_clock::now();
  const std::time_t nowT = std::chrono::system_clock::to_time_t(now);
  std::tm localTm{};
#ifdef _WIN32
  localtime_s(&localTm, &nowT);
#else
  localtime_r(&nowT, &localTm);
#endif

  std::ostringstream ts;
  ts << std::put_time(&localTm, "%Y%m%d_%H%M%S");

  const std::string safeStem = stem.empty() ? "ShooterLookup" : stem;
  const std::filesystem::path fallback =
      parent / (safeStem + "_" + ts.str() + (ext.empty() ? ".csv" : ext));
  return fallback.string();
}

}  // namespace

bool WriteCsv(const std::string& path,
              const std::vector<std::string>& header,
              const std::vector<std::vector<std::string>>& rows,
              std::string* error) {
  if (error != nullptr) {
    error->clear();
  }

  std::string primaryError;
  if (TryWriteCsvFile(path, header, rows, &primaryError)) {
    return true;
  }

  const std::string fallbackPath = BuildTimestampedPath(path);
  std::string fallbackError;
  if (fallbackPath != path &&
      TryWriteCsvFile(fallbackPath, header, rows, &fallbackError)) {
    if (error != nullptr) {
      *error = "Primary CSV path unavailable, wrote fallback file: " +
               AbsolutePathOrOriginal(fallbackPath);
    }
    return true;
  }

  if (error != nullptr) {
    *error = primaryError;
    if (!fallbackError.empty()) {
      *error += " | Fallback failed: " + fallbackError;
    }
  }

  return false;
}

}  // namespace shootersim::util
