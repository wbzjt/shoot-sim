#include "util/MathUtil.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace shootersim::util {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double DegToRad(double deg) { return deg * kPi / 180.0; }

double RadToDeg(double rad) { return rad * 180.0 / kPi; }

double Clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

bool NearlyEqual(double a, double b, double eps) {
  return std::fabs(a - b) <= eps;
}

std::vector<double> Linspace(double start, double end, double step) {
  std::vector<double> out;
  if (step <= 0.0) {
    return out;
  }
  for (double value = start; value <= end + step * 0.5; value += step) {
    out.push_back(value > end ? end : value);
    if (out.back() >= end) {
      break;
    }
  }
  if (out.empty()) {
    out.push_back(start);
  }
  return out;
}

std::size_t HashCombine(std::size_t seed, std::size_t value) {
  // 64-bit hash combine.
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}

std::size_t HashDouble(std::size_t seed, double value, double scale) {
  const auto scaled = static_cast<std::int64_t>(std::llround(value * scale));
  const auto hashed = std::hash<std::int64_t>{}(scaled);
  return HashCombine(seed, hashed);
}

std::string FormatDouble(double value, int precision) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

}  // namespace shootersim::util
