#include "MathUtil.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <limits>

namespace calibration::math {

std::string ToLower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

std::string Trim(const std::string& s) {
  std::size_t start = 0;
  while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
    ++start;
  }
  std::size_t end = s.size();
  while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(start, end - start);
}

bool ParseDouble(const std::string& s, double* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string t = Trim(s);
  if (t.empty()) {
    return false;
  }
  char* end = nullptr;
  const double v = std::strtod(t.c_str(), &end);
  if (end == t.c_str() || *end != '\0' || !std::isfinite(v)) {
    return false;
  }
  *out = v;
  return true;
}

std::optional<double> ParseOptionalDouble(const std::string& s) {
  double v = 0.0;
  if (!ParseDouble(s, &v)) {
    return std::nullopt;
  }
  return v;
}

bool ParseBoolLoose(const std::string& s, bool* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string t = ToLower(Trim(s));
  if (t == "1" || t == "true" || t == "yes" || t == "hit" || t == "y") {
    *out = true;
    return true;
  }
  if (t == "0" || t == "false" || t == "no" || t == "miss" || t == "n") {
    *out = false;
    return true;
  }
  return false;
}

std::vector<double> SortCopy(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  return values;
}

double Mean(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (double v : values) {
    sum += v;
  }
  return sum / static_cast<double>(values.size());
}

double Median(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  std::vector<double> s = SortCopy(values);
  const std::size_t n = s.size();
  if (n % 2U == 1U) {
    return s[n / 2U];
  }
  return 0.5 * (s[n / 2U - 1U] + s[n / 2U]);
}

double StdDev(const std::vector<double>& values) {
  if (values.size() < 2U) {
    return 0.0;
  }
  const double mu = Mean(values);
  double ss = 0.0;
  for (double v : values) {
    const double d = v - mu;
    ss += d * d;
  }
  return std::sqrt(ss / static_cast<double>(values.size() - 1U));
}

double Quantile(const std::vector<double>& values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  if (q <= 0.0) {
    return *std::min_element(values.begin(), values.end());
  }
  if (q >= 1.0) {
    return *std::max_element(values.begin(), values.end());
  }
  std::vector<double> s = SortCopy(values);
  const double pos = q * static_cast<double>(s.size() - 1U);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
  const double t = pos - static_cast<double>(lo);
  return Lerp(s[lo], s[hi], t);
}

double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double Lerp(double a, double b, double t) {
  return a + (b - a) * t;
}

double LinearInterpolate(const std::vector<double>& xs,
                         const std::vector<double>& ys,
                         double x,
                         bool clamp) {
  if (xs.empty() || ys.empty() || xs.size() != ys.size()) {
    return 0.0;
  }
  if (xs.size() == 1U) {
    return ys.front();
  }

  if (x <= xs.front()) {
    if (clamp) {
      return ys.front();
    }
    const double t = (x - xs[0]) / (xs[1] - xs[0]);
    return Lerp(ys[0], ys[1], t);
  }

  if (x >= xs.back()) {
    if (clamp) {
      return ys.back();
    }
    const std::size_t n = xs.size();
    const double t = (x - xs[n - 2U]) / (xs[n - 1U] - xs[n - 2U]);
    return Lerp(ys[n - 2U], ys[n - 1U], t);
  }

  const auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const std::size_t hi = static_cast<std::size_t>(it - xs.begin());
  const std::size_t lo = hi - 1U;
  const double t = (x - xs[lo]) / (xs[hi] - xs[lo]);
  return Lerp(ys[lo], ys[hi], t);
}

double RoundTo(double value, double step) {
  if (step <= 0.0) {
    return value;
  }
  return std::round(value / step) * step;
}

std::vector<double> SolveLinearSystem(std::vector<std::vector<double>> a,
                                      std::vector<double> b,
                                      bool* ok) {
  const std::size_t n = a.size();
  if (ok != nullptr) {
    *ok = false;
  }
  if (n == 0U || b.size() != n) {
    return {};
  }
  for (const auto& row : a) {
    if (row.size() != n) {
      return {};
    }
  }

  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    double maxAbs = std::fabs(a[col][col]);
    for (std::size_t r = col + 1U; r < n; ++r) {
      const double v = std::fabs(a[r][col]);
      if (v > maxAbs) {
        maxAbs = v;
        pivot = r;
      }
    }
    if (maxAbs < 1e-12) {
      return {};
    }
    if (pivot != col) {
      std::swap(a[pivot], a[col]);
      std::swap(b[pivot], b[col]);
    }

    const double div = a[col][col];
    for (std::size_t c = col; c < n; ++c) {
      a[col][c] /= div;
    }
    b[col] /= div;

    for (std::size_t r = 0; r < n; ++r) {
      if (r == col) {
        continue;
      }
      const double f = a[r][col];
      if (std::fabs(f) < 1e-14) {
        continue;
      }
      for (std::size_t c = col; c < n; ++c) {
        a[r][c] -= f * a[col][c];
      }
      b[r] -= f * b[col];
    }
  }

  if (ok != nullptr) {
    *ok = true;
  }
  return b;
}

}  // namespace calibration::math
