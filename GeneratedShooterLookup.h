#pragma once

#include <array>

namespace shootersim_generated {

struct ShooterPoint {
  double distance;
  double thetaDeg;
  double vNominal;
  double vLow;
  double vHigh;
};

// Example generated lookup table. Use GUI Export C++ Header to regenerate with live data.
constexpr std::array<ShooterPoint, 3> kShooterLookup = {{
    {2.0, 46.5, 8.20, 7.95, 8.55},
    {3.0, 51.2, 9.35, 9.05, 9.78},
    {4.0, 56.0, 10.70, 10.30, 11.15},
}};

}  // namespace shootersim_generated
