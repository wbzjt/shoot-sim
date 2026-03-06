#include "model/Geometry.h"

#include <sstream>

namespace shootersim::model {

TargetWindow ComputeRawWindow(const SimulationConfig& config, double distance) {
  TargetWindow window;
  switch (config.windowInputMode) {
    case WindowInputMode::kDirectFrontBackOffset:
      // The user-provided offsets are relative to the distance reference plane.
      window.xFront = distance + config.xFrontOffset;
      window.xBack = distance + config.xBackOffset;
      break;
    case WindowInputMode::kCenterAndDepth:
      window.xFront = distance - config.openingDepth * 0.5;
      window.xBack = distance + config.openingDepth * 0.5;
      break;
  }
  return window;
}

EffectiveWindow ComputeEffectiveWindow(const SimulationConfig& config, const TargetWindow& rawWindow) {
  EffectiveWindow effective;
  effective.xFrontEff = rawWindow.xFront + config.ballRadius + config.frontMargin;
  effective.xBackEff = rawWindow.xBack - config.ballRadius - config.backMargin;

  if (effective.xFrontEff < effective.xBackEff) {
    effective.valid = true;
    return effective;
  }

  std::ostringstream oss;
  oss << "Invalid effective window: front_eff(" << effective.xFrontEff
      << ") >= back_eff(" << effective.xBackEff << ")";
  effective.valid = false;
  effective.error = oss.str();
  return effective;
}

}  // namespace shootersim::model
