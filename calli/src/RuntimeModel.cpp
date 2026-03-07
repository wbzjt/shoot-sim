#include "RuntimeModel.h"

#include <cmath>

#include "MathUtil.h"

namespace calibration::runtime {

double EvaluatePolynomial(const std::vector<double>& coeffs, double x) {
  double y = 0.0;
  double p = 1.0;
  for (double c : coeffs) {
    y += c * p;
    p *= x;
  }
  return y;
}

double EvaluateModel(const RuntimeModelData& model, double theoryVelMps) {
  switch (model.kind) {
    case ModelKind::kLookupLinear:
      return math::LinearInterpolate(model.knotsX, model.knotsY, theoryVelMps, model.clamp);
    case ModelKind::kSegmentedLinear2:
    case ModelKind::kSegmentedLinear3:
      return math::LinearInterpolate(model.knotsX, model.knotsY, theoryVelMps, model.clamp);
    case ModelKind::kPolynomial2:
    case ModelKind::kPolynomial3:
    case ModelKind::kPolynomial4:
      return EvaluatePolynomial(model.polyCoefficients, theoryVelMps);
  }
  return 0.0;
}

bool IsModelMonotonic(const RuntimeModelData& model,
                      double xMin,
                      double xMax,
                      int sampleCount,
                      bool allowTinyReverse) {
  if (sampleCount < 3 || xMax <= xMin) {
    return true;
  }
  const double eps = allowTinyReverse ? 1e-4 : 0.0;
  double prev = EvaluateModel(model, xMin);
  for (int i = 1; i < sampleCount; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(sampleCount - 1);
    const double x = math::Lerp(xMin, xMax, alpha);
    const double y = EvaluateModel(model, x);
    if (y + eps < prev) {
      return false;
    }
    prev = y;
  }
  return true;
}

}  // namespace calibration::runtime
