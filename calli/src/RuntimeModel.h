#pragma once

#include <vector>

#include "Types.h"

namespace calibration::runtime {

double EvaluatePolynomial(const std::vector<double>& coeffs, double x);

double EvaluateModel(const RuntimeModelData& model, double theoryVelMps);

bool IsModelMonotonic(const RuntimeModelData& model,
                      double xMin,
                      double xMax,
                      int sampleCount,
                      bool allowTinyReverse);

}  // namespace calibration::runtime
