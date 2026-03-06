#pragma once

#include "model/Types.h"

namespace shootersim::gui {

bool DragAngleDeg(const char* label,
                  double* angleRad,
                  double speedDeg,
                  double minDeg,
                  double maxDeg,
                  const char* format = "%.2f deg");

void DrawConstraintStatus(const shootersim::ConstraintResult& result);
void DrawSolveSummary(const shootersim::SolveResult& result);

}  // namespace shootersim::gui
