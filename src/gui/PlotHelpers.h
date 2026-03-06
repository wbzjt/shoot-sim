#pragma once

#include "model/Types.h"

namespace shootersim::gui {

void PlotTrajectory(const shootersim::SimulationConfig& config,
                    const shootersim::SolveResult& result);

void PlotThetaVHeatmap(const shootersim::SolveResult& result);

void PlotSweepCurves(const shootersim::SweepResult& result);

}  // namespace shootersim::gui
