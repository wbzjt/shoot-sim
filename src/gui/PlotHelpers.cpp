#include "gui/PlotHelpers.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <implot.h>

#include "model/Ballistics.h"
#include "util/MathUtil.h"

namespace shootersim::gui {
namespace {

void BuildXZVectors(const std::vector<shootersim::TrajectorySample>& trajectory,
                    std::vector<double>* x,
                    std::vector<double>* z) {
  x->clear();
  z->clear();
  x->reserve(trajectory.size());
  z->reserve(trajectory.size());
  for (const auto& s : trajectory) {
    x->push_back(s.x);
    z->push_back(s.z);
  }
}

}  // namespace

void PlotTrajectory(const shootersim::SimulationConfig& config,
                    const shootersim::SolveResult& result) {
  if (!result.hasSolution) {
    ImGui::TextWrapped("No feasible trajectory under current constraints.");
    return;
  }

  const auto trajNominal = model::BuildTrajectory(config, result.bestThetaRad, result.bestVNominal,
                                                  config.trajectorySamples);
  const auto trajLow = model::BuildTrajectory(config, result.bestThetaRad, result.bestVLow,
                                              config.trajectorySamples);
  const auto trajHigh = model::BuildTrajectory(config, result.bestThetaRad, result.bestVHigh,
                                               config.trajectorySamples);

  std::vector<double> xNominal;
  std::vector<double> zNominal;
  std::vector<double> xLow;
  std::vector<double> zLow;
  std::vector<double> xHigh;
  std::vector<double> zHigh;
  BuildXZVectors(trajNominal, &xNominal, &zNominal);
  BuildXZVectors(trajLow, &xLow, &zLow);
  BuildXZVectors(trajHigh, &xHigh, &zHigh);

  const ImVec2 plotSize = ImGui::GetContentRegionAvail();
  if (ImPlot::BeginPlot("Trajectory", plotSize)) {
    ImPlot::SetupAxes("x (m)", "z (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, 0.0,
                            std::max(result.rawWindow.xBack * 1.15, 1.0),
                            ImGuiCond_Always);
    ImPlot::SetupAxisLimits(
        ImAxis_Y1, 0.0,
        std::max({config.zCeilingMax, result.nominalApex.zApex, config.zTarget}) * 1.05,
        ImGuiCond_Always);

    ImPlot::PlotLine("v_low", xLow.data(), zLow.data(), static_cast<int>(xLow.size()));
    ImPlot::PlotLine("v_nominal", xNominal.data(), zNominal.data(),
                     static_cast<int>(xNominal.size()));
    ImPlot::PlotLine("v_high", xHigh.data(), zHigh.data(), static_cast<int>(xHigh.size()));

    if (config.showTrajectoryPoints) {
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.5f);
      ImPlot::PlotScatter("nominal_pts", xNominal.data(), zNominal.data(),
                          static_cast<int>(xNominal.size()));
    }

    const double xWindowEff[2] = {result.effectiveWindow.xFrontEff, result.effectiveWindow.xBackEff};
    const double zWindowEff[2] = {config.zTarget, config.zTarget};
    ImPlot::SetNextLineStyle(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), 3.0f);
    ImPlot::PlotLine("window_eff", xWindowEff, zWindowEff, 2);

    if (config.showWindowComparison) {
      const double xWindowRaw[2] = {result.rawWindow.xFront, result.rawWindow.xBack};
      const double zWindowRaw[2] = {config.zTarget, config.zTarget};
      ImPlot::SetNextLineStyle(ImVec4(0.3f, 0.9f, 0.3f, 1.0f), 3.0f);
      ImPlot::PlotLine("window_raw", xWindowRaw, zWindowRaw, 2);
    }

    if (config.showEntryPoint) {
      const double x = result.nominalIntersection.xEntry;
      const double z = config.zTarget;
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f);
      ImPlot::PlotScatter("entry", &x, &z, 1);
    }

    if (config.showApexPoint) {
      const double x = result.nominalApex.xApex;
      const double z = result.nominalApex.zApex;
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 6.0f);
      ImPlot::PlotScatter("apex", &x, &z, 1);
    }

    ImPlot::EndPlot();
  }
}

void PlotThetaVHeatmap(const shootersim::SolveResult& result) {
  const int rows = static_cast<int>(result.thetaSamplesRad.size());
  const int cols = static_cast<int>(result.velocitySamples.size());
  if (rows <= 0 || cols <= 0 || static_cast<int>(result.validityGrid.size()) != rows * cols) {
    ImGui::TextWrapped("No theta-v validity data yet. Solve a distance first.");
    return;
  }

  std::vector<float> values;
  values.reserve(result.validityGrid.size());
  for (uint8_t v : result.validityGrid) {
    values.push_back(static_cast<float>(v));
  }

  const double xMin = result.velocitySamples.front();
  const double xMax = result.velocitySamples.back();
  const double yMin = util::RadToDeg(result.thetaSamplesRad.front());
  const double yMax = util::RadToDeg(result.thetaSamplesRad.back());

  if (ImPlot::BeginPlot("Theta-V Validity Heatmap", ImVec2(-1.0f, 240.0f))) {
    ImPlot::SetupAxes("v (m/s)", "theta (deg)");
    ImPlot::PlotHeatmap("validity", values.data(), rows, cols, 0.0, 1.0, "%.0f",
                        ImPlotPoint(xMin, yMin), ImPlotPoint(xMax, yMax));
    ImPlot::EndPlot();
  }
}

void PlotSweepCurves(const shootersim::SweepResult& result) {
  if (result.points.empty()) {
    ImGui::TextWrapped("Run a distance sweep to generate lookup curves.");
    return;
  }

  std::vector<double> d;
  std::vector<double> thetaDeg;
  std::vector<double> vNominal;
  std::vector<double> deltaV;
  d.reserve(result.points.size());
  thetaDeg.reserve(result.points.size());
  vNominal.reserve(result.points.size());
  deltaV.reserve(result.points.size());

  for (const auto& p : result.points) {
    d.push_back(p.distance);
    if (p.hasSolution) {
      thetaDeg.push_back(util::RadToDeg(p.bestThetaRad));
      vNominal.push_back(p.bestVNominal);
      deltaV.push_back(p.bestDeltaV);
    } else {
      thetaDeg.push_back(NAN);
      vNominal.push_back(NAN);
      deltaV.push_back(NAN);
    }
  }

  if (ImPlot::BeginPlot("distance -> bestTheta", ImVec2(-1.0f, 150.0f))) {
    ImPlot::SetupAxes("distance (m)", "theta (deg)");
    ImPlot::PlotLine("theta", d.data(), thetaDeg.data(), static_cast<int>(d.size()));
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("distance -> bestVNominal", ImVec2(-1.0f, 150.0f))) {
    ImPlot::SetupAxes("distance (m)", "v (m/s)");
    ImPlot::PlotLine("v_nominal", d.data(), vNominal.data(), static_cast<int>(d.size()));
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("distance -> bestDeltaV", ImVec2(-1.0f, 150.0f))) {
    ImPlot::SetupAxes("distance (m)", "delta v (m/s)");
    ImPlot::PlotLine("delta_v", d.data(), deltaV.data(), static_cast<int>(d.size()));
    ImPlot::EndPlot();
  }
}

}  // namespace shootersim::gui
