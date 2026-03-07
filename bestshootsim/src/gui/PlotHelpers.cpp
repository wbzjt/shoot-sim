#include "gui/PlotHelpers.h"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "model/Ballistics.h"
#include "util/MathUtil.h"

namespace shootersim::gui {
namespace {

void BuildXZVectors(const std::vector<shootersim::TrajectorySample>& trajectory,
                    std::vector<double>* x, std::vector<double>* z) {
  x->clear();
  z->clear();
  x->reserve(trajectory.size());
  z->reserve(trajectory.size());
  for (const auto& s : trajectory) {
    x->push_back(s.x);
    z->push_back(s.z);
  }
}

ImPlotColormap GetValidityColormap() {
  static ImPlotColormap cmap = -1;
  if (cmap == -1) {
    static const ImVec4 colors[2] = {
        ImVec4(0.12f, 0.12f, 0.14f, 1.0f),
        ImVec4(0.25f, 0.95f, 0.45f, 1.0f),
    };
    cmap = ImPlot::AddColormap("ShooterValidity", colors, 2, true);
  }
  return cmap;
}

}  // namespace

void PlotTrajectory(const shootersim::SimulationConfig& config,
                    const shootersim::SolveResult& result) {
  if (!result.hasSolution) {
    ImGui::TextWrapped("No feasible trajectory under current constraints.");
    return;
  }

  const auto trajNominal =
      model::BuildTrajectory(config, result.bestThetaRad, result.bestVNominal,
                             config.trajectorySamples);
  const auto trajLow = model::BuildTrajectory(
      config, result.bestThetaRad, result.bestVLow, config.trajectorySamples);
  const auto trajHigh = model::BuildTrajectory(
      config, result.bestThetaRad, result.bestVHigh, config.trajectorySamples);

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
        std::max(
            {config.zCeilingMax, result.nominalApex.zApex, config.zTarget}) *
            1.05,
        ImGuiCond_Always);

    ImPlot::PlotLine("v_low", xLow.data(), zLow.data(),
                     static_cast<int>(xLow.size()));
    ImPlot::PlotLine("v_nominal", xNominal.data(), zNominal.data(),
                     static_cast<int>(xNominal.size()));
    ImPlot::PlotLine("v_high", xHigh.data(), zHigh.data(),
                     static_cast<int>(xHigh.size()));

    if (config.showTrajectoryPoints) {
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.5f);
      ImPlot::PlotScatter("nominal_pts", xNominal.data(), zNominal.data(),
                          static_cast<int>(xNominal.size()));
    }

    const double xWindowEff[2] = {result.effectiveWindow.xFrontEff,
                                  result.effectiveWindow.xBackEff};
    const double zWindowEff[2] = {config.zTarget, config.zTarget};
    ImPlot::SetNextLineStyle(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), 3.0f);
    ImPlot::PlotLine("window_eff", xWindowEff, zWindowEff, 2);

    if (config.showWindowComparison) {
      const double xWindowRaw[2] = {result.rawWindow.xFront,
                                    result.rawWindow.xBack};
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
  if (rows <= 0 || cols <= 0 ||
      static_cast<int>(result.validityGrid.size()) != rows * cols) {
    ImGui::TextWrapped("No theta-v validity data yet. Solve a distance first.");
    return;
  }

  // ImPlot::PlotHeatmap convention: row 0 is at yMax (top of plot), last row
  // is at yMin (bottom). Our validityGrid stores row 0 = thetaMin (smallest
  // angle). We must reverse the row order so that thetaMin lands at the bottom
  // and thetaMax at the top, which matches the y-axis labelling.
  const double vStep =
      (cols >= 2)
          ? std::fabs(result.velocitySamples[1] - result.velocitySamples[0])
          : 0.0;
  const double thetaStepDeg =
      (rows >= 2) ? std::fabs(util::RadToDeg(result.thetaSamplesRad[1]) -
                              util::RadToDeg(result.thetaSamplesRad[0]))
                  : 0.0;
  const double xMin = result.velocitySamples.front();
  const double xMax = result.velocitySamples.back();
  const double yMin = util::RadToDeg(result.thetaSamplesRad.front());
  const double yMax = util::RadToDeg(result.thetaSamplesRad.back());
  const double xPad = 0.5 * vStep;
  const double yPad = 0.5 * thetaStepDeg;

  // Build row-flipped float buffer: plotRow 0 = original last row (thetaMax).
  constexpr int kMaxHeatmapCells = 8'000'000;
  const long long totalCells =
      static_cast<long long>(rows) * static_cast<long long>(cols);
  const bool previewMode = totalCells > kMaxHeatmapCells;
  int displayRows = rows;
  if (previewMode) {
    displayRows = std::min(rows, 800);
    ImGui::TextWrapped(
        "Heatmap preview mode: %d x %d (%.2fM cells). Rows downsampled to %d.",
        rows, cols, static_cast<double>(totalCells) / 1e6, displayRows);
  }

  // Flip rows: srcRow i (thetaMin + i*step) -> plotRow (displayRows-1-i)
  // (top=thetaMax).
  std::vector<float> displayValues(static_cast<std::size_t>(displayRows * cols),
                                   0.0f);
  for (int dr = 0; dr < displayRows; ++dr) {
    // Map display row -> source row (nearest neighbour when downsampling)
    int sr = dr;
    if (previewMode) {
      const double center =
          (static_cast<double>(dr) + 0.5) / static_cast<double>(displayRows);
      sr = std::clamp(static_cast<int>(std::floor(center * rows)), 0, rows - 1);
    }
    // Flip: plot row 0 = thetaMax = source row (rows-1)
    const int srcRow = (rows - 1) - sr;
    for (int c = 0; c < cols; ++c) {
      displayValues[static_cast<std::size_t>(dr * cols + c)] =
          static_cast<float>(
              result.validityGrid[static_cast<std::size_t>(srcRow * cols + c)]);
    }
  }

  // After flipping, plotRow 0 maps to thetaMax and plotRow (displayRows-1)
  // maps to thetaMin, so bounds_min.y = yMin and bounds_max.y = yMax is
  // correct.
  if (ImPlot::BeginPlot("Theta-V Validity Heatmap", ImVec2(-1.0f, 280.0f))) {
    ImPlot::SetupAxes("v (m/s)", "theta (deg)");
    ImPlot::SetupAxisLimits(ImAxis_X1, xMin - xPad, xMax + xPad,
                            ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, yMin - yPad, yMax + yPad,
                            ImGuiCond_Always);
    ImPlot::PushColormap(GetValidityColormap());
    ImPlot::PlotHeatmap("validity", displayValues.data(), displayRows, cols,
                        0.0, 1.0, nullptr,
                        ImPlotPoint(xMin - xPad, yMin - yPad),
                        ImPlotPoint(xMax + xPad, yMax + yPad));
    ImPlot::PopColormap();

    // Overlay: mark best solution with a crosshair annotation
    if (result.hasSolution) {
      const double bv = result.bestVNominal;
      const double bt = util::RadToDeg(result.bestThetaRad);
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, 10.0f,
                                 ImVec4(1.0f, 0.9f, 0.1f, 1.0f), 2.5f,
                                 ImVec4(1.0f, 0.9f, 0.1f, 1.0f));
      ImPlot::PlotScatter("best", &bv, &bt, 1);
    }

    ImPlot::EndPlot();
  }

  ImGui::TextColored(ImVec4(0.25f, 0.95f, 0.45f, 1.0f), "Green = valid");
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.50f, 0.50f, 0.55f, 1.0f), "Dark = invalid");
  if (result.hasSolution) {
    ImGui::Text("Best row: theta %.3f deg, interval [%.4f, %.4f], deltaV %.4f",
                util::RadToDeg(result.bestThetaRad), result.bestVLow,
                result.bestVHigh, result.bestDeltaV);
  }

  std::vector<double> thetaExact;
  std::vector<double> vLowExact;
  std::vector<double> vHighExact;
  thetaExact.reserve(result.thetaScan.size());
  vLowExact.reserve(result.thetaScan.size());
  vHighExact.reserve(result.thetaScan.size());
  for (const auto& scan : result.thetaScan) {
    if (!scan.hasValidInterval) {
      continue;
    }
    thetaExact.push_back(util::RadToDeg(scan.thetaRad));
    vLowExact.push_back(scan.bestInterval.vLow);
    vHighExact.push_back(scan.bestInterval.vHigh);
  }
  if (!thetaExact.empty() &&
      ImPlot::BeginPlot("Theta-v Interval (Exact)", ImVec2(-1.0f, 140.0f))) {
    ImPlot::SetupAxes("theta (deg)", "v (m/s)");
    ImPlot::PlotLine("v_low(theta)", thetaExact.data(), vLowExact.data(),
                     static_cast<int>(thetaExact.size()));
    ImPlot::PlotLine("v_high(theta)", thetaExact.data(), vHighExact.data(),
                     static_cast<int>(thetaExact.size()));
    if (result.hasSolution) {
      const double x1 = util::RadToDeg(result.bestThetaRad);
      const double y1 = result.bestVLow;
      const double y2 = result.bestVHigh;
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.5f,
                                 ImVec4(1.0f, 0.95f, 0.2f, 1.0f));
      ImPlot::PlotScatter("best v_low", &x1, &y1, 1);
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.5f,
                                 ImVec4(1.0f, 0.95f, 0.2f, 1.0f));
      ImPlot::PlotScatter("best v_high", &x1, &y2, 1);
    }
    ImPlot::EndPlot();
  }

  std::vector<double> thetaDeg;
  std::vector<double> deltaV;
  thetaDeg.reserve(result.thetaScan.size());
  deltaV.reserve(result.thetaScan.size());
  for (const auto& scan : result.thetaScan) {
    thetaDeg.push_back(util::RadToDeg(scan.thetaRad));
    deltaV.push_back(scan.hasValidInterval ? scan.bestInterval.deltaV : NAN);
  }

  if (ImPlot::BeginPlot("Theta vs Best DeltaV", ImVec2(-1.0f, 150.0f))) {
    ImPlot::SetupAxes("theta (deg)", "best deltaV (m/s)");
    ImPlot::PlotLine("deltaV(theta)", thetaDeg.data(), deltaV.data(),
                     static_cast<int>(thetaDeg.size()));
    if (result.hasSolution) {
      const double x = util::RadToDeg(result.bestThetaRad);
      const double y = result.bestDeltaV;
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 5.0f,
                                 ImVec4(1.0f, 0.95f, 0.2f, 1.0f));
      ImPlot::PlotScatter("best theta", &x, &y, 1);
    }
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
    ImPlot::PlotLine("theta", d.data(), thetaDeg.data(),
                     static_cast<int>(d.size()));
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("distance -> bestVNominal", ImVec2(-1.0f, 150.0f))) {
    ImPlot::SetupAxes("distance (m)", "v (m/s)");
    ImPlot::PlotLine("v_nominal", d.data(), vNominal.data(),
                     static_cast<int>(d.size()));
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("distance -> bestDeltaV", ImVec2(-1.0f, 150.0f))) {
    ImPlot::SetupAxes("distance (m)", "delta v (m/s)");
    ImPlot::PlotLine("delta_v", d.data(), deltaV.data(),
                     static_cast<int>(d.size()));
    ImPlot::EndPlot();
  }
}

}  // namespace shootersim::gui
