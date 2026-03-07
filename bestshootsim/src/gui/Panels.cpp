#include "gui/Panels.h"

#include <imgui.h>

#include "util/MathUtil.h"

namespace shootersim::gui {

bool DragAngleDeg(const char* label,
                  double* angleRad,
                  double speedDeg,
                  double minDeg,
                  double maxDeg,
                  const char* format) {
  float valueDeg = static_cast<float>(util::RadToDeg(*angleRad));
  const bool changed = ImGui::DragFloat(label, &valueDeg, static_cast<float>(speedDeg),
                                        static_cast<float>(minDeg), static_cast<float>(maxDeg), format);
  if (changed) {
    *angleRad = util::DegToRad(static_cast<double>(valueDeg));
  }
  return changed;
}

void DrawConstraintStatus(const shootersim::ConstraintResult& result) {
  ImGui::Text("Constraints");
  ImGui::Separator();
  ImGui::BulletText("Pitch range: %s", result.pitchInRange ? "PASS" : "FAIL");
  ImGui::BulletText("Speed range: %s", result.speedInRange ? "PASS" : "FAIL");
  ImGui::BulletText("Effective window valid: %s", result.effectiveWindowValid ? "PASS" : "FAIL");
  ImGui::BulletText("Ceiling: %s", result.ceilingOk ? "PASS" : "FAIL");
  ImGui::BulletText("Descending intersection: %s",
                    result.hasDescendingIntersection ? "PASS" : "FAIL");
  ImGui::BulletText("In effective window: %s", result.inEffectiveWindow ? "PASS" : "FAIL");
  ImGui::BulletText("Entry angle: %s", result.entryAngleOk ? "PASS" : "FAIL");
  ImGui::BulletText("Flight time: %s", result.flightTimeOk ? "PASS" : "FAIL");

  if (!result.failureReasons.empty()) {
    ImGui::Separator();
    ImGui::Text("Failure reasons:");
    for (const auto& reason : result.failureReasons) {
      ImGui::BulletText("%s", reason.c_str());
    }
  }
}

void DrawSolveSummary(const shootersim::SolveResult& result) {
  if (!result.hasSolution) {
    ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), "No solution");
    ImGui::TextWrapped("%s", result.failureReason.c_str());
    return;
  }

  ImGui::Text("Best theta: %.3f deg", util::RadToDeg(result.bestThetaRad));
  ImGui::Text("Best v nominal: %.4f m/s", result.bestVNominal);
  ImGui::Text("Interval: [%.4f, %.4f] m/s", result.bestVLow, result.bestVHigh);
  ImGui::Text("Delta v: %.4f m/s", result.bestDeltaV);
  ImGui::Text("Entry x: %.4f m", result.nominalIntersection.xEntry);
  ImGui::Text("Entry speed: %.4f m/s", result.nominalIntersection.entrySpeed);
  ImGui::Text("Entry angle: %.3f deg", util::RadToDeg(result.nominalIntersection.entryAngleRad));
  ImGui::Text("Apex: z=%.4f m at x=%.4f m", result.nominalApex.zApex, result.nominalApex.xApex);
  ImGui::Text("Solve time: %.3f ms", result.solveTimeMs);
}

}  // namespace shootersim::gui
