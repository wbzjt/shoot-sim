#include "CalibrationDataLoader.h"

#include <algorithm>
#include <cmath>

#include "CsvUtil.h"
#include "MathUtil.h"

namespace calibration {

bool CalibrationDataLoader::LoadFromCsv(const std::string& path,
                                        std::vector<CalibrationSample>* out,
                                        std::string* error) const {
  if (out == nullptr) {
    if (error != nullptr) {
      *error = "Calibration output vector is null";
    }
    return false;
  }

  csv::Table table;
  if (!csv::ReadCsv(path, &table, error)) {
    return false;
  }

  const auto index = csv::BuildHeaderIndex(table.header);
  const int cDist = csv::FindHeaderColumn(index, {"distance_m", "distance", "dist"});
  const int cPitch = csv::FindHeaderColumn(index, {"pitch_deg", "pitch", "theta_deg"});
  const int cTheoryVel = csv::FindHeaderColumn(index, {"theory_vel_mps", "theoryvel", "theoreticalexitvelocity", "velocity", "vel"});
  const int cTargetRpm = csv::FindHeaderColumn(index, {"motor_target_rpm", "target_rpm", "commanded_rpm", "rpm_target"});
  const int cMeasuredRpm = csv::FindHeaderColumn(index, {"measured_motor_rpm", "measured_rpm", "rpm_measured"});
  const int cShot = csv::FindHeaderColumn(index, {"shot_index", "shot"});
  const int cLane = csv::FindHeaderColumn(index, {"lane_index", "lane"});
  const int cHit = csv::FindHeaderColumn(index, {"hit", "hitormiss", "result"});
  const int cScore = csv::FindHeaderColumn(index, {"score", "hitqualityscore", "quality"});
  const int cRecovery = csv::FindHeaderColumn(index, {"recovery_ms", "recoverytimems"});
  const int cBattery = csv::FindHeaderColumn(index, {"battery_v", "batteryvoltage"});
  const int cSteadyErr = csv::FindHeaderColumn(index, {"steadystateerrorrpm", "steady_state_error_rpm"});
  const int cNotes = csv::FindHeaderColumn(index, {"notes", "note"});

  if (cDist < 0 || cPitch < 0 || cTheoryVel < 0 || cTargetRpm < 0) {
    if (error != nullptr) {
      *error = "Calibration CSV missing required columns. Need distance_m,pitch_deg,theory_vel_mps,motor_target_rpm.";
    }
    return false;
  }

  out->clear();
  out->reserve(table.rows.size());

  for (const auto& row : table.rows) {
    if (row.size() < table.header.size()) {
      continue;
    }

    CalibrationSample s;
    if (!math::ParseDouble(row[static_cast<std::size_t>(cDist)], &s.distanceM)) {
      continue;
    }
    if (!math::ParseDouble(row[static_cast<std::size_t>(cPitch)], &s.pitchDeg)) {
      continue;
    }
    if (!math::ParseDouble(row[static_cast<std::size_t>(cTheoryVel)], &s.theoryVelMps)) {
      continue;
    }
    if (!math::ParseDouble(row[static_cast<std::size_t>(cTargetRpm)], &s.motorTargetRpm)) {
      continue;
    }

    if (cMeasuredRpm >= 0 &&
        !math::ParseDouble(row[static_cast<std::size_t>(cMeasuredRpm)], &s.measuredMotorRpm)) {
      s.measuredMotorRpm = s.motorTargetRpm;
    } else if (cMeasuredRpm < 0) {
      s.measuredMotorRpm = s.motorTargetRpm;
    }

    if (cShot >= 0) {
      double shot = 0.0;
      if (math::ParseDouble(row[static_cast<std::size_t>(cShot)], &shot)) {
        s.shotIndex = static_cast<int>(std::lround(shot));
      }
    }
    if (cLane >= 0) {
      double lane = 0.0;
      if (math::ParseDouble(row[static_cast<std::size_t>(cLane)], &lane)) {
        s.laneIndex = static_cast<int>(std::lround(lane));
      }
    }

    if (cHit >= 0) {
      bool hit = false;
      if (math::ParseBoolLoose(row[static_cast<std::size_t>(cHit)], &hit)) {
        s.hit = hit;
      }
    }

    if (cScore >= 0) {
      s.score = math::ParseOptionalDouble(row[static_cast<std::size_t>(cScore)]);
    }
    if (cRecovery >= 0) {
      s.recoveryTimeMs = math::ParseOptionalDouble(row[static_cast<std::size_t>(cRecovery)]);
    }
    if (cBattery >= 0) {
      s.batteryV = math::ParseOptionalDouble(row[static_cast<std::size_t>(cBattery)]);
    }
    if (cSteadyErr >= 0) {
      s.steadyStateErrorRpm =
          math::ParseOptionalDouble(row[static_cast<std::size_t>(cSteadyErr)]);
    }
    if (cNotes >= 0) {
      s.notes = row[static_cast<std::size_t>(cNotes)];
    }

    out->push_back(s);
  }

  if (out->empty()) {
    if (error != nullptr) {
      *error = "Calibration CSV contains no valid rows";
    }
    return false;
  }

  std::sort(out->begin(), out->end(), [](const CalibrationSample& a, const CalibrationSample& b) {
    if (a.theoryVelMps != b.theoryVelMps) {
      return a.theoryVelMps < b.theoryVelMps;
    }
    return a.motorTargetRpm < b.motorTargetRpm;
  });

  return true;
}

}  // namespace calibration
