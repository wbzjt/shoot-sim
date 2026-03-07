#include "TheoryDataLoader.h"

#include <algorithm>

#include "CsvUtil.h"
#include "MathUtil.h"

namespace calibration {
namespace {

bool ParseTheoryRow(const std::vector<std::string>& row,
                    int colDistance,
                    int colPitch,
                    int colVel,
                    int colVLow,
                    int colVHigh,
                    int colDelta,
                    TheoryPoint* out) {
  if (out == nullptr) {
    return false;
  }
  if (colDistance < 0 || colPitch < 0 || colVel < 0) {
    return false;
  }

  double distance = 0.0;
  double pitch = 0.0;
  double vel = 0.0;
  if (!math::ParseDouble(row[static_cast<std::size_t>(colDistance)], &distance)) {
    return false;
  }
  if (!math::ParseDouble(row[static_cast<std::size_t>(colPitch)], &pitch)) {
    return false;
  }
  if (!math::ParseDouble(row[static_cast<std::size_t>(colVel)], &vel)) {
    return false;
  }

  out->distanceM = distance;
  out->pitchDeg = pitch;
  out->theoryVelMps = vel;

  if (colVLow >= 0) {
    out->vLowMps = math::ParseOptionalDouble(row[static_cast<std::size_t>(colVLow)]);
  }
  if (colVHigh >= 0) {
    out->vHighMps = math::ParseOptionalDouble(row[static_cast<std::size_t>(colVHigh)]);
  }
  if (colDelta >= 0) {
    out->deltaVMps = math::ParseOptionalDouble(row[static_cast<std::size_t>(colDelta)]);
  }

  return true;
}

}  // namespace

bool TheoryDataLoader::LoadFromCsv(const std::string& path,
                                   std::vector<TheoryPoint>* out,
                                   std::string* error) const {
  if (out == nullptr) {
    if (error != nullptr) {
      *error = "Theory output vector is null";
    }
    return false;
  }

  csv::Table table;
  if (!csv::ReadCsv(path, &table, error)) {
    return false;
  }

  const auto index = csv::BuildHeaderIndex(table.header);

  const int colDistance = csv::FindHeaderColumn(index, {
      "distance_m", "distance", "dist", "d",
  });
  const int colPitch = csv::FindHeaderColumn(index, {
      "pitch_deg", "pitch", "pitchdeg", "theta_deg",
  });
  const int colVel = csv::FindHeaderColumn(index, {
      "theory_vel_mps", "theoreticalexitvelocity", "theoryvel", "velocity", "vel", "v_nominal_mps",
  });

  const int colVLow = csv::FindHeaderColumn(index, {
      "v_low_mps", "vlow", "v_low",
  });
  const int colVHigh = csv::FindHeaderColumn(index, {
      "v_high_mps", "vhigh", "v_high",
  });
  const int colDelta = csv::FindHeaderColumn(index, {
      "delta_v_mps", "deltav", "delta_v",
  });

  if (colDistance < 0 || colPitch < 0 || colVel < 0) {
    if (error != nullptr) {
      *error = "Theory CSV missing required columns. Need distance_m, pitch_deg, theory_vel_mps (or aliases).";
    }
    return false;
  }

  out->clear();
  out->reserve(table.rows.size());
  for (const auto& row : table.rows) {
    if (row.size() < table.header.size()) {
      continue;
    }
    TheoryPoint p;
    if (ParseTheoryRow(row, colDistance, colPitch, colVel,
                       colVLow, colVHigh, colDelta, &p)) {
      out->push_back(p);
    }
  }

  if (out->empty()) {
    if (error != nullptr) {
      *error = "Theory CSV contains no valid data rows";
    }
    return false;
  }

  std::sort(out->begin(), out->end(), [](const TheoryPoint& a, const TheoryPoint& b) {
    return a.distanceM < b.distanceM;
  });

  return true;
}

}  // namespace calibration
