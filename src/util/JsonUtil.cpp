#include "util/JsonUtil.h"

#include <fstream>

#include <nlohmann/json.hpp>

namespace shootersim::util {
namespace {

using nlohmann::json;

json ConfigToJson(const SimulationConfig& c) {
  return json{
      {"physicsMode", static_cast<int>(c.physicsMode)},
      {"gravity", c.gravity},
      {"releaseHeight", c.releaseHeight},
      {"zTarget", c.zTarget},
      {"dragCoefficient", c.dragCoefficient},
      {"dragDt", c.dragDt},
      {"dragMaxTime", c.dragMaxTime},
      {"windowInputMode", static_cast<int>(c.windowInputMode)},
      {"xFrontOffset", c.xFrontOffset},
      {"xBackOffset", c.xBackOffset},
      {"openingDepth", c.openingDepth},
      {"ballRadius", c.ballRadius},
      {"frontMargin", c.frontMargin},
      {"backMargin", c.backMargin},
      {"thetaMinRad", c.thetaMinRad},
      {"thetaMaxRad", c.thetaMaxRad},
      {"vMin", c.vMin},
      {"vMax", c.vMax},
      {"zCeilingMax", c.zCeilingMax},
      {"thetaStepRad", c.thetaStepRad},
      {"vStep", c.vStep},
      {"enableMonotonicBoundarySearch", c.enableMonotonicBoundarySearch},
      {"enableEntryAngleConstraint", c.enableEntryAngleConstraint},
      {"minAbsEntryAngleRad", c.minAbsEntryAngleRad},
      {"enableFlightTimeConstraint", c.enableFlightTimeConstraint},
      {"maxFlightTime", c.maxFlightTime},
      {"nominalMode", static_cast<int>(c.nominalMode)},
      {"targetBias", c.targetBias},
      {"speedBias", c.speedBias},
      {"trajectorySamples", c.trajectorySamples},
      {"showTrajectoryPoints", c.showTrajectoryPoints},
      {"showEntryPoint", c.showEntryPoint},
      {"showApexPoint", c.showApexPoint},
      {"showWindowComparison", c.showWindowComparison},
  };
}

bool ParseConfigJson(const json& j, SimulationConfig* c) {
  if (c == nullptr) {
    return false;
  }

  c->physicsMode = static_cast<PhysicsMode>(j.value("physicsMode", static_cast<int>(c->physicsMode)));
  c->gravity = j.value("gravity", c->gravity);
  c->releaseHeight = j.value("releaseHeight", c->releaseHeight);
  c->zTarget = j.value("zTarget", c->zTarget);
  c->dragCoefficient = j.value("dragCoefficient", c->dragCoefficient);
  c->dragDt = j.value("dragDt", c->dragDt);
  c->dragMaxTime = j.value("dragMaxTime", c->dragMaxTime);
  c->windowInputMode =
      static_cast<WindowInputMode>(j.value("windowInputMode", static_cast<int>(c->windowInputMode)));
  c->xFrontOffset = j.value("xFrontOffset", c->xFrontOffset);
  c->xBackOffset = j.value("xBackOffset", c->xBackOffset);
  c->openingDepth = j.value("openingDepth", c->openingDepth);
  c->ballRadius = j.value("ballRadius", c->ballRadius);
  c->frontMargin = j.value("frontMargin", c->frontMargin);
  c->backMargin = j.value("backMargin", c->backMargin);
  c->thetaMinRad = j.value("thetaMinRad", c->thetaMinRad);
  c->thetaMaxRad = j.value("thetaMaxRad", c->thetaMaxRad);
  c->vMin = j.value("vMin", c->vMin);
  c->vMax = j.value("vMax", c->vMax);
  c->zCeilingMax = j.value("zCeilingMax", c->zCeilingMax);
  c->thetaStepRad = j.value("thetaStepRad", c->thetaStepRad);
  c->vStep = j.value("vStep", c->vStep);
  c->enableMonotonicBoundarySearch =
      j.value("enableMonotonicBoundarySearch", c->enableMonotonicBoundarySearch);
  c->enableEntryAngleConstraint =
      j.value("enableEntryAngleConstraint", c->enableEntryAngleConstraint);
  c->minAbsEntryAngleRad = j.value("minAbsEntryAngleRad", c->minAbsEntryAngleRad);
  c->enableFlightTimeConstraint =
      j.value("enableFlightTimeConstraint", c->enableFlightTimeConstraint);
  c->maxFlightTime = j.value("maxFlightTime", c->maxFlightTime);
  c->nominalMode = static_cast<NominalSelectionMode>(
      j.value("nominalMode", static_cast<int>(c->nominalMode)));
  c->targetBias = j.value("targetBias", c->targetBias);
  c->speedBias = j.value("speedBias", c->speedBias);
  c->trajectorySamples = j.value("trajectorySamples", c->trajectorySamples);
  c->showTrajectoryPoints = j.value("showTrajectoryPoints", c->showTrajectoryPoints);
  c->showEntryPoint = j.value("showEntryPoint", c->showEntryPoint);
  c->showApexPoint = j.value("showApexPoint", c->showApexPoint);
  c->showWindowComparison = j.value("showWindowComparison", c->showWindowComparison);
  return true;
}

}  // namespace

bool SaveConfigJson(const std::string& path, const SimulationConfig& config, std::string* error) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open config path: " + path;
    }
    return false;
  }

  try {
    file << ConfigToJson(config).dump(2);
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }

  return true;
}

bool LoadConfigJson(const std::string& path, SimulationConfig* config, std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "Config output pointer is null";
    }
    return false;
  }

  std::ifstream file(path, std::ios::in);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open config path: " + path;
    }
    return false;
  }

  try {
    json j;
    file >> j;
    return ParseConfigJson(j, config);
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }
}

bool SaveSweepJson(const std::string& path,
                   const SimulationConfig& config,
                   const SweepResult& sweep,
                   std::string* error) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open sweep path: " + path;
    }
    return false;
  }

  try {
    json j;
    j["config"] = ConfigToJson(config);
    j["request"] = {
        {"distanceMin", sweep.request.distanceMin},
        {"distanceMax", sweep.request.distanceMax},
        {"distanceStep", sweep.request.distanceStep},
    };
    j["fromCache"] = sweep.fromCache;
    j["totalTimeMs"] = sweep.totalTimeMs;

    json points = json::array();
    for (const auto& p : sweep.points) {
      points.push_back({
          {"distance", p.distance},
          {"hasSolution", p.hasSolution},
          {"bestThetaRad", p.bestThetaRad},
          {"bestVNominal", p.bestVNominal},
          {"bestVLow", p.bestVLow},
          {"bestVHigh", p.bestVHigh},
          {"bestDeltaV", p.bestDeltaV},
          {"timeOfFlight", p.timeOfFlight},
          {"entryAngleRad", p.entryAngleRad},
          {"zApex", p.zApex},
          {"failureReason", p.failureReason},
      });
    }
    j["points"] = points;

    file << j.dump(2);
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }

  return true;
}

}  // namespace shootersim::util
