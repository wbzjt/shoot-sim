#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace shootersim {

constexpr double kDefaultGravity = 9.80665;

enum class WindowInputMode {
  kDirectFrontBackOffset = 0,
  kCenterAndDepth = 1,
};

enum class NominalSelectionMode {
  kWindowBias = 0,
  kSpeedBias = 1,
};

enum class PhysicsMode {
  kAnalyticNoDrag = 0,
  kNumericQuadraticDrag = 1,
};

struct TargetWindow {
  double xFront = 0.0;
  double xBack = 0.0;
};

struct EffectiveWindow {
  double xFrontEff = 0.0;
  double xBackEff = 0.0;
  bool valid = false;
  std::string error;
};

struct TrajectorySample {
  double t = 0.0;
  double x = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vz = 0.0;
};

struct IntersectionResult {
  bool hasIntersection = false;
  bool descending = false;
  double tEntry = 0.0;
  double xEntry = 0.0;
  double zEntry = 0.0;
  double vxEntry = 0.0;
  double vzEntry = 0.0;
  double entrySpeed = 0.0;
  double entryAngleRad = 0.0;
};

struct ApexResult {
  bool valid = false;
  double tApex = 0.0;
  double xApex = 0.0;
  double zApex = 0.0;
};

struct ConstraintResult {
  bool overallValid = false;
  bool pitchInRange = false;
  bool speedInRange = false;
  bool ceilingOk = false;
  bool hasDescendingIntersection = false;
  bool inEffectiveWindow = false;
  bool effectiveWindowValid = false;
  bool entryAngleOk = true;
  bool flightTimeOk = true;
  std::vector<std::string> failureReasons;
};

struct ConstraintEvaluation {
  ConstraintResult result;
  TargetWindow rawWindow;
  EffectiveWindow effectiveWindow;
  IntersectionResult intersection;
  ApexResult apex;
};

struct ValidInterval {
  double vLow = 0.0;
  double vHigh = 0.0;
  double deltaV = 0.0;
  bool refined = false;
};

struct ThetaScanResult {
  double thetaRad = 0.0;
  bool hasValidInterval = false;
  ValidInterval bestInterval;
  double vNominal = 0.0;
};

struct SolveResult {
  bool hasSolution = false;
  double distance = 0.0;

  double bestThetaRad = 0.0;
  double bestVNominal = 0.0;
  double bestVLow = 0.0;
  double bestVHigh = 0.0;
  double bestDeltaV = 0.0;

  TargetWindow rawWindow;
  EffectiveWindow effectiveWindow;

  IntersectionResult nominalIntersection;
  IntersectionResult lowIntersection;
  IntersectionResult highIntersection;
  ApexResult nominalApex;

  ConstraintResult nominalConstraint;

  std::vector<double> thetaSamplesRad;
  std::vector<double> velocitySamples;
  std::vector<uint8_t> validityGrid;  // Row-major: theta-major.
  std::vector<ThetaScanResult> thetaScan;

  std::string failureReason;
  double solveTimeMs = 0.0;
};

struct SweepRequest {
  double distanceMin = 1.0;
  double distanceMax = 6.1;
  double distanceStep = 0.01;
};

struct SweepPoint {
  double distance = 0.0;
  bool hasSolution = false;

  double bestThetaRad = 0.0;
  double bestVNominal = 0.0;
  double bestVLow = 0.0;
  double bestVHigh = 0.0;
  double bestDeltaV = 0.0;

  double timeOfFlight = 0.0;
  double entryAngleRad = 0.0;
  double zApex = 0.0;

  std::string failureReason;
};

struct SweepResult {
  SweepRequest request;
  std::vector<SweepPoint> points;
  bool fromCache = false;
  double totalTimeMs = 0.0;
};

struct SimulationConfig {
  // Physics
  PhysicsMode physicsMode = PhysicsMode::kAnalyticNoDrag;
  double gravity = kDefaultGravity;
  // dz = zTarget - releaseHeight + kTargetOffsetZ = 1.82 - 0.66 + 0 = 1.16 m
  double releaseHeight = 0.66;
  double zTarget = 1.82;

  // Optional quadratic drag acceleration: a_drag = -k * |v| * v
  double dragCoefficient = 0.0;
  double dragDt = 0.001;
  double dragMaxTime = 5.0;

  // Target window input
  WindowInputMode windowInputMode = WindowInputMode::kCenterAndDepth;

  // Mode A: direct offsets from reference plane at distance d
  double xFrontOffset = -0.15;
  double xBackOffset = 0.15;

  // Mode B: centered opening around distance d
  // Hub top hex opening diameter: 41.7 in ~= 1.06 m
  double openingDepth = 1.06;

  // Effective window shrink
  // Fuel diameter: 0.15 m
  double ballRadius = 0.075;
  double frontMargin = 0.0;
  double backMargin = 0.0;

  // Search space (SI internal angles in rad)
  // Pitch range: 44.57 deg ~ 70.53 deg
  double thetaMinRad = 0.777893247613873;
  double thetaMaxRad = 1.2309807214316;
  // Flywheel speed range: 0 ~ 20 m/s
  double vMin = 0.0;
  double vMax = 20.0;
  double zCeilingMax = 3.3;

  double thetaStepRad = 0.00017453292519943296;  // 0.01 deg
  double vStep = 0.05;

  bool enableMonotonicBoundarySearch = false;

  // Optional constraints
  bool enableEntryAngleConstraint = false;
  double minAbsEntryAngleRad = 0.20;

  bool enableFlightTimeConstraint = false;
  double maxFlightTime = 1.30;

  // Nominal selection policy
  NominalSelectionMode nominalMode = NominalSelectionMode::kWindowBias;
  double targetBias = 0.38;
  double speedBias = 0.40;

  // Rendering / diagnostics
  int trajectorySamples = 180;
  bool showTrajectoryPoints = false;
  bool showEntryPoint = true;
  bool showApexPoint = true;
  bool showWindowComparison = true;
};

}  // namespace shootersim
