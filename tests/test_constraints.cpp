#include "TestFramework.h"

#include <cmath>

#include "model/Ballistics.h"
#include "model/Constraints.h"
#include "util/MathUtil.h"

namespace {

shootersim::SimulationConfig MakeBaseConfig() {
  shootersim::SimulationConfig cfg;
  cfg.physicsMode = shootersim::PhysicsMode::kAnalyticNoDrag;
  cfg.releaseHeight = 0.7;
  cfg.zTarget = 2.0;

  cfg.windowInputMode = shootersim::WindowInputMode::kCenterAndDepth;
  cfg.openingDepth = 5.0;
  cfg.ballRadius = 0.05;
  cfg.frontMargin = 0.0;
  cfg.backMargin = 0.0;

  cfg.thetaMinRad = shootersim::util::DegToRad(20.0);
  cfg.thetaMaxRad = shootersim::util::DegToRad(80.0);
  cfg.vMin = 4.0;
  cfg.vMax = 20.0;
  cfg.zCeilingMax = 10.0;

  return cfg;
}

}  // namespace

TEST_CASE("Constraints valid shot passes") {
  const auto cfg = MakeBaseConfig();
  const double distance = 3.5;
  const double theta = shootersim::util::DegToRad(55.0);

  bool foundValid = false;
  for (double v = 6.0; v <= 16.0; v += 0.05) {
    const auto eval = shootersim::model::EvaluateConstraints(cfg, distance, theta, v);
    if (eval.result.overallValid) {
      foundValid = true;
      break;
    }
  }
  EXPECT_TRUE(foundValid);
}

TEST_CASE("Constraints catch pitch/speed/ceiling violations") {
  auto cfg = MakeBaseConfig();
  const double distance = 3.5;

  auto evalPitch =
      shootersim::model::EvaluateConstraints(cfg, distance, shootersim::util::DegToRad(10.0), 9.0);
  EXPECT_FALSE(evalPitch.result.pitchInRange);
  EXPECT_FALSE(evalPitch.result.overallValid);

  auto evalSpeed =
      shootersim::model::EvaluateConstraints(cfg, distance, shootersim::util::DegToRad(55.0), 3.0);
  EXPECT_FALSE(evalSpeed.result.speedInRange);

  cfg.zCeilingMax = 1.0;
  auto evalCeiling =
      shootersim::model::EvaluateConstraints(cfg, distance, shootersim::util::DegToRad(55.0), 9.0);
  EXPECT_FALSE(evalCeiling.result.ceilingOk);
}

TEST_CASE("Constraints catch missing intersection and window misses") {
  auto cfg = MakeBaseConfig();
  const double distance = 3.5;

  cfg.zTarget = 8.0;
  auto evalNoIntersect =
      shootersim::model::EvaluateConstraints(cfg, distance, shootersim::util::DegToRad(55.0), 9.0);
  EXPECT_FALSE(evalNoIntersect.result.hasDescendingIntersection);

  cfg = MakeBaseConfig();
  cfg.openingDepth = 0.10;
  auto evalWindow =
      shootersim::model::EvaluateConstraints(cfg, distance, shootersim::util::DegToRad(55.0), 9.0);
  EXPECT_FALSE(evalWindow.result.inEffectiveWindow);
}

TEST_CASE("Constraints invalid effective window reports failure") {
  auto cfg = MakeBaseConfig();
  cfg.openingDepth = 0.15;
  cfg.ballRadius = 0.08;
  cfg.frontMargin = 0.03;
  cfg.backMargin = 0.03;

  const auto eval = shootersim::model::EvaluateConstraints(
      cfg, 3.0, shootersim::util::DegToRad(55.0), 9.0);

  EXPECT_FALSE(eval.result.effectiveWindowValid);
  EXPECT_FALSE(eval.result.overallValid);
  EXPECT_TRUE(!eval.result.failureReasons.empty());
}

TEST_CASE("Constraints only accept descending intersection as valid hit point") {
  auto cfg = MakeBaseConfig();
  cfg.releaseHeight = 0.66;
  cfg.zTarget = 1.82;
  cfg.windowInputMode = shootersim::WindowInputMode::kCenterAndDepth;
  cfg.openingDepth = 0.20;
  cfg.ballRadius = 0.0;
  cfg.frontMargin = 0.0;
  cfg.backMargin = 0.0;

  const double theta = shootersim::util::DegToRad(70.0);
  const double v = 10.0;
  const double distance = 0.45;  // Near ascending crossing (~0.454 m).

  // Ascending crossing exists and lies in the raw/effective window.
  const double a = -0.5 * cfg.gravity;
  const double b = v * std::sin(theta);
  const double c = cfg.releaseHeight - cfg.zTarget;
  const double d = b * b - 4.0 * a * c;
  EXPECT_GT(d, 0.0);
  const double sqrtD = std::sqrt(d);
  const double t1 = (-b - sqrtD) / (2.0 * a);
  const double t2 = (-b + sqrtD) / (2.0 * a);
  const double tAscending = std::min(t1, t2);
  const auto sAscending = shootersim::model::EvaluateAtTime(cfg, theta, v, tAscending);
  const double xFront = distance - cfg.openingDepth * 0.5;
  const double xBack = distance + cfg.openingDepth * 0.5;
  EXPECT_TRUE(sAscending.x >= xFront && sAscending.x <= xBack);

  const auto eval = shootersim::model::EvaluateConstraints(cfg, distance, theta, v);
  EXPECT_TRUE(eval.result.hasDescendingIntersection);
  EXPECT_FALSE(eval.result.inEffectiveWindow);
  EXPECT_FALSE(eval.result.overallValid);
}
