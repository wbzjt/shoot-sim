#include "TestFramework.h"

#include <cmath>

#include "model/Ballistics.h"
#include "util/MathUtil.h"

TEST_CASE("Ballistics evaluate x(t), z(t)") {
  shootersim::SimulationConfig cfg;
  cfg.physicsMode = shootersim::PhysicsMode::kAnalyticNoDrag;
  cfg.releaseHeight = 1.0;
  cfg.gravity = 9.80665;

  const double theta = shootersim::util::DegToRad(45.0);
  const double v = 10.0;
  const double t = 0.5;

  const auto s = shootersim::model::EvaluateAtTime(cfg, theta, v, t);

  const double expectedX = v * std::cos(theta) * t;
  const double expectedZ = cfg.releaseHeight + v * std::sin(theta) * t - 0.5 * cfg.gravity * t * t;
  EXPECT_NEAR(s.x, expectedX, 1e-9);
  EXPECT_NEAR(s.z, expectedZ, 1e-9);
  EXPECT_NEAR(s.vx, v * std::cos(theta), 1e-9);
}

TEST_CASE("Ballistics apex calculation") {
  shootersim::SimulationConfig cfg;
  cfg.physicsMode = shootersim::PhysicsMode::kAnalyticNoDrag;
  cfg.releaseHeight = 0.8;

  const double theta = shootersim::util::DegToRad(60.0);
  const double v = 12.0;

  const auto apex = shootersim::model::ComputeApex(cfg, theta, v);
  EXPECT_TRUE(apex.valid);

  const double tExpected = v * std::sin(theta) / cfg.gravity;
  const auto s = shootersim::model::EvaluateAtTime(cfg, theta, v, tExpected);
  EXPECT_NEAR(apex.tApex, tExpected, 1e-9);
  EXPECT_NEAR(apex.zApex, s.z, 1e-9);
}

TEST_CASE("Ballistics descending-root intersection at target height") {
  shootersim::SimulationConfig cfg;
  cfg.physicsMode = shootersim::PhysicsMode::kAnalyticNoDrag;
  cfg.releaseHeight = 1.0;

  const double theta = shootersim::util::DegToRad(75.0);
  const double v = 10.0;
  const double zTarget = 2.0;

  const auto hit = shootersim::model::ComputeIntersectionAtHeight(cfg, theta, v, zTarget);
  EXPECT_TRUE(hit.hasIntersection);
  EXPECT_TRUE(hit.descending);

  const auto apex = shootersim::model::ComputeApex(cfg, theta, v);
  EXPECT_GT(hit.tEntry, apex.tApex);

  const auto atHit = shootersim::model::EvaluateAtTime(cfg, theta, v, hit.tEntry);
  EXPECT_NEAR(atHit.z, zTarget, 1e-6);

  const double a = -0.5 * cfg.gravity;
  const double b = v * std::sin(theta);
  const double c = cfg.releaseHeight - zTarget;
  const double d = b * b - 4.0 * a * c;
  EXPECT_GT(d, 0.0);
  const double t1 = (-b - std::sqrt(d)) / (2.0 * a);
  const double t2 = (-b + std::sqrt(d)) / (2.0 * a);
  const double expectedDescending = std::max(t1, t2);
  EXPECT_NEAR(hit.tEntry, expectedDescending, 1e-6);
}

TEST_CASE("Ballistics no real root should return no intersection") {
  shootersim::SimulationConfig cfg;
  cfg.physicsMode = shootersim::PhysicsMode::kAnalyticNoDrag;
  cfg.releaseHeight = 1.0;

  const double theta = shootersim::util::DegToRad(35.0);
  const double v = 6.0;

  const auto hit = shootersim::model::ComputeIntersectionAtHeight(cfg, theta, v, 8.0);
  EXPECT_FALSE(hit.hasIntersection);
}
