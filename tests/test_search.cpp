#include "TestFramework.h"

#include <algorithm>

#include "model/Search.h"
#include "util/MathUtil.h"

namespace {

shootersim::SimulationConfig MakeSearchConfig() {
  shootersim::SimulationConfig cfg;
  cfg.physicsMode = shootersim::PhysicsMode::kAnalyticNoDrag;
  cfg.releaseHeight = 0.82;
  cfg.zTarget = 2.05;

  cfg.windowInputMode = shootersim::WindowInputMode::kCenterAndDepth;
  cfg.openingDepth = 0.42;
  cfg.ballRadius = 0.09;
  cfg.frontMargin = 0.01;
  cfg.backMargin = 0.03;

  cfg.thetaMinRad = shootersim::util::DegToRad(20.0);
  cfg.thetaMaxRad = shootersim::util::DegToRad(75.0);
  cfg.thetaStepRad = shootersim::util::DegToRad(0.25);

  cfg.vMin = 5.0;
  cfg.vMax = 15.0;
  cfg.vStep = 0.04;

  cfg.zCeilingMax = 5.0;
  cfg.enableMonotonicBoundarySearch = false;
  return cfg;
}

int CountFeasibleThetas(const shootersim::SolveResult& result) {
  int count = 0;
  for (const auto& t : result.thetaScan) {
    if (t.hasValidInterval) {
      ++count;
    }
  }
  return count;
}

}  // namespace

TEST_CASE("Search finds interval and nominal inside bounds") {
  auto cfg = MakeSearchConfig();
  cfg.nominalMode = shootersim::NominalSelectionMode::kSpeedBias;
  cfg.speedBias = 0.4;

  const auto solved = shootersim::model::SolveForDistance(cfg, 3.0);
  EXPECT_TRUE(solved.hasSolution);
  EXPECT_GT(solved.bestDeltaV, 0.0);
  EXPECT_LT(solved.bestVLow, solved.bestVNominal);
  EXPECT_LT(solved.bestVNominal, solved.bestVHigh);
}

TEST_CASE("Search window-bias shifts entry point") {
  auto cfgFront = MakeSearchConfig();
  cfgFront.nominalMode = shootersim::NominalSelectionMode::kWindowBias;
  cfgFront.targetBias = 0.2;

  auto cfgBack = cfgFront;
  cfgBack.targetBias = 0.8;

  const auto rFront = shootersim::model::SolveForDistance(cfgFront, 3.0);
  const auto rBack = shootersim::model::SolveForDistance(cfgBack, 3.0);

  EXPECT_TRUE(rFront.hasSolution);
  EXPECT_TRUE(rBack.hasSolution);
  EXPECT_LT(rFront.nominalIntersection.xEntry, rBack.nominalIntersection.xEntry);
}

TEST_CASE("Search narrower window reduces or keeps deltaV") {
  auto cfgWide = MakeSearchConfig();
  auto cfgNarrow = cfgWide;
  cfgNarrow.openingDepth = 0.33;

  const auto rWide = shootersim::model::SolveForDistance(cfgWide, 3.0);
  const auto rNarrow = shootersim::model::SolveForDistance(cfgNarrow, 3.0);

  EXPECT_TRUE(rWide.hasSolution);
  EXPECT_TRUE(rNarrow.hasSolution);
  EXPECT_TRUE(rNarrow.bestDeltaV <= rWide.bestDeltaV + 1e-5);
}

TEST_CASE("Search lower ceiling reduces feasible theta count") {
  auto cfgHigh = MakeSearchConfig();
  auto cfgLow = cfgHigh;
  cfgLow.zCeilingMax = 2.6;

  const auto rHigh = shootersim::model::SolveForDistance(cfgHigh, 3.0);
  const auto rLow = shootersim::model::SolveForDistance(cfgLow, 3.0);

  EXPECT_TRUE(rHigh.hasSolution);
  EXPECT_TRUE(CountFeasibleThetas(rLow) <= CountFeasibleThetas(rHigh));
}

TEST_CASE("Search invalid effective window should fail cleanly") {
  auto cfg = MakeSearchConfig();
  cfg.openingDepth = 0.2;
  cfg.ballRadius = 0.09;
  cfg.frontMargin = 0.03;
  cfg.backMargin = 0.03;

  const auto result = shootersim::model::SolveForDistance(cfg, 3.0);
  EXPECT_FALSE(result.hasSolution);
  EXPECT_TRUE(!result.failureReason.empty());
}
