// Verifies SRS-020 (admissible heuristic) and SRS-030 (energy budget).
#include <gtest/gtest.h>
 
#include <random>
 
#include "ump/energy.hpp"
 
namespace {
 
TEST(Energy, LevelCruiseMatchesHandCalculation) {
  ump::AircraftModel ac;  // 22 m/s, 350 W
  const ump::LegCost c = ump::legCost(ac, {0, 0, 300}, {2200, 0, 300});
  EXPECT_NEAR(c.time_s, 100.0, 1e-6);
  EXPECT_NEAR(c.energy_wh, 350.0 * 100.0 / 3600.0, 1e-9);
}
 
TEST(Energy, ClimbIsRateLimitedNotSpeedLimited) {
  ump::AircraftModel ac;
  // 100 m horizontally but 90 m up: the 3 m/s climb rate governs (30 s), not
  // the 4.5 s the horizontal distance would take.
  const ump::LegCost c = ump::legCost(ac, {0, 0, 300}, {100, 0, 390});
  EXPECT_NEAR(c.time_s, 30.0, 1e-6);
  EXPECT_NEAR(c.energy_wh, 900.0 * 30.0 / 3600.0, 1e-9);
}
 
TEST(Energy, DescentCostsLessThanCruise) {
  ump::AircraftModel ac;
  const double cruise = ump::legCost(ac, {0, 0, 300}, {1000, 0, 300}).energy_wh;
  const double descent = ump::legCost(ac, {0, 0, 300}, {1000, 0, 280}).energy_wh;
  EXPECT_LT(descent, cruise);
}
 
TEST(Energy, ReserveIsWithheldFromTheUsableBudget) {
  ump::AircraftModel ac;
  ac.energy_wh = 500.0;
  ac.reserve_frac = 0.25;
  EXPECT_NEAR(ac.usableWh(), 375.0, 1e-9);
}
 
// The A* heuristic is h = minEnergyPerMeterWh * horizontal distance. For the
// heuristic to be admissible, no single leg may ever cost less than that bound.
TEST(Energy, HeuristicLowerBoundIsNeverExceededByAnyLeg) {
  ump::AircraftModel ac;
  const double bound = ump::minEnergyPerMeterWh(ac);
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dxy(-2000.0, 2000.0), dz(-300.0, 300.0);
  for (int i = 0; i < 20000; ++i) {
    const ump::Vec3 a{0, 0, 500};
    const ump::Vec3 b{dxy(rng), dxy(rng), 500 + dz(rng)};
    const double actual = ump::legCost(ac, a, b).energy_wh;
    EXPECT_GE(actual + 1e-12, bound * ump::distXY(a, b)) << "leg " << i << " broke admissibility";
  }
}
 
}
