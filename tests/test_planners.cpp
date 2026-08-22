#include <gtest/gtest.h>
 
#include <vector>
 
#include "ump/planner.hpp"
 
namespace {
 
/// Flat 10 km x 10 km DEM at 100 m elevation, 100 m posting.
ump::Terrain flatTerrain() {
  return ump::Terrain(101, 101, 100.0, std::vector<float>(101 * 101, 100.0f));
}
 
ump::NoFlyZone wall(double x0, double x1, double y0, double y1, double ceiling_m) {
  ump::NoFlyZone z;
  z.id = "WALL";
  z.vertices = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  z.floor_m = 0.0;
  z.ceiling_m = ceiling_m;
  z.finalize();
  return z;
}
 
/// Every leg of a returned path must be independently valid.
void expectCorridorLegal(const ump::World& w, const ump::PlanResult& r) {
  ASSERT_TRUE(r.success) << r.failure;
  ASSERT_GE(r.path.size(), 2u);
  for (std::size_t i = 1; i < r.path.size(); ++i)
    EXPECT_TRUE(w.edgeValid(r.path[i - 1], r.path[i]))
        << "leg " << i << " violates a constraint";
}
 
TEST(AStar, FindsDirectPathInFreeAirspace) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {6000, 1000, 300};
 
  ump::AStarParams p;
  p.cell_m = 250.0;
  const auto r = ump::planAStar(world, req, p);
  expectCorridorLegal(world, r);
  // With no obstacles the ground track should be within a few percent of the
  // straight-line distance.
  EXPECT_LT(r.length_m, 1.05 * ump::dist3(req.start, req.goal));
}
 
TEST(AStar, RoutesAroundOrOverAProhibitedVolume) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  air.add(wall(3000, 3600, 0, 4000, 1200));  // taller than the ceiling: must go around
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {6000, 1000, 300};
 
  ump::AStarParams p;
  p.cell_m = 250.0;
  const auto r = ump::planAStar(world, req, p);
  expectCorridorLegal(world, r);
  EXPECT_GT(r.length_m, ump::dist3(req.start, req.goal));  // a detour was required
  for (const auto& v : r.path) EXPECT_FALSE(air.blocked(v));
}
 
TEST(AStar, ClimbsOverAZoneWithALowCeilingRatherThanDetouring) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  air.add(wall(3000, 3600, -20000, 20000, 400));  // unflankable, but low
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 5000, 300};
  req.goal = {6000, 5000, 300};
 
  ump::AStarParams p;
  p.cell_m = 250.0;
  p.alt_step_m = 50.0;
  const auto r = ump::planAStar(world, req, p);
  expectCorridorLegal(world, r);
  double peak = 0.0;
  for (const auto& v : r.path) peak = std::max(peak, v.z);
  EXPECT_GT(peak, 400.0) << "planner should have overflown the zone";
}
 
TEST(AStar, RespectsTerrainClearance) {
  // A ridge 400 m high crossing the direct route.
  std::vector<float> z(101 * 101, 100.0f);
  for (int j = 0; j < 101; ++j)
    for (int i = 30; i < 36; ++i) z[static_cast<std::size_t>(j) * 101 + i] = 400.0f;
  const ump::Terrain terrain(101, 101, 100.0, z);
  ump::Airspace air;
  ump::AircraftModel ac;
  ac.min_agl_m = 60.0;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 5000, 300};
  req.goal = {6000, 5000, 300};
 
  ump::AStarParams p;
  p.cell_m = 250.0;
  const auto r = ump::planAStar(world, req, p);
  expectCorridorLegal(world, r);
  for (const auto& v : r.path)
    EXPECT_GE(v.z - terrain.elevation(v.x, v.y), ac.min_agl_m - 1e-6);
}
 
TEST(AStar, FailsCleanlyWhenTheEnergyBudgetCannotCoverTheMission) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::AircraftModel ac;
  ac.energy_wh = 2.0;  // a couple of minutes of flight
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {9000, 9000, 300};
 
  ump::AStarParams p;
  p.cell_m = 500.0;
  const auto r = ump::planAStar(world, req, p);
  EXPECT_FALSE(r.success);
  EXPECT_FALSE(r.failure.empty());
}
 
TEST(AStar, RejectsAnInvalidStartWithADiagnosis) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 120};  // 20 m AGL, below the 60 m floor
  req.goal = {6000, 1000, 300};
  const auto r = ump::planAStar(world, req, ump::AStarParams{});
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.failure.find("clearance"), std::string::npos);
}
 
TEST(RRTStar, ReturnsALegalPathAroundAProhibitedVolume) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  air.add(wall(3000, 3600, 0, 4000, 1200));
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {6000, 1000, 300};
 
  ump::RRTStarParams p;
  p.max_iters = 4000;
  p.step_m = 400.0;
  p.seed = 3;
  const auto r = ump::planRRTStar(world, req, p);
  expectCorridorLegal(world, r);
  for (const auto& v : r.path) EXPECT_FALSE(air.blocked(v));
}
 
TEST(RRTStar, CostImprovesWithMoreSamples) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  air.add(wall(3000, 3600, 0, 4000, 1200));
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {6000, 1000, 300};
 
  ump::RRTStarParams few;
  few.max_iters = 600;
  few.step_m = 400.0;
  few.seed = 11;
  ump::RRTStarParams many = few;
  many.max_iters = 6000;
 
  const auto a = ump::planRRTStar(world, req, few);
  const auto b = ump::planRRTStar(world, req, many);
  ASSERT_TRUE(b.success) << b.failure;
  if (a.success) EXPECT_LE(b.energy_wh, a.energy_wh * 1.02);
}
 
// A* on this lattice must not be beaten by the straight-line lower bound.
TEST(AStar, CostIsNeverBelowTheStraightLineEnergyBound) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {8000, 6000, 300};
 
  ump::AStarParams p;
  p.cell_m = 250.0;
  const auto r = ump::planAStar(world, req, p);
  ASSERT_TRUE(r.success) << r.failure;
  const double bound = ump::minEnergyPerMeterWh(ac) * ump::distXY(req.start, req.goal);
  EXPECT_GE(r.energy_wh + 1e-9, bound);
}
 
}
