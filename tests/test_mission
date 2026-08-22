// Verifies SRS-021 (shortcutting is safe) and SRS-050/051 (export integrity).
#include <gtest/gtest.h>
 
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
 
#include "ump/mission.hpp"
#include "ump/scenario.hpp"
 
namespace {
 
ump::Terrain flatTerrain() {
  return ump::Terrain(101, 101, 100.0, std::vector<float>(101 * 101, 100.0f));
}
 
TEST(Shortcut, NeverProducesAnIllegalOrMoreExpensiveLeg) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::NoFlyZone z;
  z.id = "BLOCK";
  z.vertices = {{3000, 0}, {3600, 0}, {3600, 4000}, {3000, 4000}};
  z.floor_m = 0.0;
  z.ceiling_m = 1200.0;
  air.add(z);
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
 
  ump::PlanRequest req;
  req.start = {1000, 1000, 300};
  req.goal = {6000, 1000, 300};
  ump::AStarParams p;
  p.cell_m = 250.0;
  const auto raw = ump::planAStar(world, req, p);
  ASSERT_TRUE(raw.success) << raw.failure;
 
  const auto smoothed = ump::shortcut(world, raw.path);
  EXPECT_LE(smoothed.size(), raw.path.size());
  for (std::size_t i = 1; i < smoothed.size(); ++i)
    EXPECT_TRUE(world.edgeValid(smoothed[i - 1], smoothed[i]));
  const auto s = ump::summarize(ac, smoothed, "astar");
  EXPECT_LE(s.energy_wh, raw.energy_wh + 1e-6);
  EXPECT_EQ(smoothed.front().x, raw.path.front().x);
  EXPECT_EQ(smoothed.back().x, raw.path.back().x);
}
 
TEST(Waypoints, CumulativeTimeAndEnergyAreMonotonic) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
  const ump::GeoRef geo(ump::LatLon{29.6436, -82.3549});
 
  const std::vector<ump::Vec3> path{{0, 0, 300}, {1000, 0, 350}, {2000, 500, 300}};
  const auto wps = ump::toWaypoints(geo, world, path);
  ASSERT_EQ(wps.size(), 3u);
  for (std::size_t i = 1; i < wps.size(); ++i) {
    EXPECT_GT(wps[i].cum_time_s, wps[i - 1].cum_time_s);
    EXPECT_GT(wps[i].cum_energy_wh, wps[i - 1].cum_energy_wh);
    EXPECT_EQ(wps[i].seq, static_cast<int>(i));
  }
  EXPECT_NEAR(wps[0].agl_m, 200.0, 1e-6);
}
 
TEST(GeoRef, RoundTripsToWithinACentimetre) {
  const ump::GeoRef geo(ump::LatLon{29.6436, -82.3549});
  const ump::Vec3 p{12345.0, -6789.0, 400.0};
  const ump::LatLon ll = geo.toLatLon(p);
  const ump::Vec3 q = geo.toEnu(ll, p.z);
  EXPECT_NEAR(p.x, q.x, 0.01);
  EXPECT_NEAR(p.y, q.y, 0.01);
}
 
TEST(Export, QgcPlanContainsOneItemPerWaypoint) {
  const auto terrain = flatTerrain();
  ump::Airspace air;
  ump::AircraftModel ac;
  ump::World world(terrain, air, ac);
  const ump::GeoRef geo(ump::LatLon{29.6436, -82.3549});
  const std::vector<ump::Vec3> path{{0, 0, 300}, {1000, 0, 350}, {2000, 500, 300}};
  const auto wps = ump::toWaypoints(geo, world, path);
 
  const std::string tmp = "test_export.plan";
  ump::writeQgcPlan(tmp, wps, air, geo);
  std::ifstream in(tmp);
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  std::remove(tmp.c_str());
 
  EXPECT_NE(text.find("\"fileType\": \"Plan\""), std::string::npos);
  std::size_t items = 0, pos = 0;
  while ((pos = text.find("\"command\": 16", pos)) != std::string::npos) {
    ++items;
    pos += 5;
  }
  EXPECT_EQ(items, wps.size());
}
 
TEST(Scenario, ReportsTheOffendingLineNumber) {
  const std::string tmp = "bad_scenario.txt";
  {
    std::ofstream out(tmp);
    out << "origin 29.6 -82.3\n";
    out << "terrain synthetic 51 51 100 7\n";
    out << "aircraft not_a_field 5\n";
  }
  try {
    ump::loadScenario(tmp);
    std::remove(tmp.c_str());
    FAIL() << "expected a parse error";
  } catch (const std::exception& e) {
    std::remove(tmp.c_str());
    EXPECT_NE(std::string(e.what()).find("line 3"), std::string::npos);
  }
}
 
}
