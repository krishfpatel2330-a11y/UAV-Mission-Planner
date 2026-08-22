// Verifies SRS-012 (prohibited airspace is never entered).
#include <gtest/gtest.h>
 
#include "ump/airspace.hpp"
 
namespace {
 
ump::NoFlyZone square(double x0, double y0, double side, double floor_m, double ceiling_m) {
  ump::NoFlyZone z;
  z.id = "TEST";
  z.vertices = {{x0, y0}, {x0 + side, y0}, {x0 + side, y0 + side}, {x0, y0 + side}};
  z.floor_m = floor_m;
  z.ceiling_m = ceiling_m;
  z.finalize();
  return z;
}
 
TEST(NoFlyZone, PointInsidePolygonIsDetected) {
  const auto z = square(0, 0, 100, 0, 500);
  EXPECT_TRUE(z.contains({50, 50, 100}));
  EXPECT_FALSE(z.contains({150, 50, 100}));
  EXPECT_FALSE(z.contains({50, -10, 100}));
}
 
TEST(NoFlyZone, AltitudeBandIsRespected) {
  const auto z = square(0, 0, 100, 200, 400);
  EXPECT_TRUE(z.contains({50, 50, 300}));
  EXPECT_FALSE(z.contains({50, 50, 100}));  // below the floor
  EXPECT_FALSE(z.contains({50, 50, 500}));  // above the ceiling
}
 
TEST(NoFlyZone, SegmentCrossingIsRejectedEvenWithFreeEndpoints) {
  const auto z = square(0, 0, 100, 0, 500);
  // Both endpoints are outside, but the leg passes straight through.
  EXPECT_TRUE(z.segmentIntersects({-50, 50, 100}, {150, 50, 100}, 25.0));
}
 
TEST(NoFlyZone, OverflightAboveCeilingIsPermitted) {
  const auto z = square(0, 0, 100, 0, 500);
  EXPECT_FALSE(z.segmentIntersects({-50, 50, 600}, {150, 50, 600}, 25.0));
}
 
TEST(NoFlyZone, ThinSliverIsNotSteppedOver) {
  // A 10 m wide zone with a 50 m collision step: the sampler alone could miss
  // it, so the exact edge-crossing test must catch it.
  ump::NoFlyZone z;
  z.id = "SLIVER";
  z.vertices = {{95, -500}, {105, -500}, {105, 500}, {95, 500}};
  z.floor_m = 0;
  z.ceiling_m = 500;
  z.finalize();
  EXPECT_TRUE(z.segmentIntersects({0, 0, 100}, {400, 0, 100}, 50.0));
}
 
TEST(Airspace, ReportsViolatedZoneById) {
  ump::Airspace a;
  a.add(square(0, 0, 100, 0, 500));
  EXPECT_EQ(a.violatedBy({50, 50, 100}), "TEST");
  EXPECT_TRUE(a.violatedBy({500, 500, 100}).empty());
}
 
} 
