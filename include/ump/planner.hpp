// World model and the two path-planning front ends.
#pragma once
 
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
 
#include "ump/airspace.hpp"
#include "ump/energy.hpp"
#include "ump/geo.hpp"
#include "ump/terrain.hpp"
 
namespace ump {
 
/// Everything a planner needs to test a candidate state or leg.
class World {
 public:
  World(const Terrain& terrain, const Airspace& airspace, const AircraftModel& aircraft)
      : terrain_(terrain), airspace_(airspace), aircraft_(aircraft) {}
 
  /// Terrain clearance, altitude ceiling and prohibited airspace (SRS-010..012).
  bool stateValid(const Vec3& p) const;
 
  /// Collision-free straight leg, checked at collision_step_m resolution.
  bool edgeValid(const Vec3& a, const Vec3& b) const;
 
  /// Human-readable reason a state is rejected; empty if valid.
  std::string rejectReason(const Vec3& p) const;
 
  const Terrain& terrain() const { return terrain_; }
  const Airspace& airspace() const { return airspace_; }
  const AircraftModel& aircraft() const { return aircraft_; }
 
  double collision_step_m{25.0};
 
 private:
  const Terrain& terrain_;
  const Airspace& airspace_;
  const AircraftModel& aircraft_;
};
 
struct PlanRequest {
  Vec3 start;
  Vec3 goal;
  double goal_tolerance_m{75.0};
};
 
struct PlanResult {
  bool success{false};
  std::string planner;
  std::string failure;
  std::vector<Vec3> path;
  double time_s{0.0};
  double energy_wh{0.0};
  double length_m{0.0};
  std::size_t iterations{0};   ///< A*: nodes expanded. RRT*: samples drawn.
  double plan_ms{0.0};
};
 
/// Accumulate time, energy and length over a polyline.
PlanResult summarize(const AircraftModel& ac, std::vector<Vec3> path, std::string planner);
 
struct AStarParams {
  double cell_m{200.0};       ///< horizontal lattice spacing
  double alt_step_m{50.0};    ///< vertical lattice spacing
  double alt_min_msl_m{0.0};  ///< clamped up to terrain + min_agl at runtime
  double alt_max_msl_m{1200.0};
  std::size_t max_expansions{2000000};
  double heuristic_weight{1.0};  ///< 1.0 = optimal; >1 = bounded suboptimal
};
 
/// Energy-optimal search over a 26-connected 3-D lattice.
PlanResult planAStar(const World& world, const PlanRequest& req, const AStarParams& params);
 
struct RRTStarParams {
  double step_m{300.0};       ///< steer extension length
  double near_radius_m{700.0};///< rewiring neighbourhood
  double goal_bias{0.08};
  std::size_t max_iters{20000};
  std::uint32_t seed{1};
  double alt_min_msl_m{0.0};
  double alt_max_msl_m{1200.0};
};
 
/// Asymptotically optimal sampling-based planner over the same cost function.
PlanResult planRRTStar(const World& world, const PlanRequest& req, const RRTStarParams& params);
 
}
