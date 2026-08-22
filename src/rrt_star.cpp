#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
 
#include "ump/planner.hpp"
 
namespace ump {
namespace {
 
struct Node {
  Vec3 p;
  int parent{-1};
  double cost{0.0};  // energy from start [Wh]
};
 
/// Altitude is scaled down in the nearest-neighbour metric: a 100 m altitude
/// change is far cheaper than 100 m of horizontal travel, so an unscaled
/// Euclidean metric over-weights the vertical axis.
constexpr double kAltMetricScale = 0.25;
 
double metric(const Vec3& a, const Vec3& b) {
  const double dx = b.x - a.x, dy = b.y - a.y, dz = (b.z - a.z) * kAltMetricScale;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
 
}  // namespace
 
PlanResult planRRTStar(const World& world, const PlanRequest& req, const RRTStarParams& params) {
  const auto t0 = std::chrono::steady_clock::now();
  const AircraftModel& ac = world.aircraft();
 
  PlanResult fail;
  fail.planner = "rrtstar";
  if (!world.stateValid(req.start)) {
    fail.failure = "start state invalid: " + world.rejectReason(req.start);
    return fail;
  }
  if (!world.stateValid(req.goal)) {
    fail.failure = "goal state invalid: " + world.rejectReason(req.goal);
    return fail;
  }
 
  // Sampling box: the start/goal bounding box inflated by half its diagonal,
  // clipped to DEM coverage. Keeps samples where a detour could plausibly go.
  const double margin = 0.5 * distXY(req.start, req.goal) + 2.0 * params.step_m;
  const double x_lo = std::max(0.0, std::min(req.start.x, req.goal.x) - margin);
  const double x_hi = std::min(world.terrain().width_m(), std::max(req.start.x, req.goal.x) + margin);
  const double y_lo = std::max(0.0, std::min(req.start.y, req.goal.y) - margin);
  const double y_hi = std::min(world.terrain().height_m(), std::max(req.start.y, req.goal.y) + margin);
  const double z_lo = std::max(params.alt_min_msl_m, 0.0);
  const double z_hi = std::min(params.alt_max_msl_m, ac.max_alt_msl_m);
 
  std::mt19937 rng(params.seed);
  std::uniform_real_distribution<double> ux(x_lo, x_hi), uy(y_lo, y_hi), uz(z_lo, z_hi),
      u01(0.0, 1.0);
 
  std::vector<Node> tree;
  tree.reserve(params.max_iters + 1);
  tree.push_back(Node{req.start, -1, 0.0});
 
  const double budget = ac.usableWh();
  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  std::size_t iters = 0;
 
  for (; iters < params.max_iters; ++iters) {
    const Vec3 sample = (u01(rng) < params.goal_bias)
                            ? req.goal
                            : Vec3{ux(rng), uy(rng), uz(rng)};
 
    // Nearest node.
    int nearest = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d = metric(tree[i].p, sample);
      if (d < best_d) {
        best_d = d;
        nearest = static_cast<int>(i);
      }
    }
 
    // Steer.
    Vec3 new_p = sample;
    if (best_d > params.step_m) {
      const double t = params.step_m / best_d;
      const Vec3& np = tree[nearest].p;
      new_p = {np.x + (sample.x - np.x) * t, np.y + (sample.y - np.y) * t,
               np.z + (sample.z - np.z) * t};
    }
    new_p.z = std::clamp(new_p.z, z_lo, z_hi);
    if (!world.stateValid(new_p)) continue;
    if (!world.edgeValid(tree[nearest].p, new_p)) continue;
 
    // Choose the cheapest parent in the neighbourhood.
    int parent = nearest;
    double best_cost = tree[nearest].cost + legCost(ac, tree[nearest].p, new_p).energy_wh;
    std::vector<int> near;
    for (std::size_t i = 0; i < tree.size(); ++i) {
      if (metric(tree[i].p, new_p) > params.near_radius_m) continue;
      near.push_back(static_cast<int>(i));
      const double c = tree[i].cost + legCost(ac, tree[i].p, new_p).energy_wh;
      if (c < best_cost && world.edgeValid(tree[i].p, new_p)) {
        best_cost = c;
        parent = static_cast<int>(i);
      }
    }
    if (best_cost > budget) continue;  // SRS-030
 
    tree.push_back(Node{new_p, parent, best_cost});
    const int new_idx = static_cast<int>(tree.size()) - 1;
 
    // Rewire the neighbourhood through the new node where that is cheaper.
    for (int i : near) {
      if (i == parent) continue;
      const double c = best_cost + legCost(ac, new_p, tree[i].p).energy_wh;
      if (c < tree[i].cost - 1e-9 && world.edgeValid(new_p, tree[i].p)) {
        tree[i].parent = new_idx;
        tree[i].cost = c;
      }
    }
 
    // Goal connection: keep the best one found so far and keep sampling, which
    // is what makes the cost converge toward optimal.
    if (distXY(new_p, req.goal) <= req.goal_tolerance_m && world.edgeValid(new_p, req.goal)) {
      const double c = best_cost + legCost(ac, new_p, req.goal).energy_wh;
      if (c < best_goal_cost && c <= budget) {
        best_goal_cost = c;
        best_goal_parent = new_idx;
      }
    }
  }
 
  if (best_goal_parent < 0) {
    fail.failure = "no goal connection within " + std::to_string(params.max_iters) + " samples";
    fail.iterations = iters;
    fail.plan_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return fail;
  }
 
  std::vector<Vec3> path{req.goal};
  for (int i = best_goal_parent; i >= 0; i = tree[i].parent) path.push_back(tree[i].p);
  std::reverse(path.begin(), path.end());
 
  PlanResult r = summarize(ac, std::move(path), "rrtstar");
  r.iterations = iters;
  r.plan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return r;
}
 
}
