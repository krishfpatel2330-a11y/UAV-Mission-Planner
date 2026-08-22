#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>
 
#include "ump/planner.hpp"
 
namespace ump {
namespace {
 
/// Lattice index packed into a single key. Ranges are bounded by the DEM and
/// the altitude band, so 21 bits per axis is ample and collision-free.
using Key = std::uint64_t;
 
struct Index {
  std::int32_t ix, iy, iz;
};
 
inline Key pack(const Index& c) {
  const std::uint64_t x = static_cast<std::uint64_t>(c.ix + (1 << 20)) & 0x1FFFFF;
  const std::uint64_t y = static_cast<std::uint64_t>(c.iy + (1 << 20)) & 0x1FFFFF;
  const std::uint64_t z = static_cast<std::uint64_t>(c.iz + (1 << 20)) & 0x1FFFFF;
  return (x << 42) | (y << 21) | z;
}
 
struct QNode {
  double f;
  Key key;
  bool operator>(const QNode& o) const { return f > o.f; }
};
 
}  // namespace
 
PlanResult planAStar(const World& world, const PlanRequest& req, const AStarParams& params) {
  const auto t0 = std::chrono::steady_clock::now();
  const AircraftModel& ac = world.aircraft();
 
  PlanResult fail;
  fail.planner = "astar";
 
  const double cell = std::max(params.cell_m, 1.0);
  const double alt_step = std::max(params.alt_step_m, 1.0);
 
  // Lattice is anchored on the start state so the start is exactly on-grid.
  const Vec3 anchor = req.start;
  const auto toPos = [&](const Index& c) {
    return Vec3{anchor.x + c.ix * cell, anchor.y + c.iy * cell, anchor.z + c.iz * alt_step};
  };
 
  const double z_lo = std::max(params.alt_min_msl_m, 0.0);
  const double z_hi = std::min(params.alt_max_msl_m, ac.max_alt_msl_m);
 
  if (!world.stateValid(req.start)) {
    fail.failure = "start state invalid: " + world.rejectReason(req.start);
    return fail;
  }
  if (!world.stateValid(req.goal)) {
    fail.failure = "goal state invalid: " + world.rejectReason(req.goal);
    return fail;
  }
 
  // The lattice is anchored on the start, so the goal will not generally lie
  // on a node. Accept any node within one cell of the goal and close the plan
  // with an explicit terminal leg, which is collision-checked and costed like
  // any other leg. Documented in docs/requirements.md under SRS-022.
  const double goal_accept_m = std::max(req.goal_tolerance_m, cell);
 
  const double h_scale = minEnergyPerMeterWh(ac) * params.heuristic_weight;
  const auto heuristic = [&](const Vec3& p) { return h_scale * distXY(p, req.goal); };
 
  std::unordered_map<Key, double> g;
  std::unordered_map<Key, Key> parent;
  std::unordered_map<Key, Index> idx;
  std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;
 
  const Index start_idx{0, 0, 0};
  const Key start_key = pack(start_idx);
  g[start_key] = 0.0;
  idx[start_key] = start_idx;
  open.push({heuristic(req.start), start_key});
 
  const double budget = ac.usableWh();
  std::size_t expansions = 0;
  bool found = false;
  Key goal_key = start_key;
 
  while (!open.empty()) {
    const QNode cur = open.top();
    open.pop();
    const auto g_it = g.find(cur.key);
    if (g_it == g.end()) continue;
    const double g_cur = g_it->second;
    // Stale queue entry (lazy deletion).
    if (cur.f > g_cur + heuristic(toPos(idx[cur.key])) + 1e-9) continue;
 
    const Index c = idx[cur.key];
    const Vec3 p = toPos(c);
 
    if (distXY(p, req.goal) <= goal_accept_m && world.edgeValid(p, req.goal)) {
      goal_key = cur.key;
      found = true;
      break;
    }
 
    if (++expansions > params.max_expansions) {
      fail.failure = "expansion limit reached (" + std::to_string(params.max_expansions) + ")";
      fail.iterations = expansions;
      return fail;
    }
 
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          const Index nc{c.ix + dx, c.iy + dy, c.iz + dz};
          const Vec3 np = toPos(nc);
          if (np.z < z_lo || np.z > z_hi) continue;
          if (!world.stateValid(np)) continue;
          if (!world.edgeValid(p, np)) continue;
 
          const double g_new = g_cur + legCost(ac, p, np).energy_wh;
          if (g_new > budget) continue;  // SRS-030: prune infeasible branches
 
          const Key nk = pack(nc);
          const auto it = g.find(nk);
          if (it == g.end() || g_new < it->second - 1e-12) {
            g[nk] = g_new;
            parent[nk] = cur.key;
            idx[nk] = nc;
            open.push({g_new + heuristic(np), nk});
          }
        }
      }
    }
  }
 
  if (!found) {
    fail.failure = fail.failure.empty()
                       ? "reachable lattice exhausted: no legal route within the energy budget"
                       : fail.failure;
    fail.iterations = expansions;
    fail.plan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                       .count();
    return fail;
  }
 
  std::vector<Vec3> path;
  for (Key k = goal_key;; k = parent[k]) {
    path.push_back(toPos(idx[k]));
    if (k == start_key) break;
  }
  std::reverse(path.begin(), path.end());
  path.push_back(req.goal);
 
  PlanResult r = summarize(ac, std::move(path), "astar");
  r.iterations = expansions;
  r.plan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (r.energy_wh > budget) {
    r.success = false;
    r.failure = "path exceeds usable energy budget";
  }
  return r;
}
 
}
