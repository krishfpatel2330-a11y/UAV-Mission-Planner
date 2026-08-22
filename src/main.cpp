
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
 
#include "ump/mission.hpp"
#include "ump/planner.hpp"
#include "ump/scenario.hpp"
 
namespace {
 
void usage() {
  std::cout << "usage: ump-plan --scenario <file> [options]\n"
               "  --planner astar|rrtstar   planner front end (default: astar)\n"
               "  --out <prefix>            write <prefix>.csv and <prefix>.plan\n"
               "  --cell <m>                A* horizontal lattice spacing (default 200)\n"
               "  --alt-step <m>            A* vertical lattice spacing (default 50)\n"
               "  --iters <n>               RRT* sample budget (default 20000)\n"
               "  --step <m>                RRT* steer length (default 300)\n"
               "  --seed <n>                RRT* RNG seed (default 1)\n"
               "  --no-shortcut             skip line-of-sight path smoothing\n"
               "  --dem <file>              also dump the DEM for the plotting tool\n";
}
 
std::string arg(int argc, char** argv, const std::string& flag, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i)
    if (flag == argv[i]) return argv[i + 1];
  return def;
}
 
bool flag(int argc, char** argv, const std::string& f) {
  for (int i = 1; i < argc; ++i)
    if (f == argv[i]) return true;
  return false;
}
 
}  // namespace
 
int main(int argc, char** argv) {
  if (argc < 2 || flag(argc, argv, "--help")) {
    usage();
    return argc < 2 ? 2 : 0;
  }
 
  try {
    const std::string scenario_path = arg(argc, argv, "--scenario", "");
    if (scenario_path.empty()) {
      usage();
      return 2;
    }
 
    ump::Scenario sc = ump::loadScenario(scenario_path);
    const ump::GeoRef geo(sc.origin);
    ump::World world(sc.terrain, sc.airspace, sc.aircraft);
 
    ump::PlanRequest req;
    req.start = sc.start;
    req.goal = sc.goal;
 
    const std::string which = arg(argc, argv, "--planner", "astar");
    ump::PlanResult res;
    if (which == "astar") {
      ump::AStarParams p;
      p.cell_m = std::stod(arg(argc, argv, "--cell", "200"));
      p.alt_step_m = std::stod(arg(argc, argv, "--alt-step", "50"));
      p.alt_max_msl_m = sc.aircraft.max_alt_msl_m;
      res = ump::planAStar(world, req, p);
    } else if (which == "rrtstar") {
      ump::RRTStarParams p;
      p.max_iters = static_cast<std::size_t>(std::stoul(arg(argc, argv, "--iters", "20000")));
      p.step_m = std::stod(arg(argc, argv, "--step", "300"));
      p.seed = static_cast<std::uint32_t>(std::stoul(arg(argc, argv, "--seed", "1")));
      p.alt_max_msl_m = sc.aircraft.max_alt_msl_m;
      res = ump::planRRTStar(world, req, p);
    } else {
      std::cerr << "unknown planner '" << which << "'\n";
      return 2;
    }
 
    if (!res.success) {
      std::cerr << "PLAN FAILED (" << res.planner << "): " << res.failure << "\n";
      return 1;
    }
 
    if (!flag(argc, argv, "--no-shortcut")) {
      auto smoothed = ump::shortcut(world, res.path);
      ump::PlanResult s = ump::summarize(sc.aircraft, std::move(smoothed), res.planner);
      s.iterations = res.iterations;
      s.plan_ms = res.plan_ms;
      res = std::move(s);
    }
 
    // Independent post-hoc verification: never trust the search to have been
    // the only thing enforcing the constraints (SRS-040).
    bool verified = true;
    for (std::size_t i = 1; i < res.path.size(); ++i)
      if (!world.edgeValid(res.path[i - 1], res.path[i])) verified = false;
    const bool within_budget = res.energy_wh <= sc.aircraft.usableWh();
 
    const auto wps = ump::toWaypoints(geo, world, res.path);
    double min_agl = 1e9;
    for (const auto& w : wps) min_agl = std::min(min_agl, w.agl_m);
 
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "planner        : " << res.planner << "\n"
              << "waypoints      : " << res.path.size() << "\n"
              << "ground track   : " << res.length_m / 1000.0 << " km\n"
              << "flight time    : " << res.time_s / 60.0 << " min\n"
              << "energy         : " << res.energy_wh << " Wh of "
              << sc.aircraft.usableWh() << " Wh usable ("
              << 100.0 * res.energy_wh / sc.aircraft.usableWh() << " %)\n"
              << "min clearance  : " << min_agl << " m AGL (limit "
              << sc.aircraft.min_agl_m << " m)\n"
              << "search effort  : " << res.iterations << " iterations in " << res.plan_ms
              << " ms\n"
              << "verification   : " << (verified ? "PASS" : "FAIL") << " corridor, "
              << (within_budget ? "PASS" : "FAIL") << " energy\n";
 
    const std::string out = arg(argc, argv, "--out", "");
    if (!out.empty()) {
      ump::writeWaypointCsv(out + ".csv", wps);
      ump::writeQgcPlan(out + ".plan", wps, sc.airspace, geo);
      std::cout << "wrote          : " << out << ".csv, " << out << ".plan\n";
    }
    const std::string dem = arg(argc, argv, "--dem", "");
    if (!dem.empty()) {
      sc.terrain.saveAscii(dem);
      std::cout << "wrote          : " << dem << "\n";
    }
 
    return (verified && within_budget) ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 2;
  }
}
 
