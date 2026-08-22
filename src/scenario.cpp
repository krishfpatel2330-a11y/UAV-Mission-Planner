#include "ump/scenario.hpp"
 
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
 
namespace ump {
namespace {
 
[[noreturn]] void bad(int line, const std::string& msg) {
  throw std::runtime_error("scenario line " + std::to_string(line) + ": " + msg);
}
 
double toDouble(const std::string& s, int line) {
  try {
    return std::stod(s);
  } catch (...) {
    bad(line, "expected a number, got '" + s + "'");
  }
}
 
}  // namespace
 
Scenario loadScenario(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open scenario " + path);
 
  Scenario sc;
  bool have_terrain = false;
  std::string raw;
  int line_no = 0;
 
  while (std::getline(in, raw)) {
    ++line_no;
    const std::size_t hash = raw.find('#');
    if (hash != std::string::npos) raw = raw.substr(0, hash);
    std::istringstream ls(raw);
    std::vector<std::string> tok;
    for (std::string t; ls >> t;) tok.push_back(t);
    if (tok.empty()) continue;
 
    const std::string& key = tok[0];
 
    if (key == "origin") {
      if (tok.size() != 3) bad(line_no, "usage: origin <lat_deg> <lon_deg>");
      sc.origin = {toDouble(tok[1], line_no), toDouble(tok[2], line_no)};
 
    } else if (key == "terrain") {
      if (tok.size() >= 2 && tok[1] == "synthetic") {
        if (tok.size() != 6) bad(line_no, "usage: terrain synthetic <nx> <ny> <cell_m> <seed>");
        sc.terrain = Terrain::synthetic(static_cast<int>(toDouble(tok[2], line_no)),
                                        static_cast<int>(toDouble(tok[3], line_no)),
                                        toDouble(tok[4], line_no),
                                        static_cast<std::uint32_t>(toDouble(tok[5], line_no)));
      } else if (tok.size() == 3 && tok[1] == "file") {
        sc.terrain = Terrain::loadAscii(tok[2]);
      } else {
        bad(line_no, "usage: terrain synthetic <nx> <ny> <cell_m> <seed> | terrain file <path>");
      }
      have_terrain = true;
 
    } else if (key == "aircraft") {
      if (tok.size() != 3) bad(line_no, "usage: aircraft <field> <value>");
      const std::string& f = tok[1];
      const double v = toDouble(tok[2], line_no);
      if (f == "cruise_speed_mps") sc.aircraft.cruise_speed_mps = v;
      else if (f == "cruise_power_w") sc.aircraft.cruise_power_w = v;
      else if (f == "climb_power_w") sc.aircraft.climb_power_w = v;
      else if (f == "descent_power_w") sc.aircraft.descent_power_w = v;
      else if (f == "max_climb_rate_mps") sc.aircraft.max_climb_rate_mps = v;
      else if (f == "max_sink_rate_mps") sc.aircraft.max_sink_rate_mps = v;
      else if (f == "energy_wh") sc.aircraft.energy_wh = v;
      else if (f == "reserve_frac") sc.aircraft.reserve_frac = v;
      else if (f == "min_agl_m") sc.aircraft.min_agl_m = v;
      else if (f == "max_alt_msl_m") sc.aircraft.max_alt_msl_m = v;
      else bad(line_no, "unknown aircraft field '" + f + "'");
 
    } else if (key == "start" || key == "goal") {
      if (tok.size() != 5) bad(line_no, "usage: " + key + " <agl|msl> <x_m> <y_m> <alt_m>");
      const bool agl = tok[1] == "agl";
      if (!agl && tok[1] != "msl") bad(line_no, "altitude datum must be 'agl' or 'msl'");
      const Vec3 p{toDouble(tok[2], line_no), toDouble(tok[3], line_no),
                   toDouble(tok[4], line_no)};
      if (key == "start") { sc.start = p; sc.start_alt_is_agl = agl; }
      else { sc.goal = p; sc.goal_alt_is_agl = agl; }
 
    } else if (key == "nfz") {
      // nfz <id> floor <m> ceiling <m> poly x1 y1 x2 y2 ...
      if (tok.size() < 12) bad(line_no, "no-fly zone needs an id, a band and >= 3 vertices");
      NoFlyZone z;
      z.id = tok[1];
      std::size_t i = 2;
      if (tok[i] != "floor") bad(line_no, "expected 'floor'");
      z.floor_m = toDouble(tok[i + 1], line_no);
      i += 2;
      if (tok[i] != "ceiling") bad(line_no, "expected 'ceiling'");
      z.ceiling_m = toDouble(tok[i + 1], line_no);
      i += 2;
      if (tok[i] != "poly") bad(line_no, "expected 'poly'");
      ++i;
      if ((tok.size() - i) % 2 != 0) bad(line_no, "polygon needs an even number of coordinates");
      for (; i + 1 < tok.size(); i += 2)
        z.vertices.push_back({toDouble(tok[i], line_no), toDouble(tok[i + 1], line_no)});
      if (z.vertices.size() < 3) bad(line_no, "polygon needs at least 3 vertices");
      sc.airspace.add(std::move(z));
 
    } else {
      bad(line_no, "unknown directive '" + key + "'");
    }
  }
 
  if (!have_terrain) throw std::runtime_error("scenario: no terrain directive");
 
  // Resolve AGL altitudes against the DEM now that terrain is loaded.
  if (sc.start_alt_is_agl) sc.start.z += sc.terrain.elevation(sc.start.x, sc.start.y);
  if (sc.goal_alt_is_agl) sc.goal.z += sc.terrain.elevation(sc.goal.x, sc.goal.y);
  return sc;
}
 
}
