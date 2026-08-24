# UAV Mission Planner

Route planning for a fixed-wing UAV that has to avoid terrain, stay out of restricted airspace, and get home on the battery it left with. You give it an elevation model, some no-fly zones, and an aircraft, and it gives you back a route and a flight plan file.

There are two planners in here: a lattice A\* search and a sampling-based RRT\*. They solve the same problem against the same cost function, so you can run both on one scenario and see where they disagree. They usually do.

![Planned route over terrain and prohibited airspace](docs/mission_ridgeline.png)

*A\* (blue) hugs the low ground south of the threat ring. RRT\* (orange) goes over the ridge to the northwest instead. Both routes are legal. The panels on the right show altitude and energy against elapsed time.*

## What it does

Terrain comes from a digital elevation model, sampled with bilinear interpolation. The aircraft has a minimum height above ground it can never go below and a ceiling it can never go above, and both are enforced on every point of every leg.

Restricted airspace is a polygon with a floor and a ceiling altitude, so it's a box in 3D rather than a wall. That matters, because a zone with a low ceiling can legally be overflown, and the planner will do that whenever climbing over costs less energy than going around. There's a test for exactly that case.

The cost being minimized is energy, not distance. Climbing pulls more power than cruising, descending pulls less, and how long a leg takes depends on whichever limit binds first, either cruise speed over the horizontal distance or climb rate over the altitude change. If a branch of the search would run past the usable energy budget, which is pack capacity minus whatever reserve you configure, it gets pruned right there. A mission the aircraft can't actually fly comes back as a failure with a reason, not as a pretty route that lands the aircraft in a field.

After a route is found and smoothed, a separate pass re-checks every leg against every constraint. That code isn't part of the search and doesn't trust it. If the two disagree, the tool prints FAIL and exits non-zero. Output goes out as a QGroundControl `.plan` file, with the no-fly zones written alongside the waypoints as geofence exclusions, plus a CSV of the waypoints for plotting and analysis.

## Build and run

You need CMake 3.16 or newer and a C++17 compiler. GoogleTest gets fetched during configure, so there's nothing else to install.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

./build/ump-plan --scenario config/scenario_ridgeline.txt \
                 --planner astar --out mission --dem dem.txt
```

Running the reference scenario prints a summary of the route and how the verification pass went:

```
planner        : astar
waypoints      : 6
ground track   : 24.7 km
flight time    : 18.9 min
energy         : 95.1 Wh of 480.0 Wh usable (19.8 %)
min clearance  : 64.9 m AGL (limit 60.0 m)
search effort  : 84480 iterations in 1238.1 ms
verification   : PASS corridor, PASS energy
```

The figure at the top of this file comes from the plotting tool, which draws the route over the elevation model with the altitude and energy profiles beside it. Give it more than one CSV and it overlays the routes so you can compare them.

```sh
python3 tools/plot_mission.py --dem dem.txt --scenario config/scenario_ridgeline.txt \
        --mission mission.csv --out mission.png
```

## Scenario files

A scenario is a plain text file, one directive per line, with `#` for comments. Coordinates are meters east and north of the geodetic origin. Altitudes are meters, tagged either `agl` or `msl`. Terrain is either generated from a seed, which keeps the whole plan reproducible from the scenario file alone, or read from an ASCII elevation file. Each `aircraft` line overrides one field of the performance model, and each `nfz` line is one restricted polygon with its altitude band.

```
origin 29.6436 -82.3549
terrain synthetic 201 201 100 7        # nx ny cell_m seed  (or: terrain file dem.txt)

aircraft cruise_speed_mps  22
aircraft climb_power_w    900
aircraft energy_wh        600
aircraft reserve_frac    0.20
aircraft min_agl_m         60

start agl  1500  1500 120
goal  agl 18000 17500 120

nfz SAM_RING_ALPHA floor 0 ceiling 1200 poly 7000 6000 11000 6000 11000 10500 7000 10500
```

Bad input gets rejected with the line number that caused it, not a stack trace.

## How the two planners differ

A\* searches a 26-connected lattice anchored on the start point. You set the horizontal spacing with `--cell` and the vertical spacing with `--alt-step`. The heuristic is the straight-line horizontal distance to the goal times the least energy the aircraft could possibly burn per meter. Since any leg takes at least as long as the horizontal distance divided by cruise speed, and burns at least the lowest of the three power settings while doing it, that product can never overestimate what's left to spend. That's what makes the heuristic admissible and the result optimal on the lattice. The test suite checks that bound against twenty thousand randomly generated legs instead of just claiming it in a comment.

RRT\* works differently. It samples points in the continuous space, steers toward each one, hooks the new node up to the cheapest parent nearby, and rewires any neighbors it can improve. It keeps sampling after it first reaches the goal, and that's the part that drives the cost down over time. Whatever either planner returns then goes through the same shortcutter, which drops an intermediate waypoint only if the leg replacing it is both collision-free and no more expensive than what it replaced.

Here's what they do on the reference scenario. A\* at 400 m spacing flies 25.0 km for 96.4 Wh and takes 461 ms to search. Tighten it to 300 m and you get 24.5 km at 94.7 Wh in 696 ms, at 200 m it's 24.7 km and 95.1 Wh in 1238 ms, and at 150 m it's 24.5 km and 94.7 Wh in 1823 ms. RRT\* with 5,000 samples finds 24.2 km for 89.6 Wh in 47 ms, drops to 86.2 Wh at 15,000 samples, and reaches 85.6 Wh at 40,000.

So RRT\* wins by about ten percent on energy, which surprised me at first, since A\* is the one with the optimality proof. The catch is what it's optimal over. A\* is optimal on its lattice, and that lattice only lets the aircraft fly in 45 degree heading increments. The real best route isn't in the search space at all, and shrinking the cell size doesn't help, because the thing that's quantized is heading, not position. You can see it in the numbers above: the A\* energy figures bounce around 95 Wh and stop improving. RRT\* samples continuously, so it keeps closing in on the true optimum as you give it more samples. What you give up is repeatability. A\* returns the same route every single run and RRT\* doesn't.

## Requirements and testing

`docs/requirements.md` has a numbered requirements list where each requirement points at the test that verifies it, in the style of DO-178C traceability. Twenty-five tests cover the coordinate math, the energy model, constraint enforcement, heuristic admissibility, whether shortcutting can ever make a route illegal, and export integrity. CI builds Debug and Release, runs everything again under AddressSanitizer and UndefinedBehaviorSanitizer, and plans an end-to-end mission with both planners. The library builds clean under `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.

## Layout

Headers live in `include/ump`: `geo`, `terrain`, `airspace`, `energy`, `planner`, `mission`, and `scenario`. Implementations are in `src`, with each planner in its own file. Tests are in `tests`, named after the behavior they check rather than the function they call. `tools` has the plotting script, `config` has scenario files, and `docs` has the requirements matrix.

## What it doesn't do

A few limits are on purpose, and they're written up with reasons at the bottom of `docs/requirements.md`. Legs are straight lines, so a real fixed-wing aircraft with a turn radius couldn't fly these routes as-is. Dubins-airplane steering is the obvious next step. Routes can also pass right along the edge of a restricted zone, where real planning would add a lateral buffer sized by how much you trust your navigation. Zones never move or activate, and there's no wind, which is the biggest gap in the energy model by a wide margin. And it plans for one aircraft at a time with no deconfliction between them.
