"""Render a planned mission over the DEM and no-fly zones.
 
Usage:
    ump-plan --scenario config/scenario_ridgeline.txt --out mission --dem dem.txt
    python3 tools/plot_mission.py --dem dem.txt --scenario config/scenario_ridgeline.txt \
        --mission mission.csv --out mission.png
"""
import argparse
import csv
import math
 
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
 
 
def load_dem(path):
    with open(path) as fh:
        tokens = fh.read().split()
    nx, ny, cell = int(tokens[0]), int(tokens[1]), float(tokens[2])
    z = np.array(tokens[3:3 + nx * ny], dtype=float).reshape(ny, nx)
    return z, cell
 
 
def load_zones(path):
    zones = []
    with open(path) as fh:
        for line in fh:
            line = line.split("#")[0].split()
            if not line or line[0] != "nfz":
                continue
            zid = line[1]
            ceiling = float(line[line.index("ceiling") + 1])
            coords = [float(v) for v in line[line.index("poly") + 1:]]
            pts = list(zip(coords[0::2], coords[1::2]))
            zones.append((zid, ceiling, pts))
    return zones
 
 
def load_mission(path, origin_lat, origin_lon):
    """Reproject waypoint lat/lon back into the local ENU frame for plotting.
 
    Uses the same WGS-84 radii of curvature as ump::GeoRef so the plotted track
    matches the planner's internal frame rather than drifting a few hundred
    metres, which would make a legal route look like it clips a zone boundary.
    """
    a, e2 = 6378137.0, 6.69437999014e-3
    s_lat = math.sin(math.radians(origin_lat))
    w = math.sqrt(1.0 - e2 * s_lat * s_lat)
    m_per_deg_lat = (a * (1.0 - e2) / w**3) * math.pi / 180.0
    m_per_deg_lon = (a / w) * math.cos(math.radians(origin_lat)) * math.pi / 180.0
    xs, ys, zs, agl, t, e = [], [], [], [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            xs.append((float(row["lon_deg"]) - origin_lon) * m_per_deg_lon)
            ys.append((float(row["lat_deg"]) - origin_lat) * m_per_deg_lat)
            zs.append(float(row["alt_msl_m"]))
            agl.append(float(row["agl_m"]))
            t.append(float(row["cum_time_s"]) / 60.0)
            e.append(float(row["cum_energy_wh"]))
    return map(np.array, (xs, ys, zs, agl, t, e))
 
 
def origin_from_scenario(path):
    with open(path) as fh:
        for line in fh:
            tok = line.split("#")[0].split()
            if tok and tok[0] == "origin":
                return float(tok[1]), float(tok[2])
    return 0.0, 0.0
 
 
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dem", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--mission", required=True, nargs="+",
                    help="one or more waypoint CSV files, plotted as separate routes")
    ap.add_argument("--out", default="mission.png")
    args = ap.parse_args()
 
    dem, cell = load_dem(args.dem)
    zones = load_zones(args.scenario)
    lat0, lon0 = origin_from_scenario(args.scenario)
 
    fig = plt.figure(figsize=(15, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], hspace=0.32, wspace=0.2)
    ax = fig.add_subplot(gs[:, 0])
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_e = fig.add_subplot(gs[1, 1])
 
    extent = [0, (dem.shape[1] - 1) * cell, 0, (dem.shape[0] - 1) * cell]
    im = ax.imshow(dem, origin="lower", extent=extent, cmap="terrain", alpha=0.95)
    ax.contour(dem, levels=12, origin="lower", extent=extent, colors="k",
               linewidths=0.35, alpha=0.4)
    fig.colorbar(im, ax=ax, label="terrain elevation [m MSL]", fraction=0.045)
 
    for zid, ceiling, pts in zones:
        ax.add_patch(Polygon(pts, closed=True, facecolor="crimson", alpha=0.28,
                             edgecolor="crimson", linewidth=1.6, hatch="//"))
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        ax.text(cx, cy, f"{zid}\nSFC-{int(ceiling)} m", ha="center", va="center",
                fontsize=7.5, color="darkred", weight="bold")
 
    colors = ["#0b3d91", "#e07b00", "#1b7f3b"]
    for i, path in enumerate(args.mission):
        xs, ys, zs, agl, t, e = load_mission(path, lat0, lon0)
        label = path.rsplit("/", 1)[-1].replace(".csv", "")
        c = colors[i % len(colors)]
        ax.plot(xs, ys, "-o", color=c, ms=4, lw=2.0, label=label)
        ax_alt.plot(t, zs, "-o", color=c, ms=3.5, lw=1.6, label=f"{label} MSL")
        ax_alt.plot(t, agl, "--", color=c, lw=1.0, alpha=0.7, label=f"{label} AGL")
        ax_e.plot(t, e, "-", color=c, lw=1.8, label=label)
 
    xs, ys, *_ = load_mission(args.mission[0], lat0, lon0)
    ax.plot(xs[0], ys[0], marker="^", color="white", mec="black", ms=13, lw=0, label="launch")
    ax.plot(xs[-1], ys[-1], marker="*", color="gold", mec="black", ms=18, lw=0, label="objective")
 
    ax.set_title("Planned route over terrain and prohibited airspace")
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.legend(loc="lower right", fontsize=8)
 
    ax_alt.set_title("Vertical profile")
    ax_alt.set_xlabel("elapsed time [min]")
    ax_alt.set_ylabel("altitude [m]")
    ax_alt.grid(alpha=0.3)
    ax_alt.legend(fontsize=7)
 
    ax_e.set_title("Energy consumed")
    ax_e.set_xlabel("elapsed time [min]")
    ax_e.set_ylabel("energy [Wh]")
    ax_e.grid(alpha=0.3)
    ax_e.legend(fontsize=7)
 
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print("wrote", args.out)
 
 
if __name__ == "__main__":
    main()
