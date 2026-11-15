#!/usr/bin/env python3
"""
Plot the KPIs of the AI Emergency Network simulations.

    python tools/analyse_results.py                       # all results in simulations/results
    python tools/analyse_results.py --results simulations/results --out plots

Figures (PNG, in --out):
  coverage_vs_time_<config>.png   coverage ratio and drones on post over time (one run)
  coverage_vs_fleet.png           mean coverage / planned coverage vs number of drones (FleetSweep)
  sos_delivery_vs_fleet.png       share of SOS messages delivered vs number of drones (FleetSweep)
And a CSV summary per run (summary.csv).

Needs: opp_scavetool (OMNeT++ environment: run from the OMNeT++ shell / mingwenv),
       pandas, matplotlib  (pip install pandas matplotlib)
"""
import argparse
import glob
import io
import os
import subprocess
import sys

try:
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("pip install pandas matplotlib")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))


def scavetool_csv(files, kind, filter_expr=None):
    """Export scalars or vectors with opp_scavetool, as a long CSV (CSV-R)."""
    cmd = ["opp_scavetool", "export", "-T", kind, "-F", "CSV-R", "-o", "-"]
    if filter_expr:
        cmd += ["-f", filter_expr]
    cmd += files
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    return pd.read_csv(io.StringIO(out), low_memory=False)


def run_attrs(df):
    """Per-run attributes (configname, N, repetition) from the 'runattr'/'itervar' rows."""
    attrs = df[df["type"].isin(["runattr", "itervar"])].pivot_table(
        index="run", columns="attrname", values="attrvalue", aggfunc="first")
    return attrs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=os.path.join(ROOT, "simulations", "results"))
    ap.add_argument("--out", default=os.path.join(ROOT, "plots"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    scas = sorted(glob.glob(os.path.join(args.results, "*.sca")))
    vecs = sorted(glob.glob(os.path.join(args.results, "*.vec")))
    if not scas:
        sys.exit("No .sca file in %s: run a simulation first" % args.results)

    sca = scavetool_csv(scas, "s")
    attrs = run_attrs(sca)
    scalars = sca[sca["type"] == "scalar"]

    # ---- per-run summary
    hive = scalars[scalars["module"].str.endswith("truck.hive")]
    summary = hive.pivot_table(index="run", columns="name", values="value", aggfunc="first")
    sent = scalars[(scalars["name"] == "packetSent:count") & scalars["module"].str.contains(r"\.user\[\d+\]\.app\[0\]$")]
    rcvd = scalars[(scalars["name"] == "packetReceived:count") & scalars["module"].str.endswith("truck.app[0]")]
    summary["sosSent"] = sent.groupby("run")["value"].sum()
    summary["sosDelivered"] = rcvd.groupby("run")["value"].sum()
    summary["sosDeliveryRatio"] = summary["sosDelivered"] / summary["sosSent"]
    summary = summary.join(attrs[[c for c in ("configname", "N", "repetition") if c in attrs.columns]])
    summary.to_csv(os.path.join(args.out, "summary.csv"))
    cols = [c for c in ("configname", "numDrones", "numPosts", "plannedCoverage", "meanCoverage",
                        "timeTo50pctCoverage", "numBatterySwaps", "numCoverageGaps", "sosDeliveryRatio")
            if c in summary.columns]
    print(summary[cols].to_string())

    # ---- fleet sweep curves
    sweep = summary[summary.get("configname", "") == "FleetSweep"] if "configname" in summary else summary.iloc[0:0]
    if len(sweep):
        g = sweep.groupby("numDrones")
        fig, ax = plt.subplots(figsize=(7, 4.2))
        for col, label in (("plannedCoverage", "couverture planifiée"), ("meanCoverage", "couverture moyenne sur 2 h")):
            m, s = g[col].mean() * 100, g[col].std().fillna(0) * 100
            ax.errorbar(m.index, m.values, yerr=s.values, marker="o", capsize=3, label=label)
        ax.set_xlabel("nombre de drones relais")
        ax.set_ylabel("téléphones couverts (%)")
        ax.set_ylim(0, 100)
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_title("Couverture selon la taille de la flotte (crue de la Bhotekoshi)")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "coverage_vs_fleet.png"), dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 4.2))
        m, s = g["sosDeliveryRatio"].mean() * 100, g["sosDeliveryRatio"].std().fillna(0) * 100
        ax.errorbar(m.index, m.values, yerr=s.values, marker="o", capsize=3)
        ax.set_xlabel("nombre de drones relais")
        ax.set_ylabel("messages SOS livrés (%)")
        ax.set_ylim(0, 100)
        ax.grid(alpha=0.3)
        ax.set_title("Messages SOS livrés au camion")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "sos_delivery_vs_fleet.png"), dpi=150)
        plt.close(fig)

    # ---- time series of single runs
    if vecs:
        # only the hive's vectors: exporting every radio vector would not fit in memory
        vec = scavetool_csv(vecs, "v", 'module =~ "**.truck.hive"')
        vec = vec[vec["type"] == "vector"]
        for run, rows in vec.groupby("run"):
            cov = rows[rows["name"] == "coverage:vector"]
            post = rows[rows["name"] == "dronesOnStation:vector"]
            if cov.empty:
                continue
            t = [float(x) for x in cov.iloc[0]["vectime"].split()]
            v = [float(x) * 100 for x in cov.iloc[0]["vecvalue"].split()]
            fig, ax = plt.subplots(figsize=(8, 4))
            ax.step([x / 60 for x in t], v, where="post", label="téléphones couverts (%)")
            ax.set_xlabel("temps (min)")
            ax.set_ylabel("couverture (%)")
            ax.set_ylim(0, 100)
            ax.grid(alpha=0.3)
            if not post.empty:
                ax2 = ax.twinx()
                t2 = [float(x) / 60 for x in post.iloc[0]["vectime"].split()]
                v2 = [float(x) for x in post.iloc[0]["vecvalue"].split()]
                ax2.step(t2, v2, where="post", color="tab:green", alpha=0.6, label="drones sur poste")
                ax2.set_ylabel("drones sur poste")
            config = attrs.loc[run, "configname"] if "configname" in attrs.columns else run
            ax.set_title("Couverture au cours du temps (%s)" % config)
            fig.tight_layout()
            fig.savefig(os.path.join(args.out, "coverage_vs_time_%s.png" % config), dpi=150)
            plt.close(fig)
    print("Figures written to", args.out)


if __name__ == "__main__":
    main()
