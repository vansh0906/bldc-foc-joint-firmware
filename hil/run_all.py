"""Run every HIL scenario, write results/summary.json, results/REPORT.md and the plots in docs/img.

    python -m hil.run_all            (after building: cmake -S . -B build && cmake --build build)"""
import json
import os
import sys
import time

import numpy as np

from . import scenarios

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results")
IMG = os.path.join(ROOT, "docs", "img")


def plots(results):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tr = {}
    for r in results:
        tr.update(r.traces)
    os.makedirs(IMG, exist_ok=True)
    plt.rcParams.update({"font.size": 9})

    fig, ax = plt.subplots(1, 3, figsize=(11, 3))
    t, i = tr["current_step"]
    ax[0].plot(t, i); ax[0].axhline(5, ls="--", c="gray"); ax[0].set(title="Current loop: 0 -> 5 A", xlabel="us", ylabel="iq (A)")
    t, w = tr["speed_step"]
    ax[1].plot(t, w); ax[1].axhline(30, ls="--", c="gray"); ax[1].set(title="Speed loop: 20 -> 30 rad/s", xlabel="ms", ylabel="rad/s")
    t, e = tr["position_step"]
    ax[2].plot(t, e); ax[2].axhline(1, ls="--", c="gray"); ax[2].set(title="Position: 1 rad step (gravity load)", xlabel="ms", ylabel="rad")
    fig.tight_layout(); fig.savefig(os.path.join(IMG, "step_responses.png"), dpi=130); plt.close(fig)

    c = tr["commutation"]
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    names = ["encoder", "hall", "sixstep"]
    labels = ["FOC (encoder)", "FOC (halls, interp.)", "six-step (halls)"]
    for k, spd in enumerate((5.0, 30.0)):
        ax[0].bar(np.arange(3) + 0.35 * k, [c[(n, spd)]["torque_ripple_pct"] for n in names], 0.35, label=f"{spd:.0f} rad/s")
        ax[1].bar(np.arange(3) + 0.35 * k, [c[(n, spd)]["speed_std"] for n in names], 0.35, label=f"{spd:.0f} rad/s")
    for a, tt in zip(ax, ("Torque ripple (std/mean, %)", "Speed ripple (std, rad/s)")):
        a.set_xticks(np.arange(3) + 0.17); a.set_xticklabels(labels, fontsize=7); a.set_title(tt); a.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(IMG, "commutation.png"), dpi=130); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(9, 3))
    for key, lab in (("regen_False", "no braking limit"), ("regen_True", "ISR braking limit")):
        t, v, w = tr[key]
        ax[0].plot(t, v, label=lab); ax[1].plot(t, w, label=lab)
    ax[0].axhline(30, ls="--", c="r", lw=0.8); ax[0].set(title="Bus voltage, 150 -> 0 rad/s on a bench supply", xlabel="ms", ylabel="V")
    ax[1].set(title="Speed", xlabel="ms", ylabel="rad/s"); ax[0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(IMG, "regen.png"), dpi=130); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(9, 3), sharey=True)
    for a, key, tt in ((ax[0], "thermal_True", "Derating on stator NTC only"), (ax[1], "thermal_False", "NTC + winding model")):
        t, tw, tn, te, iq = tr[key]
        a.plot(t, tw, label="winding (true)"); a.plot(t, tn, label="NTC"); a.plot(t, te, "--", label="winding estimate")
        a.axhline(130, ls=":", c="r", lw=0.8); a.set(title=tt, xlabel="s")
    ax[0].set_ylabel("deg C"); ax[0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(IMG, "thermal.png"), dpi=130); plt.close(fig)


def main():
    os.makedirs(RES, exist_ok=True)
    results = []
    t_all = time.time()
    for fn in scenarios.ALL:
        t0 = time.time()
        r = fn()
        r.seconds = time.time() - t0
        results.append(r)
        print(f"{'PASS' if r.ok else 'FAIL'}  {r.name}  ({r.seconds:.1f} s)")
        for c in r.checks:
            if not c["ok"]:
                print(f"      FAILED {c['req']}: {c['desc']}: {c['value']} (limit {c['limit']})")
    n_checks = sum(len(r.checks) for r in results)
    n_ok = sum(c["ok"] for r in results for c in r.checks)
    summary = dict(scenarios=len(results), scenarios_passed=sum(r.ok for r in results), checks=n_checks,
                   checks_passed=n_ok, seconds=round(time.time() - t_all, 1),
                   results=[dict(name=r.name, reqs=r.reqs, ok=r.ok, checks=r.checks,
                                 metrics={k: (float(v) if isinstance(v, (int, float, np.floating)) else v)
                                          for k, v in r.metrics.items()}) for r in results])
    with open(os.path.join(RES, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=str)
    lines = ["# HIL report", "", f"{summary['scenarios_passed']}/{summary['scenarios']} scenarios, "
             f"{n_ok}/{n_checks} checks passed in {summary['seconds']} s.", "",
             "| Req | Scenario | Check | Result | Limit | |", "|---|---|---|---|---|---|"]
    for r in results:
        for c in r.checks:
            lines.append(f"| {c['req']} | {r.name} | {c['desc']} | {c['value']} | {c['limit']} | {'PASS' if c['ok'] else 'FAIL'} |")
    with open(os.path.join(RES, "REPORT.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    try:
        plots(results)
    except Exception as e:      # plots are a convenience; the checks are what count
        print("plots skipped:", e)
    print(f"\n{summary['scenarios_passed']}/{summary['scenarios']} scenarios, {n_ok}/{n_checks} checks passed")
    return 0 if n_ok == n_checks else 1


if __name__ == "__main__":
    sys.exit(main())
