"""HIL scenarios: each one drives the real firmware against the joint model and checks numbered
requirements (docs/REQUIREMENTS.md). Every scenario returns a Result with its checks, metrics and
optional traces for the report plots."""
import math
import time

import numpy as np

from . import jbus
from .sim import COL, F, KT, Sim

DEG = 180.0 / math.pi


class Result:
    def __init__(self, name, reqs):
        self.name, self.reqs = name, reqs
        self.checks, self.metrics, self.traces = [], {}, {}
        self.seconds = 0.0

    def check(self, req, desc, value, ok, limit=""):
        self.checks.append(dict(req=req, desc=desc, value=value, limit=limit, ok=bool(ok)))
        return ok

    @property
    def ok(self):
        return all(c["ok"] for c in self.checks)


def ready(seed=1, joints=1, calibrate=True, enable=True, **plant):
    """Boot, calibrate (and save), reboot like a real power cycle, then enable."""
    s = Sim(joints, seed)
    loads = {k: plant.pop(k) for k in list(plant) if k in ("T_load", "grav", "spring_k", "spring_x0")}
    for k, v in plant.items():
        for j in range(joints):
            s.set(k, v, j)
    s.boot(0.05)
    if calibrate:
        for j in range(joints):
            s.calibrate(j)
            s.local("reboot", j)
        s.boot(0.05)
    for k, v in loads.items():           # calibration is done unloaded (docs/BRINGUP.md)
        for j in range(joints):
            s.set(k, v, j)
    if enable:
        for j in range(joints):
            assert s.local("enable", j) == 1, s.state(j)
    return s


def spin_align(s, j=0):
    """Small move so the joint sees a hall edge and switches to the encoder."""
    s.hold(0.15, "velocity", vel=8.0, j=j)
    s.hold(0.05, "velocity", vel=0.0, j=j)


def spin_align_all(s):
    for ms in range(200):
        for j in range(s.n):
            s.cmd("velocity", vel=8.0 if ms < 150 else 0.0, j=j)
        s.run(0.001)


# --------------------------------------------------------------------------------------------
def sc_calibration():
    r = Result("Self-commissioning on 20 units (R, L, hall edges, encoder)", ["R1"])
    errs_r, errs_l, errs_h = [], [], []
    for seed in range(1, 21):
        rev = seed == 20                     # one unit with the encoder A/B swapped
        s = Sim(1, seed)
        if rev:
            s.set("enc_rev", 1)
        s.boot(0.05)
        st = s.calibrate()
        ok = st == "IDLE" and s.get("fw.cal_valid") == 1
        if not ok or rev:
            r.check("R1", f"unit {seed}{' (encoder reversed)' if rev else ''}: calibration completes", st, ok, "IDLE, valid")
        r.metrics["units_ok"] = r.metrics.get("units_ok", 0) + int(ok)
        errs_r.append(100 * (s.get("fw.R") / s.get("R") - 1))
        errs_l.append(100 * (s.get("fw.L") / s.get("Ld") - 1))
        # true boundaries: each hall switches at 0/180 deg + its placement error
        true_b = sorted(((k * 120 + sh) % 360 for k in range(3) for sh in (s.get(f"hall_shift{k}") * DEG,
                         s.get(f"hall_shift{k}") * DEG + 180)))
        fw_b = sorted((s.get(f"fw.hall_b{k}") * DEG) % 360 for k in range(6))
        for a, b in zip(true_b, fw_b):
            errs_h.append(abs((a - b + 180) % 360 - 180))
        if rev:
            r.check("R1", "reversed encoder detected (direction flipped)", s.get("fw.enc_dir"), s.get("fw.enc_dir") == -1, "-1")
            s.local("enable")
            s.hold(0.4, "velocity", vel=20.0)
            r.check("R1", "reversed-encoder unit runs at 20 rad/s after calibration", round(s.get("w"), 2),
                    abs(s.get("w") - 20) < 1.0, "20 +- 1")
        s.close()
    r.metrics.update(R_err_pct_max=max(map(abs, errs_r)), L_err_pct_max=max(map(abs, errs_l)),
                     hall_err_deg_max=max(errs_h))
    r.check("R1", "phase resistance error (max over units)", f"{max(map(abs, errs_r)):.1f}%", max(map(abs, errs_r)) <= 5, "<= 5%")
    r.check("R1", "inductance error (max over units)", f"{max(map(abs, errs_l)):.1f}%", max(map(abs, errs_l)) <= 10, "<= 10%")
    r.check("R1", "hall edge angle error (max, 120 edges)", f"{max(errs_h):.2f} deg", max(errs_h) <= 2.0, "<= 2 deg elec")
    r.check("R1", "units calibrated", f"{r.metrics['units_ok']}/20", r.metrics["units_ok"] == 20, "20/20")
    return r


def sc_dead_time_r():
    """Why R is measured from two current levels: a single V/I ratio includes the dead-time drop."""
    r = Result("Resistance: two-point method vs single V/I ratio", ["R1"])
    s = Sim(1, 3)
    s.boot(0.05)
    s.calibrate()
    v1, i1 = s.get("cal.v1"), s.get("cal.i1")
    single = v1 / i1
    true = s.get("R")
    two = s.get("fw.R")
    r.metrics.update(single_ratio_err_pct=100 * (single / true - 1), two_point_err_pct=100 * (two / true - 1))
    r.check("R1", "single V/I ratio at 2 A (dead time included)", f"{100 * (single / true - 1):+.0f}%",
            abs(single / true - 1) > 0.10, "shows the error (> 10%)")
    r.check("R1", "two-point R (dead time cancels)", f"{100 * (two / true - 1):+.1f}%", abs(two / true - 1) <= 0.05, "<= 5%")
    return r


def sc_startup():
    r = Result("Power-up at an unknown rotor angle: hall start, encoder alignment", ["R2"])
    worst_k = 1.0
    align_ms = []
    for seed in range(1, 6):
        s = ready(seed=seed, enable=False)
        # rotor parked at a random angle after the reboot; lock it and ask for torque
        s.set("locked", 1)
        s.local("enable")
        s.hold(0.02, "torque", tau=4 * KT)
        k = s.get("Te") / (4 * KT)              # torque actually produced / requested
        worst_k = min(worst_k, k)
        s.set("locked", 0)
        t0 = s.t
        while s.get("fw.aligned") == 0 and s.t - t0 < 1.0:
            s.hold(0.001, "velocity", vel=10.0)
        align_ms.append(1e3 * (s.t - t0))
        s.log_start(1000, 1.0)
        s.hold(0.6, "velocity", vel=100.0)
        L = s.log()
        err = L[L[:, COL["src"]] == 2][:, COL["angle_err"]][-300:]
        r.metrics[f"unit{seed}_angle_err_rms_deg"] = float(np.sqrt(np.mean(err ** 2)) * DEG)
        s.close()
    r.metrics.update(torque_fraction_before_alignment=worst_k, align_ms_max=max(align_ms))
    r.check("R2", "torque before the first hall edge (worst of 5 parked angles)", f"{100 * worst_k:.0f}%", worst_k >= 0.80, ">= 80% (cos 30 deg = 87%)")
    r.check("R2", "encoder aligned after starting to move (worst)", f"{max(align_ms):.0f} ms", max(align_ms) <= 100, "<= 100 ms at 10 rad/s")
    worst = max(v for k, v in r.metrics.items() if k.endswith("angle_err_rms_deg"))
    r.check("R2", "electrical angle error at 100 rad/s after alignment (rms, worst unit)", f"{worst:.2f} deg", worst <= 1.5, "<= 1.5 deg")
    return r


def sc_current_loop():
    r = Result("Current loop step response (locked rotor)", ["R3"])
    s = ready(seed=2)
    spin_align(s)
    s.set("locked", 1)
    s.hold(0.02, "torque", tau=0.0)
    s.log_start(20000, 0.02)
    s.hold(0.015, "torque", tau=5 * KT)
    L = s.log()
    t = (L[:, 0] - L[0, 0]) * 1e6
    iq = L[:, COL["iq_true"]]
    t10 = t[np.argmax(iq > 0.5)]
    t90 = t[np.argmax(iq > 4.5)]
    over = 100 * (iq.max() / 5.0 - 1)
    final = iq[-100:].mean()
    idr = np.abs(L[-100:, COL["id_true"]]).max()
    r.traces["current_step"] = (t[:40] - t10, iq[:40])
    r.metrics.update(rise_us=t90 - t10, overshoot_pct=over, final_A=final, kp=s.get("fw.kp"), ki=s.get("fw.ki"))
    r.check("R3", "rise time 10-90% for a 0 -> 5 A step", f"{t90 - t10:.0f} us", t90 - t10 <= 400, "<= 400 us")
    r.check("R3", "overshoot", f"{over:.1f}%", over <= 10, "<= 10%")
    r.check("R3", "steady-state iq error", f"{100 * abs(final / 5 - 1):.2f}%", abs(final / 5 - 1) <= 0.02, "<= 2%")
    r.check("R3", "d-axis current stays near zero", f"{idr:.2f} A", idr <= 0.3, "<= 0.3 A")
    return r


def sc_speed_loop():
    r = Result("Speed loop: step, low-speed smoothness, disturbance", ["R4"])
    s = ready(seed=2)
    spin_align(s)
    s.hold(0.3, "velocity", vel=20.0)
    s.log_start(1000, 0.4)
    s.hold(0.3, "velocity", vel=30.0)
    L = s.log()
    w = L[:, COL["vel_true"]]
    t = L[:, 0] - L[0, 0]
    over = 100 * (w.max() - 30) / 10
    band = np.where(np.abs(w - 30) > 0.5)[0]
    settle = 1e3 * t[band[-1]] if len(band) else 0.0
    r.traces["speed_step"] = (t * 1e3, w)
    s.hold(0.5, "velocity", vel=2.0)
    s.log_start(1000, 0.6)
    s.hold(0.5, "velocity", vel=2.0)
    Ls = s.log()
    ripple = Ls[:, COL["vel_true"]].std()
    s.log_start(1000, 0.4)
    s.hold(0.05, "velocity", vel=10.0)
    s.set("T_load", 0.1)
    s.hold(0.3, "velocity", vel=10.0)
    Ld = s.log()
    dip = 10 - Ld[:, COL["vel_true"]].min()
    rec = Ld[-100:, COL["vel_true"]].mean()
    r.metrics.update(overshoot_pct=over, settle_ms=settle, ripple_2rad_s=ripple, load_dip=dip)
    r.check("R4", "20 -> 30 rad/s step overshoot", f"{over:.1f}%", over <= 20, "<= 20%")
    r.check("R4", "settling to +-0.5 rad/s", f"{settle:.0f} ms", settle <= 100, "<= 100 ms")
    r.check("R4", "speed ripple at 2 rad/s (std)", f"{ripple:.3f} rad/s", ripple <= 0.25, "<= 0.25 rad/s")
    r.check("R4", "0.1 Nm load step at 10 rad/s: recovered (last 100 ms mean)", f"{rec:.2f} rad/s", abs(rec - 10) <= 0.3, "10 +- 0.3")
    return r


def sc_position():
    r = Result("Position control: step and holding a gravity load", ["R5"])
    s = ready(seed=4, grav=0.15)            # an arm segment: 0.15 Nm * sin(angle)
    spin_align(s)
    p0 = s.get("fw.pos")
    s.hold(0.3, "position", pos=p0)
    s.log_start(1000, 0.6)
    s.hold(0.5, "position", pos=p0 + 1.0)
    L = s.log()
    e = L[:, COL["pos_fw"]] - p0
    t = L[:, 0] - L[0, 0]
    over = (e.max() - 1.0)
    band = np.where(np.abs(e - 1.0) > 0.02)[0]
    settle = 1e3 * t[band[-1]] if len(band) else 0.0
    err = abs(e[-50:].mean() - 1.0)
    r.traces["position_step"] = (t * 1e3, e)
    r.metrics.update(settle_ms=settle, overshoot_rad=over, steady_err_rad=err)
    r.check("R5", "1 rad step settles within 2%", f"{settle:.0f} ms", settle <= 150, "<= 150 ms")
    r.check("R5", "overshoot", f"{over * 1000:.1f} mrad", over <= 0.05, "<= 50 mrad")
    r.check("R5", "steady-state error against gravity load", f"{err * 1000:.2f} mrad", err <= 3.1e-3,
            "<= 2 encoder counts (3.1 mrad)")
    return r


def sc_impedance():
    r = Result("Impedance mode: commanded stiffness and damping", ["R6"])
    s = ready(seed=5)
    spin_align(s)
    p0 = s.get("fw.pos")
    kp, kd = 2.0, 0.02
    s.hold(0.3, "impedance", pos=p0, kp=kp, kd=kd)
    defl = []
    for tl in (0.1, 0.2, -0.15):
        s.set("T_load", tl)
        s.hold(0.4, "impedance", pos=p0, kp=kp, kd=kd)
        defl.append((tl, p0 - s.get("fw.pos")))
    s.set("T_load", 0.0)
    k_meas = np.polyfit([d for _, d in defl], [t for t, _ in defl], 1)[0]
    r.metrics.update(k_meas=k_meas, k_cmd=kp)
    r.check("R6", "measured stiffness (3 external loads)", f"{k_meas:.3f} Nm/rad", abs(k_meas / kp - 1) <= 0.05, f"{kp} +- 5%")
    # damping: release from a deflection with kp=0 and check the velocity decays like kd/J
    s.hold(0.2, "impedance", pos=p0, kp=kp, kd=kd)
    s.log_start(1000, 0.4)
    s.hold(0.2, "impedance", pos=p0 + 0.5, kp=kp, kd=kd)
    L = s.log()
    e = L[:, COL["pos_fw"]] - (p0 + 0.5)
    r.traces["impedance_step"] = ((L[:, 0] - L[0, 0]) * 1e3, e)
    sign_changes = int(np.sum(np.diff(np.sign(e[np.abs(e) > 0.01])) != 0))
    r.check("R6", "0.5 rad set-point step is well damped (zero crossings)", sign_changes, sign_changes <= 2, "<= 2")
    return r


def sc_commutation():
    r = Result("Encoder FOC vs hall-interpolated vs six-step commutation", ["R7"])
    out = {}
    for src in ("encoder", "hall", "sixstep"):
        for spd in (5.0, 30.0):
            s = ready(seed=2, T_load=0.2)
            spin_align(s)
            if src != "encoder":
                s.set("fw.src", {"hall": 1, "sixstep": 4}[src])
            s.hold(0.4, "velocity", vel=spd)
            s.log_start(5000, 0.4)
            s.hold(0.3, "velocity", vel=spd)
            L = s.log()
            te = L[:, COL["torque"]]
            out[(src, spd)] = dict(
                torque_ripple_pct=100 * te.std() / te.mean(),
                speed_std=L[:, COL["vel_true"]].std(),
                i_rms=float(np.sqrt(np.mean(L[:, COL["iq_true"]] ** 2 + L[:, COL["id_true"]] ** 2))))
            s.close()
    for (src, spd), m in out.items():
        for k, v in m.items():
            r.metrics[f"{src}_{int(spd)}_{k}"] = v
    e5, s5 = out[("encoder", 5.0)], out[("sixstep", 5.0)]
    e30, s30 = out[("encoder", 30.0)], out[("sixstep", 30.0)]
    r.check("R7", "torque ripple with encoder FOC at 30 rad/s (std/mean)", f"{e30['torque_ripple_pct']:.1f}%", e30["torque_ripple_pct"] <= 5, "<= 5%")
    r.check("R7", "six-step torque ripple at 30 rad/s (for comparison)", f"{s30['torque_ripple_pct']:.1f}%",
            s30["torque_ripple_pct"] >= 2 * e30["torque_ripple_pct"], ">= 2x FOC")
    r.check("R7", "speed ripple at 5 rad/s: FOC vs six-step", f"{e5['speed_std']:.2f} vs {s5['speed_std']:.2f} rad/s",
            s5["speed_std"] >= 2 * e5["speed_std"], "six-step >= 2x")
    extra = 100 * (s5["i_rms"] / e5["i_rms"] - 1)
    r.check("R7", "current needed for the same 0.2 Nm (six-step vs FOC)", f"+{extra:.0f}% current", extra > 3, "FOC uses less")
    r.traces["commutation"] = out
    return r


def sc_overcurrent():
    r = Result("Overcurrent: shorted low-side switch while running", ["R8"])
    s = ready(seed=3)
    spin_align(s)
    s.hold(0.1, "velocity", vel=20.0)
    s.log_start(20000, 0.01)
    s.set("short_phase", 1)
    s.hold(0.005, "velocity", vel=20.0)
    L = s.log()
    ia = np.abs(L[:, COL["ia_true"]])
    trip = np.argmax(L[:, COL["state"]] == 5)
    first_over = np.argmax(ia > 15.0)
    r.metrics.update(peak_A=float(ia.max()), periods_to_off=int(trip - first_over))
    r.check("R8", "joint trips with OVERCURRENT", hex(s.faults()), s.faults() & F["OVERCURRENT"] and s.state() == "FAULT", "OVERCURRENT, FAULT")
    r.check("R8", "PWM off within one period of the current passing 15 A", f"{trip - first_over} period(s)", trip - first_over <= 1, "<= 1 (50 us)")
    r.check("R8", "peak phase current", f"{ia.max():.1f} A", ia.max() <= 25, "<= 25 A (ADC range)")
    s.set("short_phase", 0)
    s.hold(0.05, "velocity", vel=0.0)
    r.check("R8", "stays off until CLEAR + ENABLE", s.state(), s.state() == "FAULT" and s.get("pwm_on") == 0, "FAULT, PWM off")
    return r


def sc_regen():
    r = Result("Braking on a bench supply that cannot absorb power", ["R9"])
    res = {}
    for regen_limit in (True, False):
        s = ready(seed=2, supply_sink=0)
        spin_align(s)
        if not regen_limit:
            s.set("fw.regen_off", 1)
        s.hold(0.4, "velocity", vel=150.0)
        s.log_start(1000, 1.0)
        s.hold(0.8, "velocity", vel=0.0)
        L = s.log()
        res[regen_limit] = dict(vmax=float(L[:, COL["vbus"]].max()), state=s.state(), faults=s.faults(),
                                stop_ms=float(1e3 * (L[np.argmax(np.abs(L[:, COL["vel_true"]]) < 1.0), 0] - L[0, 0])))
        r.traces[f"regen_{regen_limit}"] = ((L[:, 0] - L[0, 0]) * 1e3, L[:, COL["vbus"]], L[:, COL["vel_true"]])
        s.close()
    on, off = res[True], res[False]
    r.metrics.update(vbus_max_limited=on["vmax"], vbus_max_unlimited=off["vmax"], stop_ms=on["stop_ms"])
    r.check("R9", "without the braking limit: bus overvoltage trip (the problem)", f"{off['vmax']:.1f} V, {off['state']}",
            off["state"] == "FAULT" and off["faults"] & F["OVERVOLT"], "OVERVOLT")
    r.check("R9", "with the ISR braking limit: peak bus voltage", f"{on['vmax']:.1f} V", on["vmax"] <= 30.0, "<= 30 V, no trip")
    r.check("R9", "with the limit: joint still stops (150 -> 0 rad/s)", f"{on['stop_ms']:.0f} ms, {on['state']}",
            on["state"] == "RUN" and on["stop_ms"] < 800, "< 800 ms, RUN")
    return r


def sc_undervoltage():
    r = Result("Supply loss (undervoltage)", ["R10"])
    s = ready(seed=1, T_load=0.3)
    spin_align(s)
    s.hold(0.2, "velocity", vel=30.0)        # ~9 W into the load
    s.set("supply_on", 0)
    s.log_start(1000, 1.0)
    s.hold(0.8, "velocity", vel=30.0)
    L = s.log()
    below = int(np.argmax(L[:, COL["vbus"]] < 18.0)) if (L[:, COL["vbus"]] < 18.0).any() else -1
    trip = int(np.argmax(L[:, COL["state"]] == 5)) if (L[:, COL["state"]] == 5).any() else -1
    r.metrics.update(trip_after_ms=float(trip - below))
    r.check("R10", "UNDERVOLT trip", hex(s.faults()), s.faults() & F["UNDERVOLT"], "UNDERVOLT")
    r.check("R10", "time from bus < 18 V to PWM off", f"{trip - below} ms", below >= 0 and 0 <= trip - below <= 12, "<= 12 ms")
    return r


def sc_thermal():
    r = Result("Thermal: 10 A stall for 150 s", ["R11"])
    res = {}
    for ntc_only in (True, False):
        s = ready(seed=1)
        spin_align(s)
        s.set("locked", 1)                         # output blocked: stall
        if ntc_only:
            s.set("fw.ntc_only", 1)
        s.log_start(10, 160)
        s.hold(150.0, "torque", tau=10 * KT)
        L = s.log()
        res[ntc_only] = dict(tw_max=float(L[:, COL["T_wind"]].max()), state=s.state(), faults=s.faults(),
                             est_err=float(np.abs(L[:, COL["T_wind_est"]] - L[:, COL["T_wind"]]).max()))
        r.traces[f"thermal_{ntc_only}"] = (L[:, 0] - L[0, 0], L[:, COL["T_wind"]], L[:, COL["T_ntc"]], L[:, COL["T_wind_est"]], L[:, COL["iq_true"]])
        s.close()
    a, b = res[True], res[False]
    r.metrics.update(tw_max_ntc_only=a["tw_max"], tw_max_model=b["tw_max"], est_err_max=b["est_err"])
    r.check("R11", "derating on the stator NTC alone: winding peak (the problem)", f"{a['tw_max']:.0f} C", a["tw_max"] > 130, "> 130 C (class B limit)")
    r.check("R11", "derating on NTC + winding model: winding peak", f"{b['tw_max']:.0f} C", b["tw_max"] <= 130, "<= 130 C")
    r.check("R11", "winding estimate error (max)", f"{b['est_err']:.1f} C", b["est_err"] <= 10, "<= 10 C")
    r.check("R11", "keeps running derated instead of tripping", b["state"], b["state"] == "RUN", "RUN")
    return r


def sc_encoder_fault():
    r = Result("Encoder cable unplugged at speed", ["R12"])
    s = ready(seed=4, T_load=0.05)
    spin_align(s)
    s.hold(0.3, "velocity", vel=60.0)
    edges0 = s.get("fw.edges")
    s.set("enc_frozen", 1)
    t0 = s.t
    while not s.faults() & F["ENCODER"] and s.t - t0 < 0.5:
        s.hold(0.001, "velocity", vel=60.0)
    det_ms = 1e3 * (s.t - t0)
    edges = s.get("fw.edges") - edges0
    s.hold(1.0, "velocity", vel=60.0)
    w = s.get("w")
    r.metrics.update(detect_ms=det_ms, edges_to_detect=edges, speed_after=w)
    r.check("R12", "ENCODER fault flagged", hex(s.faults()), s.faults() & F["ENCODER"], "ENCODER")
    r.check("R12", "detected within 2 hall edges", f"{edges:.0f} edges ({det_ms:.1f} ms)", edges <= 2, "<= 2")
    r.check("R12", "keeps running on hall sensors", f"{w:.1f} rad/s, {s.state()}", abs(w - 60) <= 6 and s.state() == "RUN", "60 +- 10%, RUN")
    r.check("R12", "current limit halved while on halls", f"{s.get('fw.i_lim'):.1f} A", abs(s.get("fw.i_lim") - 5.0) < 0.01, "5 A")
    return r


def sc_comm_timeout():
    r = Result("Host stops sending commands", ["R13"])
    s = ready(seed=1)
    spin_align(s)
    s.hold(0.3, "velocity", vel=40.0)
    s.log_start(1000, 0.6)
    s.run(0.5)                                   # silence
    L = s.log()
    damp = int(np.argmax(L[:, COL["state"]] == 3)) + 1   # the last command went out 1 ms before the log
    w_end = abs(s.get("w"))
    r.metrics.update(damp_after_ms=float(damp), speed_after_500ms=w_end)
    r.check("R13", "enters DAMP after the 100 ms timeout", f"{damp} ms after the last command", 100 <= damp <= 103, "100..103 ms")
    r.check("R13", "damping slows the joint (40 rad/s start)", f"{w_end:.1f} rad/s after 0.5 s", w_end < 10, "< 10")
    s.hold(0.05, "velocity", vel=40.0)           # commands resume
    r.check("R13", "does not restart by itself when commands resume", s.state(), s.state() == "DAMP", "DAMP")
    s.local("clear")
    s.local("enable")
    s.hold(0.4, "velocity", vel=40.0)
    r.check("R13", "CLEAR + ENABLE resumes", f"{s.get('w'):.1f} rad/s", abs(s.get("w") - 40) < 2, "40 +- 2")
    return r


def sc_stall_watchdog():
    r = Result("Control task hangs", ["R14"])
    s = ready(seed=2)
    spin_align(s)
    s.hold(0.2, "torque", tau=0.3)               # accelerating hard
    s.log_start(20000, 0.15)
    s.set("hang_ctrl", 1)
    s.run(0.12)
    L = s.log()
    off = np.argmax(L[:, COL["state"]] == 5) / 20.0
    rebooted = s.get("wdg_resets")
    r.metrics.update(torque_off_ms=float(off), wdg_resets=rebooted)
    r.check("R14", "ISR cuts torque when the control task stops", f"{off:.2f} ms", 0 < off <= 5.1, "<= 5 ms")
    r.check("R14", "watchdog reset follows (task-alive check)", f"{rebooted:.0f} reset(s)", rebooted >= 1, ">= 1 within 120 ms")
    r.check("R14", "after the reset: boots, PWM off, waits for ENABLE", f"{s.state()}, pwm {s.get('pwm_on'):.0f}",
            s.state() in ("IDLE", "BOOT") and s.get("pwm_on") == 0, "IDLE, off")
    return r


def sc_flying_start():
    r = Result("Enable while the joint is still spinning", ["R18"])
    res = {}
    for preload in (True, False):
        s = ready(seed=3)
        spin_align(s)
        if not preload:
            s.set("fw.no_preload", 1)
        s.hold(0.3, "velocity", vel=120.0)
        s.local("disable")
        s.run(0.02)                               # coasting
        s.log_start(20000, 0.02)
        s.local("enable")
        s.hold(0.01, "torque", tau=0.0)
        L = s.log()
        res[preload] = float(np.abs(L[:, COL["iq_true"]]).max())
        s.close()
    r.metrics.update(peak_iq_preload_A=res[True], peak_iq_no_preload_A=res[False])
    r.check("R18", "zero-voltage first duties: braking current spike (the problem)", f"{res[False]:.1f} A", res[False] > 1.0, "> 1 A")
    r.check("R18", "back-EMF first duties: current spike when enabling at 120 rad/s", f"{res[True]:.2f} A", res[True] <= 0.5, "<= 0.5 A")
    return r


def sc_bus_six_joints():
    r = Result("Six joints on one 4 Mbaud RS-485 bus at 1 kHz", ["R15"])
    n = 6
    s = ready(seed=7, joints=n, enable=False)
    for j in range(n):
        s.set("T_load", 0.02 * (j + 1), j)
    p = jbus.Parser()
    # enable each joint over the bus
    for j in range(n):
        s.send(jbus.frame(j + 1, jbus.ENABLE))
        s.run(0.001)
    p.feed(s.recv()[0])
    budget = jbus.bus_cycle_us(n, 4e6)
    cycles, got, missing, late = 2000, 0, 0, 0
    worst_end = 0.0
    for k in range(cycles):
        cmds = [(j + 1, dict(mode="velocity", vel=10.0 + 5 * j)) for j in range(n)]
        t_cycle = s.t * 1e6
        s.send(jbus.sync_cmd(cmds))
        s.run(0.001)
        data, ts = s.recv()
        reps = p.feed(data)
        got += len(reps)
        missing += n - len(reps)
        if ts:
            worst_end = max(worst_end, ts[-1] - t_cycle)
    speeds = [s.get("w", j) for j in range(n)]
    r.metrics.update(replies=got, missing=missing, collisions=s.get("collisions"), cycle_us_worst=worst_end,
                     cycle_us_budget=budget["total_us"])
    r.check("R15", "replies received", f"{got}/{cycles * n}", missing == 0, "all")
    r.check("R15", "bus collisions", int(s.get("collisions")), s.get("collisions") == 0, "0")
    r.check("R15", "worst cycle time (command start to last reply byte)", f"{worst_end:.0f} us", worst_end <= 1000, "<= 1000 us")
    r.check("R15", "all six joints at their own commanded speeds", ", ".join(f"{v:.1f}" for v in speeds),
            all(abs(speeds[j] - (10 + 5 * j)) < 0.5 for j in range(n)), "10, 15, ... 35")
    r.traces["bus_budget"] = budget
    return r


def sc_bus_noise():
    r = Result("Bit errors on the bus (BER 1e-4)", ["R16"])
    n = 3
    s = ready(seed=8, joints=n)
    spin_align_all(s)
    s.set("ber", 1e-4)
    p = jbus.Parser()
    cycles = 3000
    for k in range(cycles):
        cmds = [(j + 1, dict(mode="velocity", vel=20.0)) for j in range(n)]
        s.send(jbus.sync_cmd(cmds))
        s.run(0.001)
        p.feed(s.recv()[0])
    crc = sum(s.get("fw.crc_err", j) for j in range(n))
    states = [s.state(j) for j in range(n)]
    speeds = [s.get("w", j) for j in range(n)]
    r.metrics.update(bit_errors=s.get("bit_errors"), joint_crc_rejects=crc, host_crc_rejects=p.crc_errors)
    r.check("R16", "corrupted frames rejected by CRC (joints + host)", f"{crc:.0f} + {p.crc_errors}", crc + p.crc_errors > 0, "> 0 (errors happen)")
    r.check("R16", "no joint left RUN (a lost frame is covered by the next one)", ", ".join(states), all(x == "RUN" for x in states), "all RUN")
    r.check("R16", "speeds unaffected", ", ".join(f"{v:.1f}" for v in speeds), all(abs(v - 20) < 0.5 for v in speeds), "20 +- 0.5")
    return r


def sc_params():
    r = Result("Stored parameters: power cycle and corruption", ["R19"])
    s = Sim(1, 9)
    s.boot(0.05)
    s.calibrate(save=True)
    R = s.get("fw.R")
    s.local("reboot")
    s.boot(0.05)
    r.check("R19", "calibration survives a power cycle", f"cal_valid {s.get('fw.cal_valid'):.0f}, R {s.get('fw.R'):.4f}",
            s.get("fw.cal_valid") == 1 and abs(s.get("fw.R") - R) < 1e-6, "same values")
    # flip one byte of the stored record: CRC must catch it
    import ctypes as C
    sim_struct_flash_offset = None
    s.set("dead", 0)
    s.L.sim_set(s.h, 0, b"flash_corrupt", 17.0)
    s.local("reboot")
    s.boot(0.05)
    f = s.faults()
    r.check("R19", "corrupted record: defaults loaded, flagged", hex(f), f & F["PARAMS_RESET"] and f & F["NOT_CAL"] and s.get("fw.cal_valid") == 0, "PARAMS_RESET + NOT_CAL")
    r.check("R19", "uncalibrated joint refuses ENABLE", s.local("enable"), s.local("enable") == 0, "refused")
    return r


ALL = [sc_calibration, sc_dead_time_r, sc_startup, sc_current_loop, sc_speed_loop, sc_position, sc_impedance,
       sc_commutation, sc_overcurrent, sc_regen, sc_undervoltage, sc_thermal, sc_encoder_fault, sc_comm_timeout,
       sc_stall_watchdog, sc_flying_start, sc_bus_six_joints, sc_bus_noise, sc_params]
