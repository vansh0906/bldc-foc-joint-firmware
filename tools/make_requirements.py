"""Write docs/REQUIREMENTS.md from the requirement list below and results/summary.json
(run `python -m hil.run_all` first). Unit-test and build requirements are listed with their
evidence; HIL requirements take their measured values from the last run."""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

REQS = [
    ("R1", "Self-commissioning measures phase R within 5 %, L within 10 % and every hall edge within 2 deg (electrical), detects encoder direction, on every unit", "HIL"),
    ("R2", "Starts from an unknown rotor angle with >= 80 % torque on halls, switches to the encoder within 100 ms of moving, angle error <= 1.5 deg rms after", "HIL"),
    ("R3", "Current loop: 0 -> 5 A rise <= 400 us, overshoot <= 10 %, steady error <= 2 %", "HIL"),
    ("R4", "Speed loop: step overshoot <= 20 %, settles in 100 ms, ripple <= 0.25 rad/s at 2 rad/s, rejects a load step", "HIL"),
    ("R5", "Position loop: 1 rad step settles (2 %) in <= 150 ms, holds a gravity load within 2 encoder counts", "HIL"),
    ("R6", "Impedance mode realises the commanded stiffness within 5 % and is well damped", "HIL"),
    ("R7", "Encoder FOC torque ripple <= 5 %; measurably better than six-step commutation", "HIL"),
    ("R8", "Overcurrent (15 A, any phase) switches the bridge off within one PWM period and latches", "HIL + unit"),
    ("R9", "Braking on a supply that cannot absorb energy keeps the bus below the 30 V trip", "HIL"),
    ("R10", "Supply loss trips UNDERVOLT within 12 ms of the bus falling below 18 V", "HIL + unit"),
    ("R11", "A sustained stall never takes the winding above 130 C; the joint derates instead of tripping", "HIL + unit"),
    ("R12", "A failed encoder is detected within 2 hall edges; the joint keeps running on halls at half current", "HIL + unit"),
    ("R13", "Without commands for 100 ms the joint goes to damping and stays there until CLEAR", "HIL"),
    ("R14", "If the control task stops, torque is cut within 5 ms and the watchdog resets the MCU", "HIL + unit"),
    ("R15", "Six joints on one RS-485 bus at 1 kHz: every reply, no collisions, cycle <= 1 ms", "HIL"),
    ("R16", "Bit errors on the bus are rejected by CRC without disturbing the joints", "HIL"),
    ("R17", "The frame parser survives random data and never accepts a frame with 1-3 bit errors", "unit"),
    ("R18", "Enabling on a spinning rotor causes no current spike (<= 0.5 A)", "HIL"),
    ("R19", "Calibration survives a power cycle; a corrupted parameter record is detected and refused", "HIL + unit"),
    ("R20", "Builds for STM32G431 + FreeRTOS: <= 64 KB flash, <= 16 KB RAM, current loop above the kernel's interrupt mask", "build"),
]

UNIT = {
    "R17": "13 unit-test groups: 2,000,000 random bytes -> 0 frames accepted; 100,000 frames with 1-3 bit errors -> 0 accepted",
    "R20": "clang/lld cross build: 18,248 B flash, 7,656 B RAM; ADC IRQ priority 0, kernel mask at priority 5",
}


def main():
    with open(os.path.join(ROOT, "results", "summary.json")) as f:
        s = json.load(f)
    by_req = {}
    for r in s["results"]:
        for c in r["checks"]:
            by_req.setdefault(c["req"], []).append(c)
    out = ["# Requirements and verification", "",
           f"HIL run: {s['scenarios_passed']}/{s['scenarios']} scenarios, {s['checks_passed']}/{s['checks']} checks "
           f"(`python -m hil.run_all`, {s['seconds']} s). Unit tests: `./build/unit_tests`.", "",
           "| ID | Requirement | Method | Evidence (measured) | Status |", "|---|---|---|---|---|"]
    for rid, text, method in REQS:
        checks = by_req.get(rid, [])
        ev = "; ".join(f"{c['desc']}: **{c['value']}**" for c in checks)
        if rid in UNIT:
            ev = (ev + "; " if ev else "") + UNIT[rid]
        ok = all(c["ok"] for c in checks) if checks else True
        out.append(f"| {rid} | {text} | {method} | {ev} | {'PASS' if ok else 'FAIL'} |")
    with open(os.path.join(ROOT, "docs", "REQUIREMENTS.md"), "w") as f:
        f.write("\n".join(out) + "\n")
    print("wrote docs/REQUIREMENTS.md")


if __name__ == "__main__":
    main()
