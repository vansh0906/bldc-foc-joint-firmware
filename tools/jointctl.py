#!/usr/bin/env python3
"""Command-line tool for joints on the RS-485 bus (real adapter or ./build/pty_sim).

  python tools/jointctl.py --port /tmp/jointbus scan
  python tools/jointctl.py --port /tmp/jointbus calibrate 1
  python tools/jointctl.py --port /tmp/jointbus spin 1 --vel 20 --seconds 3
  python tools/jointctl.py --port /tmp/jointbus get 1 vel_kp
  python tools/jointctl.py --port /tmp/jointbus set 1 vel_kp 0.012"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2", "joint_bus_bridge"))
from joint_bus_bridge import jbus  # noqa: E402
from joint_bus_bridge.master import JointBus  # noqa: E402
from joint_bus_bridge.serial_port import SerialPort  # noqa: E402


def show(nid, st):
    if st is None:
        print(f"joint {nid}: no reply")
        return
    f = ",".join(st["fault_names"]) or "-"
    print(f"joint {nid}: {st['state']:<5} pos {st['pos']:+8.3f} rad  vel {st['vel']:+7.2f} rad/s  "
          f"iq {st['iq']:+6.2f} A  bus {st['vbus']:5.2f} V  winding {st['temp']:3d} C  faults {f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=4_000_000)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    for c in ("status", "enable", "disable", "clear", "calibrate", "save"):
        sub.add_parser(c).add_argument("id", type=int)
    g = sub.add_parser("get"); g.add_argument("id", type=int); g.add_argument("name", choices=list(jbus.REG))
    s = sub.add_parser("set"); s.add_argument("id", type=int); s.add_argument("name", choices=list(jbus.REG)); s.add_argument("value", type=float)
    sp = sub.add_parser("spin"); sp.add_argument("id", type=int); sp.add_argument("--vel", type=float, default=10.0)
    sp.add_argument("--seconds", type=float, default=3.0)
    ho = sub.add_parser("hold"); ho.add_argument("id", type=int); ho.add_argument("--kp", type=float, default=1.0)
    ho.add_argument("--kd", type=float, default=0.02); ho.add_argument("--seconds", type=float, default=3.0)
    a = ap.parse_args()

    if not os.path.exists(a.port):
        print(f"{a.port} does not exist. Start the joints first, in another window:\n"
              f"  ./build/pty_sim --joints 2 --link {a.port}\n"
              "(or plug in the USB-RS485 adapter and pass its /dev/... path)")
        return 1
    bus = JointBus(SerialPort(a.port, a.baud), [getattr(a, "id", 1)])
    if a.cmd == "scan":
        found = bus.scan()
        print("joints found:", found or "none")
        for nid in found:
            show(nid, bus.ping(nid).status)
    elif a.cmd == "status":
        r = bus.ping(a.id)
        show(a.id, r.status if r else None)
    elif a.cmd in ("enable", "disable", "clear", "calibrate", "save"):
        instr = dict(enable=jbus.ENABLE, disable=jbus.DISABLE, clear=jbus.CLEAR, calibrate=jbus.CALIBRATE, save=jbus.SAVE)[a.cmd]
        r = bus.action(a.id, instr)
        print("refused" if r is None or r.refused else "ok")
        if a.cmd == "calibrate" and r and not r.refused:
            while True:
                time.sleep(0.5)
                st = bus.ping(a.id).status
                print(f"  {st['state']}")
                if st["state"] != "CAL":
                    break
        if r:
            show(a.id, r.status)
    elif a.cmd == "get":
        print(a.name, "=", bus.read(a.id, a.name))
    elif a.cmd == "set":
        r = bus.write(a.id, a.name, a.value)
        print("refused" if r is None or r.refused else "ok")
    elif a.cmd in ("spin", "hold"):
        r = bus.action(a.id, jbus.ENABLE)
        if r is None or r.refused:
            print("enable refused:", r.status if r else "no reply")
            return 1
        p0 = r.status["pos"]
        cmd = dict(mode="velocity", vel=a.vel) if a.cmd == "spin" else dict(mode="impedance", pos=p0, kp=a.kp, kd=a.kd)
        t0 = time.monotonic()
        last = 0.0
        while time.monotonic() - t0 < a.seconds:
            st = bus.cycle({a.id: cmd}).get(a.id)
            if time.monotonic() - last > 0.2:
                show(a.id, st)
                last = time.monotonic()
            time.sleep(0.002)                  # ~500 Hz from Python; the joint's loops run on their own
        bus.action(a.id, jbus.DISABLE)
        print(f"cycles {bus.stats['cycles']}, replies {bus.stats['replies']}, missing {bus.stats['missing']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
