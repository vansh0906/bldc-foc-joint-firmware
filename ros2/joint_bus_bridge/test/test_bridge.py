"""Bridge logic against two simulated joints behind a pty (./build/pty_sim), with a stand-in
for rclpy (test/fake_ros.py)."""
import os
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
PTY_SIM = os.path.join(REPO, "build", "pty_sim")
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import fake_ros  # noqa: E402

fake_ros.install()
from joint_bus_bridge import jbus  # noqa: E402
from joint_bus_bridge.node import JointBusBridge  # noqa: E402


def test_protocol_module_matches_the_hil_copy():
    with open(os.path.join(REPO, "hil", "jbus.py")) as a, open(os.path.join(HERE, "..", "joint_bus_bridge", "jbus.py")) as b:
        assert a.read() == b.read()


@pytest.fixture
def bus():
    if not os.path.exists(PTY_SIM):
        pytest.skip("build the simulator first")
    link = f"/tmp/jointbus_test_{os.getpid()}"
    proc = subprocess.Popen([PTY_SIM, "--joints", "2", "--link", link], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    proc.stdout.readline()                  # the path, printed once the joints are calibrated
    yield link
    proc.terminate()
    proc.wait(5)


def test_bridge_end_to_end(bus):
    fake_ros.Node.overrides = dict(port=bus, joint_ids=[1, 2], joint_names=["hip", "knee"], mode="impedance",
                                   kp=2.0, kd=0.02, rate_hz=500.0)
    node = JointBusBridge()
    resp = node.srvs["~/enable"](None, fake_ros.Trigger.Response())
    assert resp.success, resp.message
    cycle = [cb for period, cb in node.timers if period < 0.01][0]
    diag = [cb for period, cb in node.timers if period >= 0.1][0]
    for _ in range(50):
        cycle(); time.sleep(0.002)
    p0 = {n: node.pubs["/joint_states"].msgs[-1].position[k] for k, n in enumerate(["hip", "knee"])}
    cmd = fake_ros.JointState(name=["hip", "knee"], position=[p0["hip"] + 0.5, p0["knee"] - 0.3],
                              velocity=[0.0, 0.0], effort=[0.0, 0.0])
    node.subs["~/joint_commands"](cmd)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        cycle(); time.sleep(0.002)
    js = node.pubs["/joint_states"].msgs[-1]
    assert js.name == ["hip", "knee"]
    assert abs(js.position[0] - (p0["hip"] + 0.5)) < 0.02
    assert abs(js.position[1] - (p0["knee"] - 0.3)) < 0.02
    diag()
    d = node.pubs["/diagnostics"].msgs[-1]
    assert [s.level for s in d.status] == [0, 0], [s.message for s in d.status]
    assert node.bus.stats["missing"] <= node.bus.stats["cycles"] * 2 * 0.01
    resp = node.srvs["~/disable"](None, fake_ros.Trigger.Response())
    assert resp.success
    node.on_cycle()
    assert node.last[1]["state"] == "IDLE"
