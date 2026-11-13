"""ROS 2 node: joints on the RS-485 bus <-> ROS topics (DDS).

  sub  ~/joint_commands  sensor_msgs/JointState   name[], position[], velocity[], effort[] (feed-forward Nm)
  pub  /joint_states     sensor_msgs/JointState   position, velocity, effort (= iq * Kt)
  pub  /diagnostics      diagnostic_msgs/DiagnosticArray   state, faults, bus voltage, winding temperature
  srv  ~/enable  ~/disable  ~/clear_faults   std_srvs/Trigger

Parameters: port, baud, joint_ids, joint_names, mode (torque|velocity|position|impedance), kp, kd,
rate_hz (bus cycle rate; the joints run their own 20 kHz / 1 kHz loops either way)."""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from . import jbus
from .master import JointBus
from .serial_port import SerialPort

KT = 1.5 * 7 * 0.0085


class JointBusBridge(Node):
    def __init__(self):
        super().__init__("joint_bus_bridge")
        p = self.declare_parameter
        self.port_name = p("port", "/tmp/jointbus").value
        baud = p("baud", 4_000_000).value
        self.ids = list(p("joint_ids", [1]).value)
        self.names = list(p("joint_names", [f"joint{i}" for i in self.ids]).value)
        self.mode = p("mode", "position").value
        self.kp = p("kp", 2.0).value
        self.kd = p("kd", 0.02).value
        rate = p("rate_hz", 500.0).value
        self.bus = JointBus(SerialPort(self.port_name, baud), self.ids)
        self.cmd = {nid: dict(mode="torque") for nid in self.ids}       # zero torque until commanded
        self.last = {}
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(JointState, "~/joint_commands", self.on_command, 10)
        self.create_service(Trigger, "~/enable", lambda rq, rs: self.on_action(jbus.ENABLE, rs))
        self.create_service(Trigger, "~/disable", lambda rq, rs: self.on_action(jbus.DISABLE, rs))
        self.create_service(Trigger, "~/clear_faults", lambda rq, rs: self.on_action(jbus.CLEAR, rs))
        self.create_timer(1.0 / rate, self.on_cycle)
        self.create_timer(0.1, self.on_diag)
        self.get_logger().info(f"joint bus on {self.port_name}: ids {self.ids}, mode {self.mode}")

    def on_command(self, msg):
        for k, name in enumerate(msg.name):
            if name not in self.names:
                continue
            nid = self.ids[self.names.index(name)]
            pos = msg.position[k] if k < len(msg.position) else 0.0
            vel = msg.velocity[k] if k < len(msg.velocity) else 0.0
            eff = msg.effort[k] if k < len(msg.effort) else 0.0
            self.cmd[nid] = dict(mode=self.mode, pos=pos, vel=vel, tau=eff, kp=self.kp, kd=self.kd)

    def on_cycle(self):
        st = self.bus.cycle(self.cmd)
        self.last.update(st)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for nid, name in zip(self.ids, self.names):
            s = st.get(nid)
            if s is None:
                continue
            js.name.append(name)
            js.position.append(s["pos"])
            js.velocity.append(s["vel"])
            js.effort.append(s["iq"] * KT)
        if js.name:
            self.pub_js.publish(js)

    def on_diag(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for nid, name in zip(self.ids, self.names):
            s = self.last.get(nid)
            d = DiagnosticStatus(name=f"joint_bus/{name}", hardware_id=f"node {nid}")
            if s is None:
                d.level, d.message = DiagnosticStatus.STALE, "no reply"
            else:
                latched = s["faults"] & 0xFF
                d.level = DiagnosticStatus.ERROR if latched or s["state"] == "FAULT" else (
                    DiagnosticStatus.WARN if s["faults"] & 0x0F00 else DiagnosticStatus.OK)
                d.message = f"{s['state']} {' '.join(s['fault_names'])}".strip()
                d.values = [KeyValue(key="bus_voltage", value=f"{s['vbus']:.2f}"),
                            KeyValue(key="winding_temp_est", value=str(s["temp"]))]
            arr.status.append(d)
        self.pub_diag.publish(arr)

    def on_action(self, instr, resp):
        bad = []
        for nid in self.ids:
            r = self.bus.action(nid, instr)
            if r is None or r.refused:
                bad.append(nid)
            if instr == jbus.ENABLE and r is not None and r.status:
                # hold where it is: position commands start from the current position
                self.cmd[nid] = dict(mode=self.mode, pos=r.status["pos"], kp=self.kp, kd=self.kd)
        resp.success = not bad
        resp.message = "ok" if not bad else f"refused by {bad}"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = JointBusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
