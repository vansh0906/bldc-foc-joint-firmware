"""A stand-in for rclpy and the message packages, so the bridge's logic can be tested without a
ROS 2 install. It records publishers, subscriptions, services and timers; no DDS is involved.
On a real ROS 2 machine, run the node itself (README, 'ROS 2' section)."""
import sys
import time
import types


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Header(_Msg):
    def __init__(self):
        super().__init__(stamp=None, frame_id="")


class JointState(_Msg):
    def __init__(self, **kw):
        super().__init__(header=Header(), name=[], position=[], velocity=[], effort=[])
        for k, v in kw.items():
            setattr(self, k, v)


class KeyValue(_Msg):
    pass


class DiagnosticStatus(_Msg):
    OK, WARN, ERROR, STALE = 0, 1, 2, 3

    def __init__(self, **kw):
        super().__init__(level=0, name="", message="", hardware_id="", values=[])
        for k, v in kw.items():
            setattr(self, k, v)


class DiagnosticArray(_Msg):
    def __init__(self):
        super().__init__(header=Header(), status=[])


class Trigger:
    class Request(_Msg):
        pass

    class Response(_Msg):
        def __init__(self):
            super().__init__(success=False, message="")


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)


class _Clock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: time.time())


class _Log:
    def info(self, *a):
        pass


class Node:
    overrides = {}

    def __init__(self, name):
        self.pubs, self.subs, self.srvs, self.timers = {}, {}, {}, []

    def declare_parameter(self, name, default):
        return _Param(Node.overrides.get(name, default))

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = _Pub(topic)
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_service(self, typ, name, cb):
        self.srvs[name] = cb

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def get_logger(self):
        return _Log()

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


def install():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda n: None
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node
    rclpy.node = node_mod
    sm, smm = types.ModuleType("sensor_msgs"), types.ModuleType("sensor_msgs.msg")
    smm.JointState = JointState
    dm, dmm = types.ModuleType("diagnostic_msgs"), types.ModuleType("diagnostic_msgs.msg")
    dmm.DiagnosticArray, dmm.DiagnosticStatus, dmm.KeyValue = DiagnosticArray, DiagnosticStatus, KeyValue
    ss, sss = types.ModuleType("std_srvs"), types.ModuleType("std_srvs.srv")
    sss.Trigger = Trigger
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node_mod, "sensor_msgs": sm, "sensor_msgs.msg": smm,
                        "diagnostic_msgs": dm, "diagnostic_msgs.msg": dmm, "std_srvs": ss, "std_srvs.srv": sss})
