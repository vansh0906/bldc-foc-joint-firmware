"""Bus master: requests to single joints and the 1 kHz SYNC_CMD cycle for all of them."""
import time

from . import jbus


class JointBus:
    def __init__(self, port, ids):
        self.port = port
        self.ids = list(ids)
        self.parser = jbus.Parser()
        self.stats = dict(cycles=0, replies=0, missing=0)

    def _collect(self, want, timeout):
        got = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            data = self.port.read()
            if data:
                got += self.parser.feed(data)
                if len(got) >= want:
                    break
            else:
                time.sleep(0.0002)
        return got

    def request(self, frame, timeout=0.05):
        self.port.read()
        self.port.write(frame)
        r = self._collect(1, timeout)
        return r[0] if r else None

    def ping(self, nid):
        return self.request(jbus.frame(nid, jbus.PING))

    def action(self, nid, instr):
        return self.request(jbus.frame(nid, instr))

    def read(self, nid, name, count=1):
        r = self.request(jbus.read_regs(nid, jbus.REG[name], count))
        return None if r is None or r.refused else r.regs(jbus.REG[name])

    def write(self, nid, name, value):
        return self.request(jbus.write_reg(nid, name, value))

    def scan(self, ids=range(1, 17)):
        return [nid for nid in ids if self.ping(nid) is not None]

    def cycle(self, cmds, timeout=0.004):
        """cmds: {node_id: dict(mode=..., pos=..., vel=..., tau=..., kp=..., kd=...)}.
        Sends one SYNC_CMD; returns {node_id: status dict} for the joints that answered."""
        order = [(nid, cmds[nid]) for nid in self.ids if nid in cmds]
        self.port.write(jbus.sync_cmd(order))
        replies = self._collect(len(order), timeout)
        out = {r.node: r.status for r in replies if r.status is not None}
        self.stats["cycles"] += 1
        self.stats["replies"] += len(out)
        self.stats["missing"] += len(order) - len(out)
        return out
