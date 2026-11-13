"""Host side of the joint bus protocol (pure Python, no dependencies).

Frame: AA 55 | id | len | instr | data (len-1 bytes) | CRC-16/MODBUS lo hi  (CRC over id..data)
Shared by the HIL tests, the command-line tool (tools/jointctl.py) and the ROS 2 bridge."""
import struct

BROADCAST = 0xFE
PING, READ, WRITE, SYNC_CMD, CMD = 0x01, 0x02, 0x03, 0x10, 0x11
ENABLE, DISABLE, CLEAR, CALIBRATE, SAVE = 0x20, 0x21, 0x22, 0x23, 0x24
REPLY, REFUSED = 0x80, 0x40
MODES = {"torque": 0, "velocity": 1, "position": 2, "impedance": 3}
STATES = ["BOOT", "IDLE", "RUN", "DAMP", "CAL", "FAULT"]
FAULT_BITS = {0x1: "OVERCURRENT", 0x2: "OVERVOLT", 0x4: "UNDERVOLT", 0x8: "OVERTEMP", 0x10: "CTRL_STALL",
              0x20: "CAL_FAILED", 0x40: "ADC_OFFSET", 0x100: "ENCODER", 0x200: "COMM_TIMEOUT",
              0x400: "DERATING", 0x800: "REGEN_LIMIT", 0x1000: "NOT_CAL", 0x2000: "PARAMS_RESET"}
REG = {"model": 0, "node_id": 1, "R": 2, "L": 3, "cur_bw_hz": 4, "vel_kp": 5, "vel_ki": 6, "pos_kp": 7,
       "i_limit": 8, "vel_limit": 9, "kd_safe": 10, "comm_timeout_ms": 11, "baud": 12, "cal_valid": 13,
       "hall_b0": 14, "angle_src_override": 20, "frames_ok": 21, "crc_err": 22, "t_wind_est": 23,
       "t_ntc": 24, "angle_src": 25}
REG_INT = {"model", "node_id", "comm_timeout_ms", "baud", "cal_valid", "angle_src_override", "frames_ok",
           "crc_err", "angle_src"}
STATUS_LEN = 17
REPLY_BYTES = 2 + 1 + 1 + 1 + STATUS_LEN + 2
ENTRY_LEN = 16


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(node: int, instr: int, data: bytes = b"") -> bytes:
    body = bytes([node, len(data) + 1, instr]) + data
    return b"\xAA\x55" + body + struct.pack("<H", crc16_modbus(body))


def entry_payload(mode="torque", pos=0.0, vel=0.0, tau=0.0, kp=0.0, kd=0.0) -> bytes:
    m = MODES[mode] if isinstance(mode, str) else int(mode)
    return struct.pack("<BffhHH", m, pos, vel, int(round(max(-32.767, min(32.767, tau)) * 1000)),
                       int(round(max(0, min(655.35, kp)) * 100)), int(round(max(0, min(6.5535, kd)) * 1e4)))


def sync_cmd(cmds) -> bytes:
    """cmds: list of (node_id, dict(mode=..., pos=..., ...)); reply slot = list position."""
    data = bytes([len(cmds)]) + b"".join(bytes([nid]) + entry_payload(**c) for nid, c in cmds)
    return frame(BROADCAST, SYNC_CMD, data)


def cmd(node, **c) -> bytes:
    return frame(node, CMD, entry_payload(**c))


def read_regs(node, first, count=1) -> bytes:
    return frame(node, READ, bytes([first, count]))


def write_reg(node, name, value) -> bytes:
    r = REG[name]
    raw = struct.pack("<I", int(value)) if name in REG_INT else struct.pack("<f", float(value))
    return frame(node, WRITE, bytes([r, 1]) + raw)


def bus_cycle_us(n_joints: int, baud: float, turnaround_us=30, guard_us=4) -> dict:
    """Time budget of one SYNC_CMD cycle: command frame + n reply slots."""
    byte_us = 10e6 / baud
    cmd_bytes = 7 + 1 + n_joints * ENTRY_LEN
    slot = int(REPLY_BYTES * 10e6 // baud) + 1 + guard_us
    total = cmd_bytes * byte_us + turnaround_us + (n_joints - 1) * slot + REPLY_BYTES * byte_us
    return {"cmd_us": cmd_bytes * byte_us, "slot_us": slot, "total_us": total}


class Parser:
    """Incremental frame parser (resyncs on the header; drops frames with bad length or CRC)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                self.buf = self.buf[-1:] if self.buf[-1:] == b"\xAA" else bytearray()
                return out
            del self.buf[:i]
            if len(self.buf) < 4:
                return out
            n = self.buf[3]
            total = 4 + n + 2
            if n < 1 or n > 122:
                del self.buf[:2]
                continue
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:4 + n])
            crc = struct.unpack("<H", self.buf[4 + n:total])[0]
            if crc != crc16_modbus(body):
                self.crc_errors += 1
                del self.buf[:2]
                continue
            del self.buf[:total]
            out.append(Reply(body[0], body[2], body[3:]))


class Reply:
    def __init__(self, node, instr, data):
        self.node, self.instr, self.data = node, instr, data
        self.refused = bool(instr & REFUSED) and instr & REPLY
        self.status = None
        if len(data) == STATUS_LEN:
            st, mode, faults, pos, vel, iq, vbus, temp = struct.unpack("<BBHffhHb", data)
            self.status = dict(state=STATES[st] if st < len(STATES) else st, mode=mode, faults=faults,
                               fault_names=[v for k, v in FAULT_BITS.items() if faults & k],
                               pos=pos, vel=vel, iq=iq / 1000.0, vbus=vbus / 100.0, temp=temp)

    def regs(self, names_from=0, types=None):
        n = len(self.data) // 4
        vals = []
        for k in range(n):
            raw = self.data[4 * k:4 * k + 4]
            name = next((nm for nm, r in REG.items() if r == names_from + k), None)
            vals.append(struct.unpack("<I" if name in REG_INT else "<f", raw)[0])
        return vals

    def __repr__(self):
        return f"Reply(node={self.node}, instr=0x{self.instr:02x}, status={self.status})"
