"""Host protocol module: CRC, framing, parser resync, and a round trip through the simulator."""
import struct

from hil import jbus
from hil.sim import Sim


def test_crc_check_value():
    assert jbus.crc16_modbus(b"123456789") == 0x4B37


def test_parser_resync_and_crc():
    good = jbus.frame(3, jbus.PING | jbus.REPLY, bytes(17))
    bad = bytearray(good)
    bad[6] ^= 0x01
    p = jbus.Parser()
    out = p.feed(b"\x00\x13\xAA" + bytes(bad) + b"\x55\xAA" + good[:5])
    out += p.feed(good[5:])
    assert [r.node for r in out] == [3]
    assert p.crc_errors == 1


def test_bus_budget_six_joints():
    b = jbus.bus_cycle_us(6, 4e6)
    assert b["total_us"] < 1000


def test_read_write_registers_over_the_bus():
    s = Sim(1, 5)
    s.boot(0.05)
    p = jbus.Parser()
    s.send(jbus.write_reg(1, "vel_kp", 0.012))
    s.run(0.001)
    s.send(jbus.read_regs(1, jbus.REG["vel_kp"], 3))
    s.run(0.001)
    replies = p.feed(s.recv()[0])
    assert replies[0].status["state"] == "IDLE"
    kp, ki, pos_kp = struct.unpack("<fff", replies[1].data)
    assert abs(kp - 0.012) < 1e-7 and ki > 0 and pos_kp > 0
    # an out-of-range value is refused and not applied
    s.send(jbus.write_reg(1, "vel_kp", 7.0))
    s.run(0.001)
    r = p.feed(s.recv()[0])[0]
    assert r.refused
    s.close()
