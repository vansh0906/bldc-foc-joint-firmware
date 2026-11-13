"""ctypes wrapper around build/libjointsim.so (the firmware running against the joint model)."""
import ctypes as C
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB_PATHS = [os.path.join(ROOT, "build", n) for n in ("libjointsim.so", "libjointsim.dylib")]

LOG_COLS = 20
COL = {n: i for i, n in enumerate(
    "t pos_true vel_true pos_fw vel_fw iq_ref iq_fw iq_true id_true torque vbus T_wind T_ntc "
    "T_wind_est state faults angle_err src id_fw ia_true".split())}

ST = {"BOOT": 0, "IDLE": 1, "RUN": 2, "DAMP": 3, "CAL": 4, "FAULT": 5}
ST_NAME = {v: k for k, v in ST.items()}
MODE = {"torque": 0, "velocity": 1, "position": 2, "impedance": 3}
SRC = {"none": 0, "hall": 1, "encoder": 2, "forced": 3, "sixstep": 4}
F = dict(OVERCURRENT=0x1, OVERVOLT=0x2, UNDERVOLT=0x4, OVERTEMP=0x8, CTRL_STALL=0x10, CAL_FAILED=0x20,
         ADC_OFFSET=0x40, ENCODER=0x100, COMM_TIMEOUT=0x200, DERATING=0x400, REGEN_LIMIT=0x800,
         NOT_CAL=0x1000, PARAMS_RESET=0x2000)
PWM_HZ = 20000.0
KT = 1.5 * 7 * 0.0085


def _load():
    for p in LIB_PATHS:
        if os.path.exists(p):
            lib = C.CDLL(p)
            break
    else:
        raise FileNotFoundError("build the simulator first: cmake -S . -B build && cmake --build build")
    lib.sim_new.restype = C.c_void_p
    lib.sim_new.argtypes = [C.c_int, C.c_uint64]
    lib.sim_free.argtypes = [C.c_void_p]
    lib.sim_run.argtypes = [C.c_void_p, C.c_double]
    lib.sim_master_send.argtypes = [C.c_void_p, C.c_char_p, C.c_int, C.c_double]
    lib.sim_master_recv.argtypes = [C.c_void_p, C.c_char_p, C.POINTER(C.c_double), C.c_int]
    lib.sim_master_recv.restype = C.c_int
    lib.sim_local_cmd.argtypes = [C.c_void_p, C.c_int, C.c_int] + [C.c_double] * 5
    lib.sim_local.argtypes = [C.c_void_p, C.c_int, C.c_char_p]
    lib.sim_local.restype = C.c_int
    lib.sim_log_config.argtypes = [C.c_void_p, C.c_int, C.c_double, C.c_int]
    lib.sim_log_get.argtypes = [C.c_void_p, C.c_int, C.POINTER(C.c_double), C.c_int]
    lib.sim_log_get.restype = C.c_int
    lib.sim_time.argtypes = [C.c_void_p]
    lib.sim_time.restype = C.c_double
    lib.sim_set.argtypes = [C.c_void_p, C.c_int, C.c_char_p, C.c_double]
    lib.sim_set.restype = C.c_int
    lib.sim_get.argtypes = [C.c_void_p, C.c_int, C.c_char_p]
    lib.sim_get.restype = C.c_double
    return lib


_LIB = None


def lib():
    global _LIB
    if _LIB is None:
        _LIB = _load()
    return _LIB


class Sim:
    """N joints on one RS-485 bus. Joint index j (0-based) has node id j+1."""

    def __init__(self, joints=1, seed=1):
        self.L = lib()
        self.h = self.L.sim_new(joints, seed)
        self.n = joints
        self._log_rows = [0] * joints

    def close(self):
        if self.h:
            self.L.sim_free(self.h)
            self.h = None

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    @property
    def t(self):
        return self.L.sim_time(self.h)

    def run(self, seconds):
        self.L.sim_run(self.h, seconds)

    def set(self, key, value, j=0):
        if self.L.sim_set(self.h, j, key.encode(), float(value)) != 0:
            raise KeyError(key)

    def get(self, key, j=0):
        v = self.L.sim_get(self.h, j, key.encode())
        if v != v:
            raise KeyError(key)
        return v

    def state(self, j=0):
        return ST_NAME[int(self.get("fw.state", j))]

    def faults(self, j=0):
        return int(self.get("fw.faults", j))

    def local(self, what, j=0):
        return self.L.sim_local(self.h, j, what.encode())

    def cmd(self, mode="torque", pos=0.0, vel=0.0, tau=0.0, kp=0.0, kd=0.0, j=0):
        """Test shortcut: write a command straight into the firmware (as the bus would)."""
        self.L.sim_local_cmd(self.h, j, MODE[mode], pos, vel, tau, kp, kd)

    def hold(self, seconds, mode="torque", j=0, **kw):
        """Keep re-sending the same command every ms (keeps the comm watchdog fed)."""
        steps = int(round(seconds * 1000))
        for _ in range(steps):
            self.cmd(mode, j=j, **kw)
            self.run(0.001)

    def log_start(self, rate_hz=1000.0, seconds=10.0, j=0):
        rows = int(rate_hz * seconds) + 10
        self._log_rows[j] = rows
        self.L.sim_log_config(self.h, j, rate_hz, rows)

    def log(self, j=0):
        rows = self._log_rows[j]
        buf = (C.c_double * (rows * LOG_COLS))()
        n = self.L.sim_log_get(self.h, j, buf, rows)
        return np.ctypeslib.as_array(buf)[: n * LOG_COLS].reshape(n, LOG_COLS).copy()

    # ---- host port ----
    def send(self, data: bytes, delay_us=0.0):
        self.L.sim_master_send(self.h, data, len(data), delay_us)

    def recv(self):
        buf = C.create_string_buffer(65536)
        ts = (C.c_double * 65536)()
        n = self.L.sim_master_recv(self.h, buf, ts, 65536)
        return buf.raw[:n], [ts[i] for i in range(n)]

    def boot(self, seconds=0.05):
        self.run(seconds)

    def calibrate(self, j=0, save=True):
        assert self.local("cal", j) == 1, "calibration refused"
        for _ in range(200):
            self.run(0.05)
            if self.state(j) != "CAL":
                break
        if save:
            self.local("save", j)
        return self.state(j)
