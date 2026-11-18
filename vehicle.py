"""Simulated vehicle state: a looping drive cycle plus manual overrides and DTCs."""

import math
import time


def encode_dtc(code):
    """'P0301' -> (0x03, 0x01)."""
    code = code.strip().upper()
    kinds = {"P": 0, "C": 1, "B": 2, "U": 3}
    if len(code) != 5 or code[0] not in kinds:
        raise ValueError("bad DTC: %r (expected e.g. P0301)" % code)
    d = [int(c, 16) for c in code[1:]]
    if d[0] > 3:
        raise ValueError("bad DTC: %r (second char must be 0-3)" % code)
    return (kinds[code[0]] << 6) | (d[0] << 4) | d[1], (d[2] << 4) | d[3]


class ECU:
    """One control module. dtcs = [(3-byte code, UDS status byte), ...]."""

    def __init__(self, name, dtcs=(), obd=False):
        self.name = name
        self.dtcs = list(dtcs)
        self.obd = obd          # answers generic OBD-II modes 01-0A


def parse_address(text):
    """'7E0' / 'can:7E0' -> ('can', 0x7E0);  'ext:12' / 'bmw:12' -> ('ext', 0x12)."""
    kind, _, val = text.strip().lower().rpartition(":")
    kind = "ext" if kind in ("ext", "bmw") else "can"
    return kind, int(val, 16)


class Vehicle:
    CYCLE = 90.0  # seconds for one idle -> accelerate -> cruise -> brake loop

    def __init__(self, vin="1HGCM82633A004352", dtcs=(), pending=()):
        self.vin = vin.upper()[:17].ljust(17, "0")
        self.start = time.time()
        self.dtcs = list(dtcs)
        self.pending = list(pending)
        self.overrides = {}          # name -> fixed value
        self.discover = False        # answer at every probed 11-bit address (to map app module names)
        self.ext_format = None       # reply format for BMW extended addressing; None = learn it
        self.fuel = 72.0             # percent
        self.dist_mil = 0
        self.dist_since_clear = 1234
        self.time_since_clear = 0
        # Generic engine ECU: OBD-II requests to 7DF/7E0 land here, replies come from 7E8.
        self.ecus = {("can", 0x7E0): ECU("Engine (OBD-II)", obd=True)}

    def load_scenario(self, path):
        """Load a JSON scenario: VIN, OBD-II DTCs and per-module manufacturer DTCs."""
        import json
        with open(path) as f:
            sc = json.load(f)
        if sc.get("vin"):
            self.vin = sc["vin"].upper()[:17].ljust(17, "0")
        self.dtcs = [c.upper() for c in sc.get("obd_dtcs", [])]
        self.pending = [c.upper() for c in sc.get("obd_pending", [])]
        for m in sc.get("ecus", []):
            addrs = m.get("address") or []
            if isinstance(addrs, str):
                addrs = [addrs]
            dtcs = [(int(code, 16), int(str(status), 16)) for code, status in m.get("dtcs", [])]
            for a in addrs:
                key = parse_address(a)
                existing = self.ecus.get(key)
                if existing is not None and existing.obd:
                    existing.name, existing.dtcs = m["name"], list(dtcs)
                else:
                    self.ecus[key] = ECU(m["name"], dtcs)
        return sc

    # --- time-based drive cycle ---
    def _cycle(self):
        t = (time.time() - self.start) % self.CYCLE
        if t < 10:                    # idle
            speed = 0.0
        elif t < 30:                  # accelerate to ~100 km/h
            speed = 100.0 * (t - 10) / 20
        elif t < 65:                  # cruise with a little wobble
            speed = 100.0 + 6 * math.sin(t / 3)
        elif t < 80:                  # brake
            speed = 100.0 * (80 - t) / 15
        else:
            speed = 0.0
        accelerating = 10 <= t < 30
        return speed, accelerating

    def values(self):
        uptime = time.time() - self.start
        speed, accel = self._cycle()
        if speed < 1:
            rpm = 780 + 20 * math.sin(uptime)
        else:
            gear_ratios = [(20, 110), (40, 65), (60, 45), (85, 34), (999, 27)]
            ratio = next(r for top, r in gear_ratios if speed < top)
            rpm = max(900.0, speed * ratio + (600 if accel else 0))
        load = 22 + (45 if accel else 0) + speed * 0.15
        throttle = 14 + (40 if accel else 0) + speed * 0.1
        coolant = min(90.0, 20 + uptime / 4)          # warms up over ~5 min
        v = {
            "rpm": rpm,
            "speed": speed,
            "coolant": coolant,
            "intake": 25 + speed * 0.05,
            "oil": min(105.0, 20 + uptime / 3.5),
            "ambient": 22,
            "load": min(load, 100),
            "throttle": min(throttle, 100),
            "maf": rpm * load / 2200.0,
            "map": 30 + load * 0.7,
            "baro": 101,
            "timing": 10 + rpm / 400.0,
            "stft": 2 * math.sin(uptime / 5),
            "ltft": 3.1,
            "fuel": max(0.0, self.fuel - uptime / 600.0),
            "voltage": 14.1 if rpm > 300 else 12.6,
            "runtime": uptime,
        }
        v.update(self.overrides)
        return v

    def clear_dtcs(self):
        self.dtcs.clear()
        self.pending.clear()
        self.dist_mil = 0
        self.dist_since_clear = 0
        self.time_since_clear = 0

    @property
    def mil(self):
        return bool(self.dtcs)
