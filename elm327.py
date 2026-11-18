"""ELM327 command interpreter that answers like an adapter plugged into a CAN (ISO 15765-4, 11-bit/500k) car."""

import logging

from vehicle import ECU, encode_dtc

log = logging.getLogger("elm327")

ECU_TX = 0x7E8  # engine ECU response ID


def u8(x):
    return max(0, min(255, int(round(x))))


def u16(x):
    x = max(0, min(65535, int(round(x))))
    return [x >> 8, x & 0xFF]


# Mode 01 PID encoders: pid -> f(values, vehicle) -> list of data bytes
def _status(v, car):
    return [(0x80 if car.mil else 0) | min(len(car.dtcs), 127), 0x07, 0xE5, 0x00]


PIDS = {
    0x01: _status,
    0x03: lambda v, c: [0x02, 0x00],                       # fuel system: closed loop
    0x04: lambda v, c: [u8(v["load"] * 255 / 100)],
    0x05: lambda v, c: [u8(v["coolant"] + 40)],
    0x06: lambda v, c: [u8(v["stft"] * 128 / 100 + 128)],
    0x07: lambda v, c: [u8(v["ltft"] * 128 / 100 + 128)],
    0x0B: lambda v, c: [u8(v["map"])],
    0x0C: lambda v, c: u16(v["rpm"] * 4),
    0x0D: lambda v, c: [u8(v["speed"])],
    0x0E: lambda v, c: [u8((v["timing"] + 64) * 2)],
    0x0F: lambda v, c: [u8(v["intake"] + 40)],
    0x10: lambda v, c: u16(v["maf"] * 100),
    0x11: lambda v, c: [u8(v["throttle"] * 255 / 100)],
    0x1C: lambda v, c: [0x06],                             # OBD standard: EOBD
    0x1F: lambda v, c: u16(v["runtime"]),
    0x21: lambda v, c: u16(c.dist_mil),
    0x2F: lambda v, c: [u8(v["fuel"] * 255 / 100)],
    0x31: lambda v, c: u16(c.dist_since_clear),
    0x33: lambda v, c: [u8(v["baro"])],
    0x42: lambda v, c: u16(v["voltage"] * 1000),
    0x46: lambda v, c: [u8(v["ambient"] + 40)],
    0x4E: lambda v, c: u16(c.time_since_clear),
    0x51: lambda v, c: [0x01],                             # fuel type: gasoline
    0x5C: lambda v, c: [u8(v["oil"] + 40)],
}

MODE09_SUPPORTED = (0x02, 0x04, 0x0A)

# How an adapter may print replies from a module using CAN extended addressing (BMW).
# Not every adapter does it the same way, so until the app accepts one, the four
# presence pings (3E) to a module are each answered in a different format.
EXT_FORMATS = [
    ("tester byte shown, padded", True, True),
    ("tester byte stripped, padded", False, True),
    ("tester byte shown, unpadded", True, False),
    ("tester byte stripped, unpadded", False, False),
]


def supported_bitmap(pids, base):
    """Bitmap for PID base (0x00, 0x20, ...) — bit for base+1 .. base+0x20."""
    bits = 0
    for p in pids:
        if base < p <= base + 0x20:
            bits |= 1 << (32 - (p - base))
    if any(p > base + 0x20 for p in pids):
        bits |= 1  # next range supported
    return [(bits >> s) & 0xFF for s in (24, 16, 8, 0)]


class ELM327:
    VERSION = "ELM327 v2.2"   # apps pick features by this; v2.x supports CAN extended addressing

    def __init__(self, vehicle):
        self.car = vehicle
        self.last = ""
        self.reset()

    def reset(self):
        self.echo = True
        self.linefeed = False
        self.spaces = True
        self.headers = False
        self.auto_protocol = True
        self.searched = False
        self.header = 0x7DF  # functional broadcast; ATSH changes the target ECU
        self.cea = None      # CAN extended address (BMW: ATSH6F1 + ATCEA<ecu>)
        self.ext_tries = {}  # ext address -> presence pings answered so far
        self.ext_last = {}   # ext address -> format used for the last ping

    # ---------- formatting ----------
    @property
    def eol(self):
        return "\r\n" if self.linefeed else "\r"

    def _hex(self, data):
        sep = " " if self.spaces else ""
        return sep.join("%02X" % b for b in data)

    @staticmethod
    def _pad(frame):
        """Real ECUs send full 8-byte CAN frames; with headers on the ELM327 shows the padding."""
        return frame + [0x00] * (8 - len(frame))

    def _frames(self, payload, rx_id=ECU_TX, ext=None, ext_fmt=0):
        """Format an ISO-TP message the way an ELM327 prints it."""
        sep = " " if self.spaces else ""
        hdr = ("%03X" % rx_id + sep) if self.headers else ""
        if ext is not None:
            show_addr, pad = EXT_FORMATS[ext_fmt][1:]
            # Extended addressing: every CAN frame starts with the address byte, then the PCI.
            if len(payload) <= 6:
                frames = [[len(payload)] + payload]
            else:
                n = len(payload)
                frames = [[0x10 | (n >> 8), n & 0xFF] + payload[:5]]
                rest, i = payload[5:], 1
                while rest:
                    frames.append([0x20 | (i & 0x0F)] + rest[:6])
                    rest, i = rest[6:], i + 1
            out = []
            for f in frames:
                f = ([ext] if show_addr else []) + f
                if pad:
                    f = f + [0x00] * ((8 if show_addr else 7) - len(f))
                out.append(hdr + self._hex(f))
            return out
        if len(payload) <= 7:
            if self.headers:
                return [hdr + self._hex(self._pad([len(payload)] + payload))]
            return [self._hex(payload)]
        # multi-frame: first frame + consecutive frames
        n = len(payload)
        chunks = [payload[:6]]
        rest = payload[6:]
        while rest:
            chunks.append(rest[:7])
            rest = rest[7:]
        lines = []
        if self.headers:
            lines.append(hdr + self._hex([0x10 | (n >> 8), n & 0xFF] + chunks[0]))
            for i, ch in enumerate(chunks[1:], 1):
                lines.append(hdr + self._hex(self._pad([0x20 | (i & 0x0F)] + ch)))
        else:
            lines.append("%03X" % n)
            for i, ch in enumerate(chunks):
                lines.append("%X:%s%s" % (i & 0x0F, sep, self._hex(ch)))
        return lines

    # ---------- entry point ----------
    def handle(self, raw):
        """Take one command line (without CR); return the full reply including the '>' prompt."""
        cmd = raw.strip().upper().replace(" ", "")
        if cmd == "":
            cmd = self.last          # empty line repeats the last command
        else:
            self.last = cmd
        lines = self._dispatch(cmd) if cmd else []
        out = ""
        if self.echo:
            out += raw.strip() + self.eol
        out += "".join(l + self.eol for l in lines)
        out += self.eol + ">"
        log.info("%-12s -> %s", raw.strip() or "(repeat)", " | ".join(lines))
        return out

    def _dispatch(self, cmd):
        if cmd.startswith("AT"):
            return self._at(cmd[2:])
        try:
            data = bytes.fromhex(cmd if len(cmd) % 2 == 0 else cmd[:-1])  # drop response-count digit
        except ValueError:
            return ["?"]
        if not data:
            return ["?"]
        prefix = []
        if self.auto_protocol and not self.searched:
            self.searched = True
            prefix = ["SEARCHING..."]
        sid, args = data[0], list(data[1:])
        ext_fmt = 0
        if self.cea is not None:                     # BMW-style: 6F1 + extended address
            ecu = self.car.ecus.get(("ext", self.cea))
            if ecu is None and self.car.discover:
                # Discovery mode: every probed BMW address answers with one DTC equal to its address.
                # DD00xx: a code starting with 0000 looks like an empty slot to apps.
                code = 0xDD0000 | self.cea
                ecu = self.car.ecus[("ext", self.cea)] = ECU("probe ext:%02X" % self.cea,
                                                             [(code, 0x09)])
                log.warning("discovery: BMW module at ext:%02X (reply %03X) -> DTC %06X",
                            self.cea, 0x600 + self.cea, code)
            rx, ext = 0x600 + self.cea, (self.header or 0x6F1) & 0xFF
            ext_fmt = self._ext_format(ecu, sid)
        else:
            target = 0x7E0 if self.header == 0x7DF else self.header
            ecu = self.car.ecus.get(("can", target))
            if ecu is None and self.car.discover and target and 0x600 <= target <= 0x7F7:
                # Discovery mode: every probed address answers with one DTC equal to its address.
                code = 0xDD0000 | target
                ecu = self.car.ecus[("can", target)] = ECU("probe %03X" % target, [(code, 0x09)])
                log.warning("discovery: module at %03X (reply %03X) -> DTC %06X", target, target + 8, code)
            rx, ext = (target or 0) + 8, None
        if ecu is None:
            return prefix + ["NO DATA"]              # no ECU at that address in this car
        if sid < 0x10:
            reply = self._obd(sid, args) if ecu.obd else None
        else:
            reply = self._uds(ecu, sid, args)
        if reply is None:
            return prefix + ["NO DATA"]
        return prefix + self._frames(reply, rx, ext, ext_fmt)

    def _ext_format(self, ecu, sid):
        """Pick the reply format for an extended-address module (see EXT_FORMATS)."""
        car = self.car
        if car.ext_format is not None:
            return car.ext_format
        if ecu is None:
            return 0
        if sid == 0x3E:                              # presence ping: rotate through formats
            n = self.ext_tries.get(self.cea, 0)
            self.ext_tries[self.cea] = n + 1
            self.ext_last[self.cea] = n % len(EXT_FORMATS)
            return self.ext_last[self.cea]
        if self.cea in self.ext_last:                # app moved on after a ping: it liked that format
            car.ext_format = self.ext_last[self.cea]
            log.warning("BMW reply format accepted by the app: #%d %s  (rerun with --ext-format %d)",
                        car.ext_format, EXT_FORMATS[car.ext_format][0], car.ext_format)
            return car.ext_format
        return 0

    # ---------- AT commands ----------
    def _at(self, c):
        if c in ("Z", "WS"):
            self.reset()
            return ["", self.VERSION] if c == "Z" else [self.VERSION]
        if c == "D":
            self.reset()
            return ["OK"]
        if c == "I":
            return [self.VERSION]
        if c == "@1":
            return ["OBDII to RS232 Interpreter"]
        if c == "@2":
            return ["?"]
        if c == "RV":
            return ["%.1fV" % self.car.values()["voltage"]]
        if c == "DP":
            return [("AUTO, " if self.auto_protocol else "") + "ISO 15765-4 (CAN 11/500)"]
        if c == "DPN":
            return ["A6" if self.auto_protocol else "6"]
        if c in ("E0", "E1"):
            self.echo = c == "E1"
            return ["OK"]
        if c in ("L0", "L1"):
            self.linefeed = c == "L1"
            return ["OK"]
        if c in ("S0", "S1"):
            self.spaces = c == "S1"
            return ["OK"]
        if c in ("H0", "H1"):
            self.headers = c == "H1"
            return ["OK"]
        if c.startswith("SP") or c.startswith("TP"):
            arg = c[2:]
            self.auto_protocol = arg.startswith("A") or arg == "0"
            self.searched = False
            return ["OK"]
        if c.startswith("SH"):
            try:
                self.header = int(c[2:], 16) & 0x7FF if len(c[2:]) == 3 else None
            except ValueError:
                return ["?"]
            return ["OK"]
        if c.startswith("CEA"):
            try:
                self.cea = int(c[3:], 16) & 0xFF if c[3:] else None
            except ValueError:
                return ["?"]
            return ["OK"]
        if c.startswith("TA"):                       # tester address (BMW uses F1)
            return ["OK"]
        if c.startswith("CS"):
            return ["T:00 R:00"]
        # Everything else adapters accept silently: timeouts, CAN filters, headers, adaptive timing...
        known = ("M", "AT", "ST", "AL", "NL", "CAF", "CFC", "CF", "CM", "CRA", "SH", "PC",
                 "FC", "D0", "D1", "KW", "BI", "MA", "AR", "IB", "IIA", "SW", "WM", "CEA",
                 "CSM", "R0", "R1", "V0", "V1", "JE", "JS", "PP", "SS", "AMC", "AMT")
        if any(c.startswith(k) for k in known):
            return ["OK"]
        return ["?"]

    # ---------- OBD services ----------
    def _obd(self, mode, args):
        car = self.car
        if mode in (0x01, 0x02):
            if mode == 0x02:            # freeze frame: pairs of PID, frame#
                args = args[0::2]
            v = car.values()
            out = [0x40 + mode]
            for pid in args[:6]:
                if pid % 0x20 == 0:
                    data = supported_bitmap(list(PIDS) + [0x02] * (mode == 2), pid)
                elif mode == 0x02 and pid == 0x02:
                    data = list(encode_dtc(car.dtcs[0])) if car.dtcs else [0, 0]
                elif pid in PIDS:
                    data = PIDS[pid](v, car)
                else:
                    continue
                out += [pid] + ([0x00] if mode == 2 else []) + data
            if mode == 0x02 and not car.dtcs:
                return None
            return out if len(out) > 1 else None
        if mode in (0x03, 0x07, 0x0A):
            codes = {0x03: car.dtcs, 0x07: car.pending, 0x0A: car.dtcs}[mode]
            out = [0x40 + mode, len(codes)]
            for code in codes:
                out += list(encode_dtc(code))
            return out
        if mode == 0x04:
            car.clear_dtcs()
            return [0x44]
        if mode == 0x09 and args:
            pid = args[0]
            if pid == 0x00:
                bits = 0
                for p in MODE09_SUPPORTED:
                    bits |= 1 << (32 - p)
                return [0x49, 0x00] + [(bits >> s) & 0xFF for s in (24, 16, 8, 0)]
            if pid == 0x02:
                return [0x49, 0x02, 0x01] + list(car.vin.encode())
            if pid == 0x04:
                return [0x49, 0x04, 0x01] + list(b"OBDSIM-CAL-0001\0")
            if pid == 0x0A:
                return [0x49, 0x0A, 0x01] + list(b"ECM\0-EngineControl\0\0")[:20]
        return None

    # ---------- UDS (manufacturer diagnostics) ----------
    def _uds(self, ecu, sid, args):
        neg = lambda code: [0x7F, sid, code]
        if sid == 0x3E:                                   # TesterPresent
            return None if args[:1] == [0x80] else [0x7E, 0x00]
        if sid == 0x10 and args:                          # DiagnosticSessionControl
            return [0x50, args[0], 0x00, 0x32, 0x01, 0xF4]
        if sid == 0x19 and args:                          # ReadDTCInformation
            sub, mask = args[0], (args[1] if len(args) > 1 else 0xFF)
            if sub in (0x02, 0x0A, 0x0F, 0x17):
                hits = ecu.dtcs if sub == 0x0A else [d for d in ecu.dtcs if d[1] & mask]
                out = [0x59, sub, 0xFF]
                for code, status in hits:
                    out += [code >> 16, (code >> 8) & 0xFF, code & 0xFF, status]
                return out
            if sub in (0x01, 0x11, 0x12):
                n = sum(1 for d in ecu.dtcs if d[1] & mask)
                return [0x59, sub, 0xFF, 0x01, n >> 8, n & 0xFF]
            return neg(0x12)
        # KWP2000 service 18 (ReadDTCByStatus) is deliberately left unsupported: UDS-era BMW
        # modules reject it, and Car Scanner falls back to it, parsing 2-byte KWP codes.
        if sid == 0x14:                                   # ClearDiagnosticInformation
            ecu.dtcs.clear()
            if ecu.obd:
                self.car.clear_dtcs()
            return [0x54]
        if sid == 0x22 and len(args) >= 2:                # ReadDataByIdentifier
            did = (args[0] << 8) | args[1]
            if did == 0xF190:
                return [0x62, 0xF1, 0x90] + list(self.car.vin.encode())
            return neg(0x31)
        return neg(0x11)
