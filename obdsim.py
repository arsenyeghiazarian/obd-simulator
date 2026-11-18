#!/usr/bin/env python3
"""ELM327 OBD-II adapter simulator.

Exposes a fake ELM327 over WiFi (TCP, like WiFi OBD dongles) and/or Bluetooth LE
(like Vgate/Veepeak BLE dongles) so phone apps such as Car Scanner can connect to it.
"""

import argparse
import asyncio
import logging
import os
import socket
import sys

from elm327 import ELM327
from vehicle import Vehicle, encode_dtc

log = logging.getLogger("obdsim")


# ---------------------------------------------------------------- WiFi / TCP
async def run_tcp(car, host, port):
    async def client(reader, writer):
        peer = writer.get_extra_info("peername")
        log.info("WiFi client connected: %s", peer)
        elm = ELM327(car)
        buf = b""
        try:
            while True:
                chunk = await reader.read(256)
                if not chunk:
                    break
                buf += chunk
                while True:
                    idx = min((i for i in (buf.find(b"\r"), buf.find(b"\n")) if i >= 0), default=-1)
                    if idx < 0:
                        break
                    line, buf = buf[:idx], buf[idx + 1:]
                    if line == b"" and buf[:1] in (b"\n", b"\r"):
                        continue  # swallow the \n of a \r\n pair
                    writer.write(elm.handle(line.decode(errors="ignore")).encode())
                    await writer.drain()
        except (ConnectionError, asyncio.IncompleteReadError):
            pass
        log.info("WiFi client disconnected: %s", peer)
        writer.close()

    try:
        server = await asyncio.start_server(client, host, port)
    except OSError as e:
        log.error("WiFi server not started: port %d is busy (%s). Is another obdsim still "
                  "running? Stop it with:  pkill -f obdsim.py", port, e.strerror)
        return
    log.info("WiFi (TCP) ELM327 listening on %s:%d  (phone: use IP %s, port %d)",
             host, port, local_ip(), port)
    async with server:
        await server.serve_forever()


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


# ---------------------------------------------------------------- Bluetooth LE
# Two common layouts used by BLE OBD dongles; we expose both.
def _u(short):
    return "0000%s-0000-1000-8000-00805f9b34fb" % short


BLE_LAYOUTS = [
    # (service, write char, notify char)
    (_u("fff0"), _u("fff2"), _u("fff1")),   # Vgate iCar Pro / many clones
    (_u("ffe0"), _u("ffe1"), _u("ffe1")),   # HM-10 style single characteristic
]


async def run_ble(car, name, retry=False):
    try:
        from bless import (BlessServer, GATTAttributePermissions,
                           GATTCharacteristicProperties)
    except ImportError:
        log.error("BLE needs the 'bless' package:  pip3 install bless")
        return

    loop = asyncio.get_running_loop()
    server = BlessServer(name=name, loop=loop)
    elm = ELM327(car)
    write_to_layout = {}
    buf = {"data": b""}

    def on_read(characteristic, **kwargs):
        return characteristic.value

    send_lock = asyncio.Lock()
    stats = {"retries": 0}

    async def send(layout, text):
        service, _, notify = layout
        data = text.encode()
        if not retry:
            # Plain sender: 20-byte packets (default BLE payload), 5 ms apart.
            for i in range(0, len(data), 20):       # default BLE MTU payload
                server.get_characteristic(notify).value = bytearray(data[i:i + 20])
                server.update_value(service, notify)
                await asyncio.sleep(0.005)
            return
        async with send_lock:                       # replies must never interleave
            for i in range(0, len(data), 20):       # default BLE MTU payload
                chunk = bytearray(data[i:i + 20])
                # When macOS's notification queue is full, update_value returns False and the
                # packet is NOT sent (Apple: wait until ready, then resend). Dropping it makes
                # the app miss the '>' prompt and restart its setup.
                for attempt in range(300):
                    server.get_characteristic(notify).value = chunk
                    if server.update_value(service, notify):
                        break
                    stats["retries"] += 1
                    await asyncio.sleep(0.02)
                else:
                    log.warning("BLE: packet still refused after 6 s; reply %r truncated", text[:24])
                    return
                await asyncio.sleep(0.005)
        if stats["retries"] >= 50:
            log.info("BLE: %d packets had to be resent (link is busy)", stats["retries"])
            stats["retries"] = 0

    def on_write(characteristic, value, **kwargs):
        layout = write_to_layout.get(str(characteristic.uuid).lower())
        if layout is None:
            return
        buf["data"] += bytes(value)
        while b"\r" in buf["data"]:
            line, buf["data"] = buf["data"].split(b"\r", 1)
            reply = elm.handle(line.decode(errors="ignore").replace("\n", ""))
            asyncio.run_coroutine_threadsafe(send(layout, reply), loop)

    server.read_request_func = on_read
    server.write_request_func = on_write

    props_w = (GATTCharacteristicProperties.write
               | GATTCharacteristicProperties.write_without_response)
    props_n = GATTCharacteristicProperties.notify | GATTCharacteristicProperties.read
    perms = GATTAttributePermissions.readable | GATTAttributePermissions.writeable

    for layout in BLE_LAYOUTS:
        service, wchar, nchar = layout
        await server.add_new_service(service)
        if wchar == nchar:
            await server.add_new_characteristic(service, wchar, props_w | props_n, None, perms)
        else:
            await server.add_new_characteristic(service, nchar, props_n, None, perms)
            await server.add_new_characteristic(service, wchar, props_w, None, perms)
        write_to_layout[wchar] = layout

    await server.start()
    log.info("Bluetooth LE ELM327 advertising as '%s' (services FFF0 and FFE0)", name)
    await asyncio.Event().wait()


# ---------------------------------------------------------------- console
HELP = """commands:
  status                 show live values and DTCs
  dtc P0301 P0420        set stored DTCs (MIL on)      dtc        -> clear them
  pending P0171          set pending DTCs
  set <name> <value>     pin a value, e.g. set speed 120 / set coolant 115 / set rpm 6500
  unset <name> | auto    release one pinned value / all of them
  ecus                   list simulated modules and their DTC counts
  help, quit"""


def console(car, line):
    parts = line.split()
    if not parts:
        return
    cmd, args = parts[0].lower(), parts[1:]
    try:
        if cmd == "status":
            v = car.values()
            print("  " + "  ".join("%s=%.1f" % kv for kv in sorted(v.items())))
            print("  DTCs:", car.dtcs or "-", " pending:", car.pending or "-",
                  " pinned:", car.overrides or "-")
        elif cmd in ("dtc", "pending"):
            for a in args:
                encode_dtc(a)
            codes = [a.upper() for a in args]
            if cmd == "dtc":
                car.dtcs = codes
            else:
                car.pending = codes
            print("  ok:", codes or "cleared")
        elif cmd == "set" and len(args) == 2:
            if args[0] not in car.values():
                print("  unknown name; choose from:", ", ".join(sorted(car.values())))
            else:
                car.overrides[args[0]] = float(args[1])
                print("  ok")
        elif cmd == "unset" and args:
            car.overrides.pop(args[0], None)
        elif cmd == "auto":
            car.overrides.clear()
        elif cmd == "ecus":
            for (kind, addr), ecu in sorted(car.ecus.items()):
                print("  %s:%02X  %-36s %d DTCs" % (kind, addr, ecu.name, len(ecu.dtcs)))
        elif cmd in ("quit", "exit"):
            os._exit(0)
        else:
            print(HELP)
    except ValueError as e:
        print("  error:", e)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="ELM327 OBD-II simulator")
    ap.add_argument("--no-wifi", action="store_true", help="disable the TCP (WiFi) server")
    ap.add_argument("--ble", action="store_true", help="also advertise as a Bluetooth LE adapter")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=35000)
    ap.add_argument("--name", default="OBDII", help="BLE device name")
    ap.add_argument("--ble-retry", action="store_true",
                    help="resend BLE packets macOS refuses (experimental; can make Car Scanner fail)")
    ap.add_argument("--vin", default=None, help="VIN to report (overrides the scenario's VIN)")
    ap.add_argument("--dtc", default="", help="stored DTCs, comma separated (e.g. P0301,P0420)")
    ap.add_argument("--pending", default="", help="pending DTCs, comma separated")
    ap.add_argument("--scenario", help="JSON file with VIN and per-module DTCs (see scenarios/)")
    ap.add_argument("--discover", action="store_true",
                    help="every module address the app probes answers with a DTC equal to its "
                         "address, so the app's module names can be mapped to addresses")
    ap.add_argument("--ext-format", type=int, choices=range(-1, 4), default=2,
                    help="reply format for BMW extended-address modules (default 2: tester byte "
                         "shown, no padding, which Car Scanner accepts); -1 tries all four and "
                         "logs the one the app accepts")
    ap.add_argument("--elm-version", default=ELM327.VERSION,
                    help="what ATZ/ATI report (default %(default)r; e.g. 'ELM327 v1.5' for a cheap clone)")
    ap.add_argument("--log", default="obdsim.log",
                    help="also write the full log to this file, overwritten on each start "
                         "(default obdsim.log; '' to disable)")
    ap.add_argument("-q", "--quiet", action="store_true", help="don't log each command")
    a = ap.parse_args()

    handlers = [logging.StreamHandler()]
    if a.log:
        handlers.append(logging.FileHandler(a.log, mode="w"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S",
                        handlers=handlers)
    if a.quiet:
        logging.getLogger("elm327").setLevel(logging.WARNING)

    split = lambda s: [x.strip().upper() for x in s.split(",") if x.strip()]
    car = Vehicle(vin=a.vin or "1HGCM82633A004352", dtcs=split(a.dtc), pending=split(a.pending))
    ELM327.VERSION = a.elm_version
    car.discover = a.discover
    car.ext_format = None if a.ext_format < 0 else a.ext_format
    if a.scenario:
        car.load_scenario(a.scenario)
        if a.vin:
            car.vin = a.vin.upper()[:17].ljust(17, "0")
        log.info("scenario %s: VIN %s, %d modules", a.scenario, car.vin, len(car.ecus))

    async def amain():
        tasks = []
        if not a.no_wifi:
            tasks.append(asyncio.ensure_future(run_tcp(car, a.host, a.port)))
        if a.ble:
            tasks.append(asyncio.ensure_future(run_ble(car, a.name, a.ble_retry)))
        if not tasks:
            ap.error("nothing to run: enable WiFi or --ble")
        if sys.stdin.isatty():
            try:
                loop = asyncio.get_running_loop()
                loop.add_reader(sys.stdin, lambda: console(car, sys.stdin.readline()))
                print("type 'help' for simulator commands")
            except NotImplementedError:      # Windows event loops can't watch stdin
                log.info("console commands aren't available on this platform")
        await asyncio.gather(*tasks)

    try:
        asyncio.run(amain())
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    main()
