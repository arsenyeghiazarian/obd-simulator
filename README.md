# OBD-II / ELM327 simulator

Test car diagnostic apps without a car. This program pretends to be an **ELM327 OBD-II adapter plugged into a running car**, so apps like **Car Scanner**, **Torque** or your own app can connect to it over **WiFi** or **Bluetooth LE**, read live data, and read fault codes (DTCs).

It goes beyond generic OBD-II: it also simulates **manufacturer modules** (engine, gateway, ABS, airbag, ...) with their own fault codes over UDS. The included BMW G30 scenario reproduces a real 58-code fault report, and Car Scanner reads it exactly like the real car.

- Pure Python, no dependencies for WiFi mode
- Live data that changes over time (a looping drive cycle)
- OBD-II fault codes: stored, pending, permanent; clearing works
- Per-module manufacturer fault codes from a JSON scenario file
- Every request and reply is logged, so you can see exactly what an app sends
- Change values and codes while it runs, from the console

> Tested on macOS with Car Scanner 2.2.52 (iOS). WiFi mode should run anywhere Python 3.8+ runs.

## Quick start

```bash
git clone https://github.com/arsenyeghiazarian/obd-simulator.git
cd obd-simulator
python3 obdsim.py --dtc P0301,P0420
```

It prints something like:

```
WiFi (TCP) ELM327 listening on 0.0.0.0:35000  (phone: use IP 192.168.1.20, port 35000)
```

Now connect your app to that IP address and port `35000` (see [Connect an app](#connect-an-app)). The phone and the computer must be on the same WiFi network.

## Connect an app

### WiFi (recommended)

WiFi dongles are TCP servers on port 35000, and so is the simulator.

**Car Scanner:** Settings → Adapter → connection type **Wi-Fi** → IP = the address the simulator printed, port `35000` → Connect.

**Torque and other apps:** pick a WiFi / TCP ELM327 adapter and enter the same IP and port.

**Your own code or a terminal:**

```bash
nc 127.0.0.1 35000
ATZ
010C
```

If the phone can't connect, check that the macOS firewall allows incoming connections for Python, and that the phone isn't on a guest network that isolates devices.

### Bluetooth LE (macOS)

The computer advertises as a BLE OBD dongle (services `FFF0`/`FFF1`/`FFF2` and `FFE0`/`FFE1`, like Vgate and Veepeak BLE adapters).

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python obdsim.py --ble --dtc P0301
```

- Run it from Terminal.app or iTerm. The first time, macOS asks to give the terminal Bluetooth access; allow it. If you denied it earlier, turn it on in System Settings → Privacy & Security → Bluetooth. Without it, macOS stops the program with "Aborted".
- In the app, choose **Bluetooth LE** and pick the device. iOS may list it under your Mac's name instead of `OBDII`.
- `--no-wifi` runs Bluetooth only.

> **Use WiFi for long fault reports.** Over Bluetooth, macOS sometimes drops notification packets when a reply is long (for example a module with 30+ codes). The app then shows that module with fewer or no codes. Short replies and live data work fine over Bluetooth.

Classic Bluetooth (SPP) dongles can't be emulated on macOS, because macOS no longer lets apps host an SPP serial port.

## What it simulates

**Live data (mode 01)**, following a 90-second drive loop: idle → accelerate to ~100 km/h → cruise → brake. Coolant and oil warm up over a few minutes.

| PID | Value | PID | Value |
|---|---|---|---|
| 01 | Monitor status, MIL, DTC count | 1F | Run time since start |
| 03 | Fuel system status | 21 | Distance with MIL on |
| 04 | Engine load | 2F | Fuel level |
| 05 | Coolant temperature | 31 | Distance since codes cleared |
| 06 / 07 | Short / long term fuel trim | 33 | Barometric pressure |
| 0B | Intake manifold pressure | 42 | Control module voltage |
| 0C | RPM | 46 | Ambient temperature |
| 0D | Speed | 4E | Time since codes cleared |
| 0E | Timing advance | 51 | Fuel type |
| 0F | Intake air temperature | 5C | Oil temperature |
| 10 | MAF air flow | 00 / 20 / 40 | Supported-PID bitmaps |
| 11 | Throttle position | 1C | OBD standard |

Several PIDs can be asked in one request (`010C0D`).

**Other OBD-II services**

| Mode | What it does |
|---|---|
| 02 | Freeze frame |
| 03 / 07 / 0A | Stored / pending / permanent DTCs |
| 04 | Clear DTCs (MIL goes off) |
| 09 | VIN, calibration ID, ECU name (multi-frame ISO-TP) |

**Manufacturer diagnostics (UDS)** for modules defined in a scenario:

| Service | What it does |
|---|---|
| 3E | Tester present |
| 10 | Diagnostic session control |
| 19 01 / 02 / 0A / 0F / 11 / 12 / 17 | Read DTCs and DTC counts, filtered by status mask |
| 14 | Clear DTCs |
| 22 F190 | Read VIN |

**ELM327 AT commands** that apps send during setup: `Z`, `I`, `D`, `WS`, `E0/1`, `L0/1`, `S0/1`, `H0/1`, `SP`, `TP`, `DP`, `DPN`, `RV`, `@1`, `SH`, `CEA`, `TA`, `CS`. Filters, timeouts, flow control and other settings (`ST`, `AT`, `CRA`, `CF`, `CM`, `FC...`, `AR`, ...) are accepted with `OK`.

The protocol is ISO 15765-4 CAN, 11-bit, 500 kbit/s (`ATDPN` → `A6`). The adapter reports itself as `ELM327 v2.2`.

## Command-line options

```bash
python3 obdsim.py [options]
```

| Option | What it does |
|---|---|
| `--dtc P0301,P0420` | Stored OBD-II codes (check-engine light on) |
| `--pending P0171` | Pending OBD-II codes |
| `--vin VIN` | VIN to report (overrides the scenario's VIN) |
| `--scenario FILE` | Load a JSON scenario with modules and their codes (see below) |
| `--port 35000` | TCP port for WiFi mode |
| `--host 0.0.0.0` | Address to listen on |
| `--ble` | Also advertise as a Bluetooth LE adapter (needs `bless`) |
| `--no-wifi` | Don't start the WiFi server |
| `--name OBDII` | Bluetooth device name |
| `--elm-version "ELM327 v1.5"` | What `ATZ`/`ATI` reports (default `ELM327 v2.2`) |
| `--log FILE` | Also write the log to this file (default `obdsim.log`; `--log ""` turns it off) |
| `-q` | Don't log every command |
| `--discover` | Every module address an app probes answers with one fault code equal to its address. Use it to find out which address an app uses for each module name |
| `--ext-format N` | Reply format for BMW-style modules (default `2`; `-1` tries all four and logs the one the app accepts) |
| `--ble-retry` | Resend Bluetooth packets macOS refuses (experimental; made Car Scanner fail in testing) |

## Console commands

While it runs, type commands into the terminal:

```
status                  live values and DTCs
dtc P0300 P0420         set stored DTCs (MIL on); "dtc" alone clears them
pending P0128           set pending DTCs
set speed 130           pin a value (rpm, speed, coolant, fuel, voltage, ...)
unset speed             release one pinned value
auto                    release all pinned values
ecus                    list scenario modules and their DTC counts
help, quit
```

## Scenarios

A scenario is a JSON file with a VIN, OBD-II codes, and a list of modules, each with its own fault codes.

```bash
python3 obdsim.py --scenario scenarios/example.json
```

Included:

| File | What it is |
|---|---|
| [`scenarios/example.json`](scenarios/example.json) | A small generic example to copy |
| [`scenarios/bmw_g30.json`](scenarios/bmw_g30.json) | A real BMW G30 (5 Series) fault report: 58 codes in 12 modules (engine 9, CAN gateway 32, power control unit 5, DSC, ABS, roof module, camera, instrument cluster, infotainment, ...). The codes come from a real car; the VIN is made up |

### Format

```json
{
  "vin": "1HGCM82633A004352",
  "obd_dtcs": ["P0301", "P0420"],
  "obd_pending": ["P0171"],
  "ecus": [
    {
      "name": "Transmission",
      "address": "can:7E1",
      "dtcs": [["070000", "09"]]
    },
    {
      "name": "Engine (BMW DME)",
      "address": "ext:12",
      "dtcs": [["120408", "24"], ["140210", "A8"]]
    }
  ]
}
```

- **`obd_dtcs` / `obd_pending`**: generic OBD-II codes, answered on modes 03 / 07 / 0A.
- **`address`** (also a list, to answer at several addresses):
  - `can:7E1`: an 11-bit CAN request ID. The reply comes from that ID + 8 (`7E9`). Use this for most brands.
  - `ext:12`: a BMW-style module address (CAN extended addressing). The app sends `ATSH6F1` + `ATCEA12`; the reply comes from `612` with `F1` as the first byte.
  - `null`: the module exists in the file but doesn't answer.
- **`dtcs`**: `[code, status]` pairs, both hex.
  - `code` is the 3-byte UDS code: the 2-byte DTC plus a failure-type byte. `070000` is P0700, `523400` is C1234. BMW apps show the raw hex, like `480A11`.
  - `status` is the UDS status byte, a sum of these bits:

    | Bit | Meaning |
    |---|---|
    | `01` | Test failed |
    | `02` | Test failed this operation cycle |
    | `04` | Pending |
    | `08` | Confirmed |
    | `10` | Test not completed since last clear |
    | `20` | Test failed since last clear |
    | `40` | Test not completed this operation cycle |
    | `80` | Warning indicator requested |

    Example: `2F` = 01 + 02 + 04 + 08 + 20 → "Test failed, failed this cycle, pending, confirmed, failed since clear".

### Making a scenario for your car

1. Read your car with the app and export the fault report.
2. Find the module addresses the app uses: run with `--discover` and scan. Every module the app asks for answers with a code equal to its address (`DD0012` = `ext:12`, `DD07E1` = `can:7E1`), so the app's report tells you which name goes with which address. The log shows them too.
3. Write the codes and status bytes into a JSON file like the ones above, scan again and compare the reports.

## Car Scanner notes

These are things Car Scanner (iOS) needs, learned by testing against the BMW profile:

- Use the **BMW → OBD-II / EOBD G-body** connection profile for the BMW scenario.
- Use WiFi for the BMW scenario; the gateway's 32-code reply is too long to arrive reliably over Bluetooth.
- The adapter has to report version 2.x. As `ELM327 v1.5`, Car Scanner reads only the first CAN frame of each module's reply.
- BMW replies are sent unpadded with the `F1` tester byte shown; padded frames are rejected.
- KWP2000 service `18` is deliberately rejected: Car Scanner falls back to it and would show wrong 2-byte codes.
- Run only one copy of the simulator, and fully close the app before reconnecting.

**How to tell a scan went wrong from the log:** for a module with codes, a good scan shows `1902AC`, `1902AF` and then `ATAR`. If the app also sends `1800FFFF` / `1802FFFF` right after, it didn't get (or didn't accept) the reply.

## Files

| File | What it does |
|---|---|
| `obdsim.py` | Entry point: WiFi and Bluetooth LE transports, console, options |
| `elm327.py` | ELM327 command interpreter, OBD-II and UDS replies, CAN/ISO-TP frame formatting |
| `vehicle.py` | Simulated car: drive cycle, live values, DTCs, scenario loading |
| `scenarios/` | Example scenarios |

## Limitations

- Only CAN 11-bit 500 kbit/s is simulated; older protocols (K-line, J1850, ...) aren't.
- Only one app can be connected over Bluetooth at a time.
- The console needs a real terminal (it's skipped when input isn't a TTY), and doesn't work on Windows.
- Bluetooth LE mode is tested on macOS only.

## License

[MIT](LICENSE)
