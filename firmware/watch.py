#!/usr/bin/env python3
"""Show the car's MQTT messages one block each, every field named.

    python3 watch.py                  broker from car_config.h
    python3 watch.py 10.0.0.5         another broker
    python3 watch.py | tee run.txt    keep a copy to paste or attach

Events stand out with >>> so a hit, a barcode or an obstacle is easy to
spot between the 5 s messages. Needs mosquitto_sub, from
`brew install mosquitto`. Ctrl-C stops it.

The key meanings follow comms/README.md. A key not listed here prints
under its own name, so a new one still shows.
"""

import json
import re
import subprocess
import sys
import time
from pathlib import Path

CONFIG = Path(__file__).parent / "app_program" / "common" / "car_config.h"

# car.h enums, in order. Renumbering one there means updating it here.
STATES = ["init", "follow line", "decode barcode", "execute turn",
          "avoid obstacle", "recover line", "halted", "collision"]
NAV = ["none", "left", "right", "straight", "u-turn"]
MOTIONS = ["still", "accelerating", "turning", "climbing", "descending",
           "impact"]
CODES = {"A": "A, turn left", "B": "B, turn right", "C": "C, straight on",
         "D": "D, u-turn"}

PERIODIC = {
    "car/telemetry": "TELEMETRY  (every 5 s)",
    "car/terrain":   "TERRAIN  (every 5 s)",
    "car/heartbeat": "HEARTBEAT  (every 5 s)",
}

EVENTS = {
    "hit":      "the car was hit, or a wheel stalled",
    "obstacle": "an obstacle scan finished",
    "barcode":  "a barcode was read",
    "hump":     "the car came off a hump",
    "action":   "the car started doing something else",
}


def yes_no(value):
    return "yes" if value else "no"


def sensors(mask):
    names = [("left", 1), ("barcode", 2), ("right", 4)]
    return ", ".join(f"{name} {'dark' if mask & bit else 'light'}"
                     for name, bit in names)


# key: (label, unit text, or a list to look the number up in, or a function)
FIELDS = {
    "state":     ("mission state", STATES),
    "action":    ("doing", None),
    "speed":     ("speed", "mm/s"),
    "dist":      ("distance driven", "mm"),
    "encl":      ("left encoder", "counts"),
    "encr":      ("right encoder", "counts"),
    "mask":      ("line sensors", sensors),
    "nav":       ("last command", NAV),
    "barcode":   ("last barcode", None),
    "barcodes":  ("barcodes read", None),
    "hump":      ("highest hump", "mm"),
    "obst.valid":   ("obstacle found", yes_no),
    "obst.bearing": ("obstacle bearing", "deg, + is left"),
    "obst.range":   ("obstacle range", "mm"),
    "obst.width":   ("obstacle width", "mm"),
    "obst.left":    ("clearance left", "mm"),
    "obst.right":   ("clearance right", "mm"),
    "imu.pitch":  ("pitch", "deg, nose up +"),
    "imu.ok":     ("pitch trusted", yes_no),
    "imu.head":   ("heading", "deg"),
    "imu.rate":   ("turn rate", "deg/s"),
    "imu.event":  ("motion", MOTIONS),
    "imu.hit":    ("backing off a hit", yes_no),
    "imu.stable": ("terrain stable", yes_no),
    "imu.rough":  ("roughness", "mg"),
    "pitch":     ("pitch", "deg, nose up +"),
    "ok":        ("pitch trusted", yes_no),
    "mag":       ("acceleration", "mg"),
    "rough":     ("roughness", "mg"),
    "terrain":   ("terrain", None),
    "event":     ("motion", None),
    "on_hump":   ("on a hump", yes_no),
    "humps":     ("humps crossed", None),
    "last_mm":   ("last hump", "mm"),
    "peak_mm":   ("highest hump", "mm"),
    "uptime_ms": ("up for", lambda ms: f"{ms / 1000:.0f} s"),
    "connected": ("connected", yes_no),
    "sessions":  ("connections so far", None),
    "code":      ("barcode", lambda code: CODES.get(code, f"{code}, unused")),
    "count":     ("count so far", None),
    "mm":        ("this hump", "mm"),
    "found":     ("obstacle found", yes_no),
    "bearing":   ("bearing", "deg, + is left"),
    "range":     ("range", "mm"),
    "width":     ("width", "mm"),
    "left":      ("clearance left", "mm"),
    "right":     ("clearance right", "mm"),
}


def broker():
    if len(sys.argv) > 1:
        return sys.argv[1]
    found = re.search(r'#define\s+COMMS_MQTT_BROKER_HOST\s+"([^"]+)"',
                      CONFIG.read_text())
    return found.group(1) if found else "localhost"


def flatten(values, prefix=""):
    for key, value in values.items():
        if isinstance(value, dict):
            yield from flatten(value, prefix + key + ".")
        else:
            yield prefix + key, value


def field(key, value):
    label, how = FIELDS.get(key, (key, None))
    if callable(how):
        text = how(value)
    elif isinstance(how, list):
        text = how[value] if 0 <= value < len(how) else str(value)
    elif how:
        text = f"{value} {how}"
    else:
        text = str(value) if value != "" else "none yet"
    return f"    {label:<20} {text}"


gb_after_block = False   # True when a blank line is already above


def show(topic, payload):
    global gb_after_block
    now = time.strftime("%H:%M:%S")

    # Log lines run together. Every other message is a block with a blank
    # line either side.
    if topic == "car/log":
        for line in payload.splitlines():
            print(f"{now}  log  {line}")
        gb_after_block = False
        return
    if not gb_after_block:
        print()
    gb_after_block = True
    if topic == "car/command":
        print(f"{now}  COMMAND SENT  {payload}\n")
        return

    try:
        values = dict(flatten(json.loads(payload)))
    except ValueError:
        print(f"{now}  {topic}  {payload}")
        return

    if topic == "car/event":
        kind = values.pop("type", "?")
        print(f"{now}  >>> EVENT: {kind.upper()}  "
              f"({EVENTS.get(kind, 'new event type')})")
    else:
        print(f"{now}  {PERIODIC.get(topic, topic)}")

    # A scan that found nothing has nothing to say about the obstacle.
    if values.get("obst.valid") == 0:
        values = {key: value for key, value in values.items()
                  if not key.startswith("obst.") or key == "obst.valid"}

    for key, value in values.items():
        print(field(key, value))
    print()


def main():
    host = broker()
    print(f"watching car/# on {host}, Ctrl-C to stop", flush=True)
    try:
        sub = subprocess.Popen(["mosquitto_sub", "-h", host, "-t", "car/#",
                                "-F", "%j"],
                               stdout=subprocess.PIPE, text=True)
    except FileNotFoundError:
        sys.exit("mosquitto_sub not found: brew install mosquitto")

    try:
        for line in sub.stdout:
            message = json.loads(line)
            show(message["topic"], message.get("payload", ""))
            sys.stdout.flush()
    except KeyboardInterrupt:
        return

    if sub.wait() != 0:
        sys.exit(f"\nno broker at {host}. Start it in its own terminal:\n"
                 "    mosquitto -c ~/mosquitto-car.conf -v")


if __name__ == "__main__":
    main()
