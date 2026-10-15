#!/usr/bin/env python3
"""Live dashboard for the car, in a browser on this laptop.

    python3 dashboard.py               broker from car_config.h
    python3 dashboard.py 10.0.0.5      another broker

Start the broker first (mosquitto -c ~/mosquitto-car.conf -v), then this,
then open the address it prints if the page does not open by itself:
http://127.0.0.1:8765, or the next free port when that one is taken.

It runs mosquitto_sub on car/#, keeps every message in runs/<date>.jsonl
beside this file, and passes them to dashboard.html as they arrive. The
page sends L, R, S and U through mosquitto_pub, and exports any day it
has kept as a zip of CSV files. Standard library only; nothing is fetched
from the internet. watch.py is the terminal fallback and can run beside it.
"""

import csv
import io
import json
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import webbrowser
import zipfile
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import watch

PORT = 8765      # The first free one from here up is used
HERE = Path(__file__).parent
PAGE = HERE / "dashboard.html"
RUNS = HERE / "runs"
COMMANDS = {"L", "R", "S", "U"}
REPLAY = 3000   # Messages a newly opened page is sent, the newest

g_lock = threading.Lock()
g_history = deque(maxlen=REPLAY)
g_clients = []
g_broker = {"topic": "_broker", "payload": {"state": "starting"}, "t": 0}


def now_ms():
    return int(time.time() * 1000)


def day_of(t_ms):
    return time.strftime("%Y-%m-%d", time.localtime(t_ms / 1000))


def publish(record, keep=True):
    """Hand one record to every open page, and to the history if kept."""
    with g_lock:
        if keep:
            g_history.append(record)
        for client in list(g_clients):
            try:
                client.put_nowait(record)
            except queue.Full:
                g_clients.remove(client)   # A page that stopped reading


def store(record):
    RUNS.mkdir(exist_ok=True)
    with open(RUNS / f"{day_of(record['t'])}.jsonl", "a") as log:
        log.write(json.dumps(record) + "\n")


def set_broker(state):
    g_broker.update(t=now_ms(), payload={"state": state, "host": HOST})
    publish(dict(g_broker), keep=False)


def subscribe():
    """Keep mosquitto_sub running.

    It exits at once when no broker answers, and otherwise reconnects by
    itself after a drop, silently, so the page judges the car's link by
    how old the last message is.
    """
    while True:
        set_broker("connecting")
        try:
            sub = subprocess.Popen(
                ["mosquitto_sub", "-h", HOST, "-t", "car/#", "-F", "%j"],
                stdout=subprocess.PIPE, text=True)
        except FileNotFoundError:
            sys.exit("mosquitto_sub not found: brew install mosquitto")

        # mosquitto_sub exits at once when nothing answers, so one still
        # running a moment later has its connection.
        threading.Timer(1.5, lambda: sub.poll() is None
                        and set_broker("up")).start()
        for line in sub.stdout:
            message = json.loads(line)
            payload = message.get("payload", "")
            try:
                payload = json.loads(payload)
            except ValueError:
                pass   # car/log and car/command are plain text
            record = {"t": now_ms(), "topic": message["topic"],
                      "payload": payload}
            store(record)
            publish(record)

        set_broker("down")
        time.sleep(3)


def load_today():
    path = RUNS / f"{day_of(now_ms())}.jsonl"
    if path.exists():
        with open(path) as log:
            for line in deque(log, maxlen=REPLAY):
                try:
                    g_history.append(json.loads(line))
                except ValueError:
                    pass   # A line cut short by a crash


def days():
    found = []
    for path in sorted(RUNS.glob("*.jsonl"), reverse=True):
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", path.stem):
            continue   # Only the day files dashboard.py writes
        with open(path) as log:
            found.append({"date": path.stem, "messages": sum(1 for _ in log),
                          "bytes": path.stat().st_size})
    return found


def readable(key, value):
    """A CSV header and value, named as watch.py names them."""
    label, how = watch.FIELDS.get(key, (key, None))
    if isinstance(how, str):
        label = f"{label} ({how})"
    if isinstance(how, list) and isinstance(value, int) \
            and 0 <= value < len(how):
        value = how[value]
    return label, value


def csv_text(rows):
    columns = ["time"]
    for row in rows:
        columns += [key for key in row if key not in columns]
    out = io.StringIO()
    writer = csv.DictWriter(out, fieldnames=columns)
    writer.writeheader()
    writer.writerows(rows)
    return out.getvalue()


def export(date):
    """One day as a zip: a CSV per topic, the log as text, and the raw file."""
    path = RUNS / f"{date}.jsonl"
    tables = {"telemetry": [], "terrain": [], "heartbeat": [], "events": [],
              "commands": []}
    log_lines = []
    with open(path) as log:
        for line in log:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            stamp = time.strftime("%Y-%m-%d %H:%M:%S",
                                  time.localtime(record["t"] / 1000))
            stamp += f".{record['t'] % 1000:03d}"
            topic = record["topic"].removeprefix("car/")
            payload = record["payload"]
            if topic == "log":
                log_lines += [f"{stamp}  {text}"
                              for text in str(payload).splitlines()]
            elif topic == "command":
                tables["commands"].append({"time": stamp,
                                           "command": payload})
            elif isinstance(payload, dict):
                row = {"time": stamp}
                for key, value in watch.flatten(payload):
                    label, value = readable(key, value)
                    row[label] = value
                tables["events" if topic == "event" else topic].append(row)

    out = io.BytesIO()
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, rows in tables.items():
            if rows:
                archive.writestr(f"{date}/{name}.csv", csv_text(rows))
        archive.writestr(f"{date}/log.txt", "\n".join(log_lines) + "\n")
        archive.write(path, f"{date}/raw.jsonl")
    return out.getvalue()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass   # Keep the terminal for errors

    def reply(self, code, body=b"", kind="text/plain; charset=utf-8",
              extra=None):
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/":
            self.reply(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        elif url.path == "/stream":
            self.stream()
        elif url.path == "/days":
            self.reply(200, json.dumps(days()).encode(), "application/json")
        elif url.path == "/export":
            date = parse_qs(url.query).get("date", [""])[0]
            if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", date) \
                    or not (RUNS / f"{date}.jsonl").exists():
                self.reply(404, b"no data kept for that date")
                return
            self.reply(200, export(date), "application/zip", {
                "Content-Disposition":
                    f'attachment; filename="car_{date}.zip"'})
        else:
            self.reply(404, b"not found")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        command = self.rfile.read(min(length, 8)).decode().strip().upper()
        if urlparse(self.path).path != "/command" or command not in COMMANDS:
            self.reply(400, b"send one of L, R, S, U to /command")
            return
        sent = subprocess.run(["mosquitto_pub", "-h", HOST, "-t",
                               "car/command", "-m", command],
                              capture_output=True, text=True, timeout=5)
        if sent.returncode == 0:
            self.reply(204)
        else:
            self.reply(502, (sent.stderr or "broker unreachable").encode())

    def stream(self):
        """Server-Sent Events: today's history first, then live."""
        client = queue.Queue(maxsize=1000)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with g_lock:
            backlog = list(g_history) + [dict(g_broker)]
            g_clients.append(client)
        try:
            self.wfile.write(b"retry: 2000\n\n")
            for record in backlog:
                self.wfile.write(f"data: {json.dumps(record)}\n\n".encode())
            self.wfile.write(b"event: replayed\ndata: {}\n\n")
            self.wfile.flush()
            while True:
                try:
                    record = client.get(timeout=15)
                    chunk = f"data: {json.dumps(record)}\n\n"
                except queue.Empty:
                    chunk = ": still here\n\n"   # Keeps the page connected
                self.wfile.write(chunk.encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with g_lock:
                if client in g_clients:
                    g_clients.remove(client)


def taken(port):
    """Whether something already answers on the port, IPv4 or IPv6.

    Docker listens on every address, so binding 127.0.0.1 alone can succeed
    beside it while a browser asking for localhost still reaches Docker.
    """
    for host in ("127.0.0.1", "::1"):
        try:
            socket.create_connection((host, port), timeout=0.2).close()
            return True
        except OSError:
            pass
    return False


def main():
    load_today()
    threading.Thread(target=subscribe, daemon=True).start()
    for port in range(PORT, PORT + 10):
        if taken(port):
            continue
        try:
            server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue   # Taken by something else, try the next
    else:
        sys.exit(f"ports {PORT} to {PORT + 9} are all in use")
    server.daemon_threads = True
    print(f"dashboard at http://127.0.0.1:{port}, car/# on {HOST}. "
          "Ctrl-C to stop.", flush=True)
    webbrowser.open(f"http://127.0.0.1:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


HOST = watch.broker()

if __name__ == "__main__":
    main()
