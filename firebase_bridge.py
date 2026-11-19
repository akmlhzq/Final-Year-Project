#!/usr/bin/env python3
"""
Firebase <-> ESP32 relay bridge (runs on the Raspberry Pi).

App writes   /robot/relays/relay1, relay2, relay4, relay5   (true / false)
Bridge sends "R <n> <1|0>" to the ESP32 over serial.
ESP32 replies "RELAYS a b c d e"; bridge writes it to /robot/status/relays
so the app can show the real state.

Relay 3 follows relay 1 on the ESP32, so the app only needs a relay1 button
(do NOT create a relay3 key in the database).

Install:  pip install firebase-admin pyserial
"""
import threading
import time

import serial
import firebase_admin
from firebase_admin import credentials, db

# ---------- settings ----------
SERIAL_PORT = "/dev/serial0"          # check with: ls -l /dev/serial* /dev/ttyAMA* /dev/ttyS*
BAUD = 115200
KEY_FILE = "serviceAccountKey.json"   # downloaded from Firebase console
DB_URL = "https://vaccum-fyp-default-rtdb.firebaseio.com/"   # <-- change this

# ---------- Firebase ----------
cred = credentials.Certificate(KEY_FILE)
firebase_admin.initialize_app(cred, {"databaseURL": DB_URL})
relays_ref = db.reference("/robot/relays")
status_ref = db.reference("/robot/status/relays")

# ---------- Serial ----------
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.2)
ser_lock = threading.Lock()


def send(line: str) -> None:
    with ser_lock:
        ser.write((line + "\n").encode())
    print("-> ESP32:", line)


def apply_relay(key: str, value) -> None:
    """key looks like 'relay1'..'relay5', value must be a bool."""
    if not key.startswith("relay") or not isinstance(value, bool):
        return
    try:
        n = int(key[5:])
    except ValueError:
        return
    if 1 <= n <= 5:
        send(f"R {n} {1 if value else 0}")


def on_change(event) -> None:
    if event.data is None:
        return
    if event.path == "/" and isinstance(event.data, dict):
        # first event after listen(): the whole relays node
        for key, value in event.data.items():
            apply_relay(key, value)
    else:
        apply_relay(event.path.lstrip("/"), event.data)


def read_serial() -> None:
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("RELAYS"):
                parts = line.split()[1:]
                if len(parts) == 5:
                    status_ref.set({f"relay{i + 1}": p == "1"
                                    for i, p in enumerate(parts)})
        except Exception as e:      # keep the thread alive
            print("serial/firebase error:", e)
            time.sleep(1)


if __name__ == "__main__":
    threading.Thread(target=read_serial, daemon=True).start()
    relays_ref.listen(on_change)
    print("Bridge running. Ctrl+C to stop.")
    while True:
        time.sleep(1)
