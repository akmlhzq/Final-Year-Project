#!/usr/bin/env python3
"""
Raspberry Pi 4: read a 2D LiDAR (Slamtec RPLIDAR C1 over USB) and make the robot
follow the nearest object in front of it.

Pi  --UART-->  ESP32  --> 2x BTS7960 --> motors
Command format sent to ESP32 (one line, 20 Hz-ish):  "L,R\n"   (L, R = -255..255)

Install (Ubuntu, use a virtual environment):
    sudo apt install python3-venv
    python3 -m venv ~/lidar-env && source ~/lidar-env/bin/activate
    pip install rplidar-roboticia pyserial
Pi UART on Ubuntu (no raspi-config):
    /boot/firmware/config.txt  -> add:  enable_uart=1  and  dtoverlay=disable-bt
    /boot/firmware/cmdline.txt -> remove:  console=serial0,115200  (or console=ttyAMA0,115200)
    sudo systemctl disable --now serial-getty@ttyAMA0.service
    sudo usermod -aG dialout $USER   (then log out/in)
    reboot -> UART is /dev/ttyAMA0
    Wiring: Pi TX (GPIO14, pin 8)  -> ESP32 RX2 (GPIO16)
            Pi RX (GPIO15, pin 10) <- ESP32 TX2 (GPIO17)
            GND <-> GND
"""

import time
import serial
from rplidar import RPLidar, RPLidarException

# ---------------- settings (tune these) ----------------
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800    # RPLIDAR C1 uses 460800 (the library default 115200 will NOT work)
ESP_PORT = "/dev/ttyAMA0"   # Ubuntu: GPIO14/15 UART after dtoverlay=disable-bt
ESP_BAUD = 115200

ANGLE_OFFSET = 0       # deg; set to 90/180/270 if LiDAR 0° is not the robot's front
FOV = 90               # only look +-FOV degrees around the front
MIN_DIST = 250         # mm, ignore anything closer (robot body / noise)
MAX_DIST = 1500        # mm, ignore anything farther
TARGET_DIST = 700      # mm, distance the robot tries to keep from the target

KP_TURN = 2.0          # PWM units per degree of error
KP_FWD = 0.20          # PWM units per mm of distance error
MAX_SPEED = 150        # forward limit (0-255), keep low while testing
MAX_TURN = 120
ANGLE_DEADBAND = 5     # deg, no steering if target is inside this
LOST_SCANS = 5         # stop after this many scans without a target
# --------------------------------------------------------


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(a):
    """LiDAR angle (0..360, clockwise) -> -180..180, 0 = front, + = right."""
    a = (a + ANGLE_OFFSET) % 360
    return a - 360 if a > 180 else a


def find_target(scan):
    """Return (angle_deg, dist_mm) of the nearest object, or None."""
    pts = []
    for quality, angle, dist in scan:
        if quality == 0:
            continue
        a = norm_angle(angle)
        if abs(a) <= FOV and MIN_DIST <= dist <= MAX_DIST:
            pts.append((a, dist))
    if not pts:
        return None
    nearest = min(d for _, d in pts)
    # average the points around the nearest one -> less jitter
    cluster = [(a, d) for a, d in pts if d - nearest < 150]
    ang = sum(a for a, _ in cluster) / len(cluster)
    dist = sum(d for _, d in cluster) / len(cluster)
    return ang, dist


def compute_motors(angle, dist):
    # forward speed: only drive forward when target is farther than TARGET_DIST
    fwd = clamp(KP_FWD * (dist - TARGET_DIST), 0, MAX_SPEED)
    turn = 0 if abs(angle) < ANGLE_DEADBAND else clamp(KP_TURN * angle, -MAX_TURN, MAX_TURN)
    left = clamp(fwd + turn, -255, 255)    # target on right (+angle) -> left wheel faster
    right = clamp(fwd - turn, -255, 255)
    return int(left), int(right)


def send(esp, left, right):
    esp.write(f"{left},{right}\n".encode())


def main():
    esp = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.1)
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)
    lost = 0
    print("LiDAR info:", lidar.get_info())
    print("LiDAR health:", lidar.get_health())

    try:
        while True:
            try:
                for scan in lidar.iter_scans(max_buf_meas=800):
                    target = find_target(scan)
                    if target is None:
                        lost += 1
                        if lost >= LOST_SCANS:
                            send(esp, 0, 0)
                        continue
                    lost = 0
                    angle, dist = target
                    left, right = compute_motors(angle, dist)
                    send(esp, left, right)
                    print(f"target {angle:6.1f} deg  {dist:6.0f} mm  ->  L={left:4d} R={right:4d}")
            except RPLidarException as e:
                print("LiDAR error, retrying:", e)
                send(esp, 0, 0)
                lidar.stop()
                lidar.clean_input()
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        send(esp, 0, 0)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        esp.close()


if __name__ == "__main__":
    main()
