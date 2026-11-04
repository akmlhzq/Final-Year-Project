#!/usr/bin/env python3
import math
import time
from typing import NamedTuple

ESP32_PORT = "/dev/ttyUSB0"
FRONT_OFFSET_DEG = 180.0

FORWARD_PWM = 100
SLOW_PWM = 70
TURN_PWM = 65
REVERSE_PWM = 70

RUN_SECONDS = 0  # 0 = run until Ctrl+C; set 30 for a timed test.

# Your body measurements, in metres.
FRONT = 0.12
BACK = 0.30
HALF_WIDTH = 0.18

# Front distances are measured forward from the LiDAR.
FRONT_STOP_M = 0.16
FRONT_CLEAR_M = 0.18
FRONT_SLOW_M = 0.36

# Rear clearance is measured from the REAR BODY EDGE.
REAR_STOP_GAP_M = 0.20

BRAKE_SECONDS = 0.25
MAX_REVERSE_SECONDS = 2.0
MAX_TURN_SECONDS = 5.0
MIN_TURN_SECONDS = 0.60

# Assumes wheel axle at body centre, 9 cm behind the LiDAR.
# Change to the actual measured position if different.
WHEEL_AXLE_X = (FRONT - BACK) / 2

TURN_RADIUS = max(
    math.hypot(x - WHEEL_AXLE_X, y)
    for x in (-BACK, FRONT)
    for y in (-HALF_WIDTH, HALF_WIDTH)
) + 0.03

REVERSE_CLEAR_M = max(
    0.26,
    WHEEL_AXLE_X + TURN_RADIUS + 0.03,
)


class View(NamedTuple):
    front: float
    rear_gap: float
    front_seen: bool
    rear_seen: bool
    turn_seen: bool
    turn_clear: bool
    left: float
    right: float


def scan_view(scan):
    n = len(scan.ranges)
    metadata = (
        scan.angle_min,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
    )

    if (
        n < 36
        or not all(math.isfinite(v) for v in metadata)
        or not math.radians(350)
        <= abs(scan.angle_increment) * (n - 1)
        <= math.radians(370)
        or not 0 <= scan.range_min < scan.range_max
    ):
        return None

    seen = [False] * n
    quadrants = [[], [], [], []]
    fi, ri, left, right = [], [], [], []

    front = math.inf
    rear_gap = math.inf
    masked_front = 0
    turn_clear = True

    for i, raw in enumerate(scan.ranges):
        a = (
            scan.angle_min
            + i * scan.angle_increment
            - math.radians(FRONT_OFFSET_DEG)
        )
        a = (a + math.pi) % (2 * math.pi) - math.pi

        q = min(3, int((a + math.pi) / (math.pi / 2)))
        quadrants[q].append(i)

        if abs(a) <= math.radians(65):
            fi.append(i)

        if abs(a) >= math.radians(115):
            ri.append(i)

        r = float(raw)

        if (
            not math.isfinite(r)
            or not scan.range_min <= r <= scan.range_max
            or r <= 0
        ):
            continue

        x = r * math.cos(a)
        y = r * math.sin(a)

        # Exclude returns within the measured body footprint.
        # These rays remain unobserved for coverage checks.
        if -BACK <= x <= FRONT and abs(y) <= HALF_WIDTH:
            masked_front += int(abs(a) <= math.radians(65))
            continue

        seen[i] = True

        if math.hypot(x - WHEEL_AXLE_X, y) <= TURN_RADIUS:
            turn_clear = False

        if abs(y) <= HALF_WIDTH + 0.05:
            if x >= FRONT:
                front = min(front, x)

            if x <= -BACK:
                rear_gap = min(rear_gap, -BACK - x)

        if math.radians(60) <= a <= math.radians(120):
            left.append(r)

        if math.radians(-120) <= a <= math.radians(-60):
            right.append(r)

    def coverage(indices, minimum):
        flags = [seen[i] for i in indices]

        if len(flags) < 10 or sum(flags) / len(flags) < minimum:
            return False

        longest = run = 0

        for good in flags + flags:
            run = 0 if good else run + 1
            longest = max(longest, run)

        missing_angle = (
            min(longest, len(flags))
            * abs(scan.angle_increment)
        )
        return missing_angle <= math.radians(20)

    def space(values):
        if not values:
            return 0.0
        return sorted(values)[int(0.2 * (len(values) - 1))]

    front_seen = (
        coverage(fi, 0.60)
        and masked_front / max(1, len(fi)) <= 0.10
    )

    rear_seen = coverage(ri, 0.60)

    turn_seen = (
        coverage(list(range(n)), 0.50)
        and all(
            len(q) >= 5
            and sum(seen[i] for i in q) / len(q) >= 0.50
            for q in quadrants
        )
    )

    return View(
        front,
        rear_gap,
        front_seen,
        rear_seen,
        turn_seen,
        turn_clear,
        space(left),
        space(right),
    )


class Avoider:
    def __init__(self):
        self.phase = "FORWARD"
        self.since = 0.0
        self.direction = 1
        self.backed = False
        self.fault = ""

    def fail(self, message):
        self.fault = message
        return message, 0, 0

    def step(self, v, fresh, now):
        if self.fault:
            return self.fault, 0, 0

        if not fresh or v is None:
            return "STOP: scan missing or stale", 0, 0

        # Reverse while continuously checking the rear.
        if self.phase == "REVERSE":
            if (
                v.front_seen
                and v.front >= REVERSE_CLEAR_M
                and v.turn_clear
            ):
                self.phase = "BRAKE"
                self.since = now
                return "STOP: reverse complete", 0, 0

            if now - self.since >= MAX_REVERSE_SECONDS:
                return self.fail(
                    "STOP: reverse timeout; check wheel motion and readings"
                )

            if not v.rear_seen:
                return "STOP: rear scan insufficient", 0, 0

            if v.rear_gap <= REAR_STOP_GAP_M:
                return "STOP: obstacle behind robot", 0, 0

            return "REVERSE", -REVERSE_PWM, -REVERSE_PWM

        if not v.front_seen:
            return "STOP: front scan insufficient", 0, 0

        # Turn while continuously checking surrounding clearance.
        if self.phase == "TURN":
            if now - self.since >= MAX_TURN_SECONDS:
                return self.fail(
                    "STOP: turn timeout; check wheel direction and readings"
                )

            if not v.turn_seen:
                return "STOP: side/rear scan insufficient for turning", 0, 0

            if not v.turn_clear:
                self.phase = "BRAKE"
                self.since = now
                return "STOP: turning space blocked", 0, 0

            # After reversing, seek a more open forward heading.
            turn_target = (
                FRONT_SLOW_M if self.backed else FRONT_CLEAR_M
            )

            if (
                now - self.since >= MIN_TURN_SECONDS
                and v.front > turn_target
            ):
                self.phase = "FORWARD"
                self.backed = False
                return "STOP: turn complete", 0, 0

            return (
                "TURN LEFT" if self.direction > 0 else "TURN RIGHT",
                -self.direction * TURN_PWM,
                self.direction * TURN_PWM,
            )

        # Pause before deciding whether to turn or reverse.
        if self.phase == "BRAKE":
            if now - self.since < BRAKE_SECONDS:
                return "STOP: braking", 0, 0

            if not self.backed and v.front > FRONT_CLEAR_M:
                self.phase = "FORWARD"
                return "STOP: path cleared", 0, 0

            if v.turn_clear and v.turn_seen:
                self.direction = (
                    1 if v.left >= v.right - 0.01 else -1
                )
                self.phase = "TURN"
                self.since = now

                return (
                    "TURN LEFT" if self.direction > 0 else "TURN RIGHT",
                    -self.direction * TURN_PWM,
                    self.direction * TURN_PWM,
                )

            if not v.turn_clear:
                if not v.rear_seen:
                    return "STOP: rear scan insufficient", 0, 0

                if v.rear_gap <= REAR_STOP_GAP_M:
                    return "STOP: obstacle behind robot", 0, 0

                self.phase = "REVERSE"
                self.since = now
                self.backed = True

                return "REVERSE", -REVERSE_PWM, -REVERSE_PWM

            return "STOP: side/rear scan insufficient for turning", 0, 0

        # Normal forward movement.
        if v.front <= FRONT_STOP_M:
            self.phase = "BRAKE"
            self.since = now
            self.backed = False
            return "STOP: obstacle ahead", 0, 0

        speed = (
            SLOW_PWM
            if v.front <= FRONT_SLOW_M
            else FORWARD_PWM
        )

        return (
            "SLOW" if v.front <= FRONT_SLOW_M else "FORWARD",
            speed,
            speed,
        )


def main():
    import serial
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan

    port = node = None
    rclpy.init(args=[])

    try:
        input(
            "Mop power disconnected; test area clear front/rear. "
            "Enter to start: "
        )

        port = serial.Serial(
            ESP32_PORT,
            115200,
            timeout=0.08,
            write_timeout=0.08,
            exclusive=True,
        )

        time.sleep(2)
        port.reset_input_buffer()
        port.write(b"PING\n")

        deadline = time.monotonic() + 2

        while time.monotonic() < deadline:
            reply = port.readline().strip()

            if reply == b"PONG robot_wheels_v1":
                break

            if reply.startswith(b"ERR"):
                raise RuntimeError(
                    reply.decode(errors="replace")
                )
        else:
            raise RuntimeError(
                "ESP32 did not reply PONG robot_wheels_v1"
            )

        print(
            "ESP32 connected. Stop: 16 cm forward from LiDAR. "
            "Ctrl+C stops.",
            flush=True,
        )

        class Robot(Node):
            def __init__(self):
                super().__init__("robot_move")

                self.view = None
                self.received = 0.0
                self.stamp = 0.0
                self.avoider = Avoider()
                self.started = time.monotonic()
                self.done = False
                self.last_state = ""
                self.last_log = 0.0

                qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                )

                self.create_subscription(
                    LaserScan, "/scan", self.on_scan, qos
                )
                self.create_timer(0.1, self.control)

            def on_scan(self, msg):
                stamp = (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec * 1e-9
                )
                age = (
                    self.get_clock().now().nanoseconds / 1e9
                    - stamp
                )

                if stamp <= self.stamp or not -0.1 <= age <= 0.5:
                    self.view = None
                    return

                self.stamp = stamp
                self.view = scan_view(msg)
                self.received = time.monotonic()

            def control(self):
                now = time.monotonic()

                state, left, right = self.avoider.step(
                    self.view,
                    now - self.received <= 0.35,
                    now,
                )

                if (
                    RUN_SECONDS > 0
                    and now - self.started >= RUN_SECONDS
                ):
                    state, left, right = "TEST FINISHED", 0, 0
                    self.done = True

                if self.avoider.fault:
                    self.done = True

                command = f"{left} {right} 0 0"
                packet = (command + "\n").encode()

                if port.write(packet) != len(packet):
                    raise RuntimeError("Incomplete serial write")

                reply = port.readline().strip()

                if reply != ("OK " + command).encode():
                    raise RuntimeError(
                        "ESP32 acknowledgement failed: "
                        + repr(reply)
                    )

                if (
                    state != self.last_state
                    or now - self.last_log >= 1
                ):
                    def fmt(value):
                        return (
                            f"{value:.2f}m"
                            if math.isfinite(value)
                            else "none"
                        )

                    front = (
                        fmt(self.view.front)
                        if self.view else "none"
                    )
                    rear = (
                        fmt(self.view.rear_gap)
                        if self.view else "none"
                    )

                    print(
                        f"{state} | front from LiDAR={front} | "
                        f"rear-edge gap={rear} | command={command}",
                        flush=True,
                    )

                    self.last_state = state
                    self.last_log = now

        node = Robot()

        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except (KeyboardInterrupt, ExternalShutdownException):
        print("Stopped.")

    finally:
        if port is not None:
            try:
                port.write(b"STOP\n")
            except Exception:
                pass
            port.close()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("STOP: " + str(error))