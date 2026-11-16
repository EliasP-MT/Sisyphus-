"""ROS 2 rover controller for the RDK using system Python."""

import argparse
import curses
import math
import signal
import time

from base_geometry import BASE_RADIUS, WHEEL_ANGLES_DEG, WHEEL_RADIUS
from rover import Bus

SPEED = 0.5  # m/s requested; MAX_RAW also limits actual speed.
TURN_SPEED = 30  # degrees/s
MAX_RAW = 2000  # Maximum wheel speed in encoder steps per second.
MOTORS = (7, 8, 9)  # left, back, right


def wheel_commands(x, y, angular):
    """LeKiwi kinematics: x/y in m/s, angular in rad/s; 5 cm wheel radius."""
    if not all(math.isfinite(v) for v in (x, y, angular)):
        raise ValueError("Velocity must be finite")
    values = [
        (math.cos(math.radians(a)) * x + math.sin(math.radians(a)) * y + BASE_RADIUS * angular)
        / WHEEL_RADIUS
        * 4096
        / (2 * math.pi)
        for a in WHEEL_ANGLES_DEG
    ]
    scale = min(1, MAX_RAW / max(max(map(abs, values)), 1))
    return [round(v * scale) for v in values]


class Controller:
    def __init__(self, port):
        self.bus = Bus(port)
        self.modes = {}
        self.started = False

    def check(self):
        for motor in MOTORS:
            model = self.bus.read(motor, 3, 2)
            mode = self.bus.read(motor, 33, 1)
            voltage = self.bus.read(motor, 62, 1) / 10
            print(f"ID {motor}: model={model}, mode={mode}, voltage={voltage:.1f} V")
            if model != 777 or mode not in (0, 1):
                raise RuntimeError(f"Unexpected motor model/mode for ID {motor}")
            self.modes[motor] = mode

    def start(self):
        self.check()
        self.started = True
        for motor in MOTORS:
            self.bus.write(motor, 40, 0)
            self.bus.write(motor, 46, 0, 2)
            self.bus.write(motor, 33, 1)
        for motor in MOTORS:
            self.bus.write(motor, 40, 1)

    def move(self, x=0, y=0, angular=0):
        for motor, value in zip(MOTORS, wheel_commands(x, y, angular)):
            encoded = abs(value) | (0x8000 if value < 0 else 0)
            self.bus.write(motor, 46, encoded, 2)

    def close(self):
        errors = []
        if self.started:
            for motor in MOTORS:
                for address, value, size in ((46, 0, 2), (40, 0, 1)):
                    try:
                        self.bus.write(motor, address, value, size)
                    except Exception as exc:
                        errors.append(str(exc))
                if motor in self.modes:
                    try:
                        self.bus.write(motor, 33, self.modes[motor])
                    except Exception as exc:
                        errors.append(str(exc))
        self.bus.serial.close()
        if errors:
            raise RuntimeError("Stop/restore failed; switch off motor power: " + "; ".join(errors))


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--check", action="store_true", help="Read hardware only; no movement")
    parser.add_argument(
        "--keyboard", action="store_true", help="AZERTY control in this SSH terminal"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Test ROS commands without opening hardware"
    )
    parser.add_argument(
        "--odom",
        action="store_true",
        help="Read wheel encoders and publish /odom and odom->base_link",
    )
    parser.add_argument("--seconds", type=int, default=60, choices=range(1, 601))
    args = parser.parse_args()
    if args.check and args.dry_run:
        parser.error("--check requires hardware; omit --dry-run")
    controller = None
    node = None
    rclpy.init(args=[])
    try:
        if not args.dry_run:
            controller = Controller(args.port)
        if args.check:
            controller.check()
            return
        node = Node("rover_base")
        odometry = None
        if args.odom and controller:
            from geometry_msgs.msg import TransformStamped
            from nav_msgs.msg import Odometry
            from tf2_ros import TransformBroadcaster

            from base_geometry import WheelOdometry

            odometry = WheelOdometry()
            odom_publisher = node.create_publisher(Odometry, "/odom", 1)
            transform_publisher = TransformBroadcaster(node)
        last_odom = 0

        def publish_odometry():
            positions = [controller.bus.read(motor, 56, 2) for motor in MOTORS]
            velocity = odometry.update(positions, time.monotonic())
            if velocity is None:
                return
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "odom"
            message.child_frame_id = "base_link"
            message.pose.pose.position.x, message.pose.pose.position.y = odometry.x, odometry.y
            message.pose.pose.orientation.z = math.sin(odometry.yaw / 2)
            message.pose.pose.orientation.w = math.cos(odometry.yaw / 2)
            (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            ) = velocity
            # Provisional uncertainty, pending measured wheel/slip calibration.
            for i, variance in enumerate((0.05, 0.05, 1e6, 1e6, 1e6, 0.1)):
                message.pose.covariance[i * 7] = variance
                message.twist.covariance[i * 7] = variance
            odom_publisher.publish(message)
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = "base_link"
            transform.transform.translation.x, transform.transform.translation.y = (
                odometry.x,
                odometry.y,
            )
            transform.transform.rotation = message.pose.pose.orientation
            transform_publisher.sendTransform(transform)

        command = [0.0, 0.0, 0.0]
        expires = 0.0

        def receive(msg):
            nonlocal expires
            values = (msg.linear.x, msg.linear.y, msg.angular.z)
            if not all(math.isfinite(v) for v in values):
                return
            command[:] = [max(-SPEED, min(SPEED, v)) for v in values[:2]] + [
                max(-math.radians(TURN_SPEED), min(math.radians(TURN_SPEED), values[2]))
            ]
            expires = time.monotonic() + 0.2
            if args.dry_run:
                node.get_logger().info(
                    f"cmd_vel={tuple(command)}, wheels={wheel_commands(*command)}"
                )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        node.create_subscription(Twist, "/cmd_vel", receive, qos)
        publisher = node.create_publisher(Twist, "/cmd_vel", 1) if args.keyboard else None
        if controller:
            controller.start()
        node.get_logger().info(
            "Ready on /cmd_vel; timeout 0.2 s" + (" (NO HARDWARE)" if args.dry_run else "")
        )

        def run(screen=None):
            nonlocal last_odom
            if screen:
                screen.nodelay(True)
                screen.addstr("Z/S forward/back | Q/D sideways | A/E turn | SPACE stop | X quit\n")
                screen.addstr("Repeat a key to continue; commands expire after 0.2 seconds.\n")
                screen.refresh()
            keys = {
                "z": (SPEED, 0, 0),
                "s": (-SPEED, 0, 0),
                "q": (0, SPEED, 0),
                "d": (0, -SPEED, 0),
                "a": (0, 0, math.radians(TURN_SPEED)),
                "e": (0, 0, -math.radians(TURN_SPEED)),
            }
            deadline = time.monotonic() + args.seconds
            while rclpy.ok() and time.monotonic() < deadline:
                if screen:
                    key = screen.getch()
                    if key in (ord("x"), ord("X")):
                        break
                    if 0 <= key < 256:
                        values = keys.get(chr(key).lower(), (0, 0, 0))
                        msg = Twist()
                        msg.linear.x, msg.linear.y, msg.angular.z = map(float, values)
                        publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.01)
                if controller:
                    controller.move(*(command if time.monotonic() < expires else (0, 0, 0)))
                    if odometry and time.monotonic() - last_odom >= 0.1:
                        publish_odometry()
                        last_odom = time.monotonic()
                time.sleep(0.015)

        if args.keyboard:
            curses.wrapper(run)
        else:
            run()
    finally:
        try:
            if controller:
                controller.close()
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
