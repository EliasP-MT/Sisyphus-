"""Live ROS 2 bridge to main's postal_robot runtime. No direct hardware ownership."""

import argparse
import base64
import json
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from mailbox_slam import MailboxDiscovery, MailboxSurvey, SurveySettings
from mailbox_slam.live_health import geometry_warnings, readiness_blockers
from mailbox_slam.mapping import json_value, write_json
from mailbox_slam.robot_model import Telemetry, load_robot_config

ROOT = Path(__file__).resolve().parent


def slim(box):
    """Only what the panel needs from a mailbox record."""
    keep = ("id", "name", "position_xyz_m", "outward_normal_xyz", "observations", "identity", "T_map_tag")
    return {k: box[k] for k in keep if k in box}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-yaml", type=Path, default=ROOT.parent / "ros2_robot/postal_robot/config/robot.yaml"
    )
    parser.add_argument(
        "--calibration", type=Path, default=Path.home() / "postal_config/calibration.local.json"
    )
    parser.add_argument("--urdf", type=Path, default=Path.home() / "postal_config/elrobot.urdf")
    parser.add_argument(
        "--intrinsics", type=Path, default=ROOT / "config/arm-camera-intrinsics.json"
    )
    parser.add_argument("--base-link", default="base_footprint")
    parser.add_argument(
        "--camera-link",
        default="camera_2_optical_frame",
        help="Arm camera optical frame (camera 2)",
    )
    parser.add_argument(
        "--camera-topic",
        default="/robot/camera_2/image/compressed",
        help="Arm camera compressed topic (camera 2, confirmed by operator)",
    )
    parser.add_argument(
        "--camera-mount",
        type=Path,
        help="Measured {parent_link,T_parent_camera} JSON if camera is absent from URDF",
    )
    parser.add_argument(
        "--arm-base",
        type=Path,
        help="JSON {T_base_arm}: URDF base link pose in the robot Kiwi frame (default identity)",
    )
    parser.add_argument(
        "--execute", action="store_true", help="Publish base commands; otherwise proposals only"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / "postal_config/mailbox-map.local.json",
        help="Persistent mailbox map (kept outside the source tree, survives redeploys)",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Stay idle; a run starts/stops only on /slam/command (station panel)",
    )
    parser.add_argument(
        "--preflight",
        action="store_true",
        help="Validate model and print geometry without ROS or motion",
    )
    parser.add_argument(
        "--hardcode",
        type=int,
        metavar="N",
        help="Full 360 discovery only: require exactly N distinct mailboxes and export map; no approaches/OCR",
    )
    parser.add_argument(
        "--scan-only",
        action="store_true",
        help="Full 360 geometry scan and map export without approach/OCR",
    )
    args, ros_args = parser.parse_known_args()
    if args.hardcode is not None and args.hardcode <= 0:
        parser.error("--hardcode must be a positive integer")
    if args.base_link != "base_footprint" and args.arm_base is None:
        parser.error(
            "--arm-base is required when the URDF base is not the Kiwi base_footprint; identity would silently misplace the map"
        )
    mount = json.loads(args.camera_mount.read_text()) if args.camera_mount else None
    arm_base = json.loads(args.arm_base.read_text()) if args.arm_base else None
    config, params = load_robot_config(
        args.robot_yaml,
        args.calibration,
        args.urdf,
        args.intrinsics,
        base_link=args.base_link,
        camera_link=args.camera_link,
        camera_mount=mount,
        arm_base=arm_base,
    )
    from mailbox_slam.config import validate

    validate(config)
    warnings = geometry_warnings(json.loads(args.intrinsics.read_text()), mount, arm_base)
    if args.preflight:
        print(
            json.dumps(
                dict(
                    warnings=warnings,
                    T_base_arm=config["arm"]["T_base_arm"],
                    T_last_camera=config["arm"]["T_last_camera"],
                    wheel_ids=[w["name"] for w in config["wheels"]],
                    camera_topic=args.camera_topic,
                    camera_frame=args.camera_link,
                ),
                indent=2,
            )
        )
        return
    if args.execute and not params.get("base_direction_verified", False):
        parser.error("Main robot configuration does not verify base direction")
    import rclpy
    from geometry_msgs.msg import Twist
    from postal_robot_interfaces.msg import MotorStateArray, SupervisorState
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import CompressedImage, JointState
    from std_msgs.msg import String

    class LiveSLAM(Node):
        def __init__(self):
            super().__init__("mailbox_slam", namespace="robot")
            self.lock = threading.Lock()
            self.execute = args.execute
            self.active = False
            self.final_zero = False
            self.cancel_run = threading.Event()
            self.run_mode = "idle"
            self.stored_map = self.load_stored_map()
            # Names are read by the PC station (no PaddleOCR on the RDK): see RemoteOCR.
            from mailbox_slam.remote_ocr import RemoteOCR

            self.remote_ocr = RemoteOCR(
                lambda payload: self.ocr_request_pub.publish(String(data=json.dumps(payload)))
            )
            self.new_run_state()
            self.photos = {
                k: dict(area=0.0, jpeg=v) for k, v in (self.stored_photos or {}).items()
            }
            self.photos_dirty = True
            self.events = deque()
            self.frame = None
            self.supervisor = None
            self.supervisor_received = 0.0
            self.camera_received = 0.0
            self.command = None
            self.failure = None
            self.runtime_verified = False
            self.parameter_request = None
            self.parameter_requested_at = None
            self.timing = {}
            self.status_sent, self.status_phase = 0.0, None
            self.stop = threading.Event()
            self.velocity = self.create_publisher(Twist, "base/cmd_vel", 10)
            self.create_subscription(String, "/slam/command", self.on_command, 10)
            self.ocr_request_pub = self.create_publisher(String, "/slam/ocr_request", 10)
            self.create_subscription(String, "/slam/ocr_result", self.on_ocr_result, 10)
            self.status_pub = self.create_publisher(String, "/slam/status", 10)
            self.photos_pub = self.create_publisher(
                String,
                "/slam/photos",
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            self.image_pub = self.create_publisher(
                CompressedImage, "/slam/image/compressed", qos_profile_sensor_data
            )
            # Deep queues: a long vision step must not drop 20 Hz encoder messages (a lost
            # sample is a gap in the odometry, which the integrator refuses).
            self.create_subscription(
                JointState,
                "joint_states",
                self.joints,
                QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(MotorStateArray, "motors", self.motors, 400)
            self.create_subscription(SupervisorState, "supervisor/state", self.state, 10)
            self.create_subscription(
                CompressedImage, args.camera_topic, self.image, qos_profile_sensor_data
            )
            self.create_timer(0.05, self.publish)
            from rcl_interfaces.srv import GetParameters

            self.parameter_client = self.create_client(GetParameters, "/robot/core/get_parameters")
            self.create_timer(0.5, self.verify_parameters)
            self.worker = None
            if not args.serve:
                self.start_run(args.execute)

        def load_stored_map(self):
            self.stored_photos = {}
            try:
                data = json.loads(args.output.read_text())
                self.stored_photos = data.get("photos", {})
                return dict(
                    saved_at=args.output.stat().st_mtime,
                    mailboxes=data.get("mailboxes", []),
                    initialization=data.get("initialization"),
                )
            except (OSError, ValueError):
                return None

        def new_run_state(self):
            self.mapping = MailboxDiscovery(config)
            self.mapping.locator.ocr = self.remote_ocr
            self.mapping.parallel_wall = True  # boxes are on the wall the base faces at the start
            self.survey = MailboxSurvey(
                self.mapping,
                SurveySettings(
                    discovery_only=args.scan_only or args.hardcode is not None,
                    expected_mailboxes=args.hardcode,
                    scan_mode="translate",  # start facing the wall, slide 1.5 m to the right
                    read_during_sweep=True,  # names are read on this first pass, at the start distance
                    second_pass_closer_m=0.0,  # unread boxes: second pass at the same distance
                    scan_translate_m=1.5,
                    scan_direction=(0.0, -1.0),
                    frontal_approach=True,
                    read_standoffs_m=(0.50, 0.35, 0.28),
                    max_read_attempts=6,
                ),
            )
            self.telemetry = Telemetry(config)
            self.events = deque()
            self.last_wheel_t, self.max_wheel_gap, self.max_loop_s = None, 0.0, 0.0
            self.last_joint_t, self.max_joint_gap = None, 0.0
            self.last_observe_s, self.max_observe_s, self.last_run_write = 0.0, 0.0, 0.0
            self.catch_up_since, self.catch_up_waits = None, 0
            self.command = None
            self.failure = None
            self.status = {"phase": "idle"}

        def start_run(self, execute):
            with self.lock:
                if self.active:
                    return False
                if execute and not params.get("base_direction_verified", False):
                    self.get_logger().error("Main robot configuration does not verify base direction")
                    self.status = {"phase": "refused", "reason": "base direction not verified"}
                    return False
                if self.worker is not None:
                    self.worker.join(timeout=2.0)
                self.new_run_state()
                self.execute = bool(execute)
                self.cancel_run.clear()
                self.final_zero = False
                self.active = True
                self.photos, self.photos_dirty = {}, True
                self.status = {"phase": "waiting_for_telemetry"}
            self.worker = threading.Thread(target=self.work, daemon=True)
            self.worker.start()
            self.get_logger().info(f"SLAM run started (execute={self.execute})")
            return True

        def on_ocr_result(self, message):
            try:
                self.remote_ocr.deliver(json.loads(message.data))
            except ValueError:
                self.get_logger().warn("Ignored malformed /slam/ocr_result")

        def on_command(self, message):
            try:
                command = json.loads(message.data)
                kind = command["cmd"]
            except (ValueError, KeyError, TypeError):
                self.get_logger().warn("Ignored malformed /slam/command")
                return
            if kind == "start":
                if not self.start_run(command.get("execute", False)):
                    self.get_logger().warn("Start ignored: a run is already active")
            elif kind == "stop":
                self.cancel_run.set()
            else:
                self.get_logger().warn(f"Ignored unknown /slam/command {kind!r}")

        def telemetry_caught_up(self, image_stamp):
            """After a long computation the encoder messages queue up and are delivered one per
            executor cycle, while the (single) image message arrives at once: the image can then
            look newer than every encoder sample received so far. Wait for the backlog instead of
            treating it as a telemetry loss; give up (and let the survey fault) after 5 s."""
            wheels = self.mapping.wheels.buffer.samples
            joints = self.mapping.arm.buffer.samples
            if wheels and joints and min(wheels[-1][0], joints[-1][0]) >= image_stamp - 0.05:
                self.catch_up_since = None
                return True
            now = time.monotonic()
            if self.catch_up_since is None:
                self.catch_up_since = now
            self.catch_up_waits += 1
            return now - self.catch_up_since > 5.0  # then the survey reports the real problem

        def collect_photos(self, image):
            """Keep, per box, the sharpest-sized view: label (green) and slot (red) contours
            drawn on the raw frame, cropped around the box front face."""
            from mailbox_slam.photos import box_photo

            locator = self.mapping.locator
            for key, result in self.mapping.last_associations.items():
                area = float(result["label_area_px2"])
                if key in self.photos and self.photos[key]["area"] >= area:
                    continue
                jpeg = box_photo(image, result, locator.calibration, locator.geometry)
                if jpeg:
                    self.photos[key] = dict(area=area, jpeg=jpeg)
                    self.photos_dirty = True

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

        def enqueue(self, event):
            with self.lock:
                if not self.active:
                    return
                if len(self.events) >= 5000:
                    self.failure = "Telemetry queue overflow"
                    return
                self.events.append(event)

        def joints(self, message):
            self.enqueue(
                ("joints", self.stamp(message), list(message.name), list(message.position))
            )

        def motors(self, message):
            self.enqueue(("wheels", self.stamp(message), list(message.motors)))

        def state(self, message):
            with self.lock:
                self.supervisor, self.supervisor_received = message, time.monotonic()

        def image(self, message):
            with self.lock:
                self.frame = message  # Latest only, never queue stale OCR frames.
                self.camera_received = time.monotonic()

        def verify_parameters(self):
            if self.runtime_verified or self.failure or self.parameter_request is not None:
                return
            if not self.parameter_client.service_is_ready():
                return
            from rcl_interfaces.srv import GetParameters

            keys = [
                "wheel_ids",
                "wheel_signs",
                "wheel_mount_deg",
                "wheel_radius_m",
                "base_radius_m",
                "base_direction_verified",
                "max_wheel_raw",
            ]
            self.parameter_requested_at = time.monotonic()
            self.parameter_request = self.parameter_client.call_async(
                GetParameters.Request(names=keys)
            )

            def complete(future):
                from rclpy.parameter import parameter_value_to_python

                try:
                    actual = [parameter_value_to_python(v) for v in future.result().values]
                    if len(actual) != len(keys) or any(
                        (list(value) if isinstance(params[key], list) else value) != params[key]
                        for key, value in zip(keys, actual)
                    ):
                        raise ValueError(
                            "Active robot wheel parameters differ from supplied robot YAML"
                        )
                    self.runtime_verified = True
                except Exception as exc:
                    self.failure = str(exc)

            self.parameter_request.add_done_callback(complete)

        def health(self):
            now = time.monotonic()
            return readiness_blockers(
                self.supervisor,
                runtime_verified=self.runtime_verified,
                supervisor_age_s=now - self.supervisor_received,
                camera_age_s=now - self.camera_received,
            )

        def allowed(self):
            return not self.health()

        def blockers(self):
            return ", ".join(self.health()) or "none"

        def publish(self):
            with self.lock:
                permitted = self.allowed() and time.monotonic() - self.camera_received < 1.0
                command, status = self.command, dict(self.status)
                failed = self.failure
                blockers = self.health()
                if (
                    self.parameter_requested_at is not None
                    and not self.runtime_verified
                    and time.monotonic() - self.parameter_requested_at > 5
                ):
                    self.failure = failed = (
                        "Robot parameter service timed out; check ROS discovery and restart"
                    )
            msg = Twist()
            if command and permitted and not failed and self.now() <= command["expires_at"]:
                msg.linear.x, msg.linear.y, msg.angular.z = command["velocity_base"]
            # Base commands only while an executing run is active (plus one final zero),
            # so an idle SLAM never competes with the station's own cmd_vel.
            if self.photos_dirty:
                self.photos_dirty = False
                self.photos_pub.publish(
                    String(data=json.dumps({k: v["jpeg"] for k, v in self.photos.items()}))
                )
            if (self.active and self.execute) or self.final_zero:
                self.velocity.publish(msg)
                self.final_zero = False
            status.update(
                execution_enabled=self.execute,
                active=self.active,
                stored_map=self.stored_map,
                supervisor_permits_motion=permitted,
                error=failed,
                blockers=blockers,
                geometry_warnings=warnings,
                camera_topic=args.camera_topic,
                timing=self.timing,
            )
            # Status is for display only: 2 Hz and slim (a 20 Hz stream of the full map
            # backs up on the Wi-Fi link and the panel then shows a stale state).
            if time.monotonic() - self.status_sent >= 0.5 or status.get("phase") != self.status_phase:
                self.status_sent, self.status_phase = time.monotonic(), status.get("phase")
                status["mailboxes"] = [slim(m) for m in status.get("mailboxes", [])]
                stored = status.get("stored_map")
                if stored:
                    status["stored_map"] = {**stored, "mailboxes": [slim(m) for m in stored["mailboxes"]]}
                    status["stored_map"].pop("initialization", None)
                self.status_pub.publish(String(data=json.dumps(status, default=json_value)))

        def work(self):
            run_path = args.output.with_name(args.output.stem + ".run.json")
            started = False
            try:
                loop_t = time.monotonic()
                while not self.stop.wait(0.02):
                    self.max_loop_s = max(self.max_loop_s, time.monotonic() - loop_t - 0.02)
                    loop_t = time.monotonic()
                    if self.cancel_run.is_set():
                        self.survey.phase = "stopped"
                        self.survey.reason = "stopped by the operator"
                        break
                    with self.lock:
                        events = list(self.events)
                        self.events.clear()
                        frame = self.frame
                        ready = self.allowed() and time.monotonic() - self.camera_received < 1.0
                        failed = self.failure
                    if failed:
                        raise ValueError(failed)
                    for event in events:
                        if event[0] == "joints":
                            sample = self.telemetry.joint_sample(event[1], event[2], event[3])
                            if sample:
                                if self.last_joint_t is not None:
                                    self.max_joint_gap = max(self.max_joint_gap, sample[0] - self.last_joint_t)
                                self.last_joint_t = sample[0]
                                self.mapping.add_joint_sample(*sample, units="radians")
                        else:
                            try:
                                sample = self.telemetry.wheel_sample(event[1], event[2])
                            except ValueError:
                                if started:
                                    raise
                                continue
                            if sample:
                                if self.last_wheel_t is not None:
                                    self.max_wheel_gap = max(self.max_wheel_gap, sample[0] - self.last_wheel_t)
                                self.last_wheel_t = sample[0]
                                self.mapping.add_wheel_sample(*sample)
                    now = self.now()
                    if frame is not None:
                        w = self.mapping.wheels.buffer.samples
                        j = self.mapping.arm.buffer.samples
                        self.timing = dict(
                            image_minus_wheel_s=self.stamp(frame) - w[-1][0] if w else None,
                            image_minus_joint_s=self.stamp(frame) - j[-1][0] if j else None,
                            now_minus_image_s=now - self.stamp(frame),
                            now_minus_wheel_s=now - w[-1][0] if w else None,
                            now_minus_joint_s=now - j[-1][0] if j else None,
                            max_wheel_gap_s=self.max_wheel_gap,
                            max_joint_gap_s=self.max_joint_gap,
                            last_observe_s=self.last_observe_s,
                            max_observe_s=self.max_observe_s,
                            catch_up_waits=self.catch_up_waits,
                            max_worker_loop_s=self.max_loop_s,
                        )
                    if not started:
                        if (
                            not ready
                            or not self.mapping.arm.buffer.samples
                            or not self.mapping.wheels.buffer.samples
                        ):
                            continue
                        self.survey.start(now)
                        started = True
                    elif not ready:
                        with self.lock:
                            why = self.blockers()
                        raise ValueError(
                            f"Supervisor permission or camera stream lost ({why}); "
                            "explicit restart required"
                        )
                    proposal = self.survey.step(now)
                    with self.lock:
                        self.command = proposal
                        self.status = {**self.survey.status(), "proposal": proposal}
                    if (
                        proposal["capture"]
                        and frame is not None
                        and self.stamp(frame) >= self.survey.phase_started
                        and self.telemetry_caught_up(self.stamp(frame))
                    ):
                        if frame.header.frame_id != args.camera_link:
                            raise ValueError(
                                "Camera frame_id does not match the configured optical frame"
                            )
                        image = cv2.imdecode(
                            np.frombuffer(bytes(frame.data), np.uint8), cv2.IMREAD_COLOR
                        )
                        if image is None:
                            raise ValueError("Cannot decode camera frame")
                        t_obs = time.monotonic()
                        self.survey.observe(image, self.stamp(frame))
                        self.last_observe_s = time.monotonic() - t_obs
                        self.max_observe_s = max(self.max_observe_s, self.last_observe_s)
                        self.collect_photos(image)
                        for result in self.mapping.last_associations.values():
                            image = self.mapping.locator.annotate(image, result)
                        ok, jpeg = cv2.imencode(".jpg", image)
                        if ok:
                            output = CompressedImage()
                            output.header = frame.header
                            output.format, output.data = "jpeg", jpeg.tobytes()
                            self.image_pub.publish(output)
                        if time.monotonic() - self.last_run_write > 10.0:  # the final write is in finally
                            self.last_run_write = time.monotonic()
                            write_json(
                                run_path,
                                {
                                    **self.mapping.map_payload(),
                                    "initialization": self.survey.status(),
                                    "photos": {k: v["jpeg"] for k, v in self.photos.items()},
                                },
                            )
                    if self.survey.phase in ("complete", "incomplete", "fault"):
                        break
            except Exception as exc:
                import traceback

                self.get_logger().error(traceback.format_exc())
                self.survey.phase = "fault"
                self.survey.reason = str(exc)
                with self.lock:
                    self.failure = str(exc)
            finally:
                with self.lock:
                    self.command = None
                    self.status = self.survey.status()
                if started:
                    payload = {
                        **self.mapping.map_payload(),
                        "initialization": self.survey.status(),
                        "runtime_error": self.failure,
                        "photos": {k: v["jpeg"] for k, v in self.photos.items()},
                    }
                    write_json(run_path, payload)
                    # Only a completed survey replaces the stored map (previous one is kept).
                    if self.survey.phase == "complete" and not self.failure:
                        if args.output.exists():
                            args.output.replace(
                                args.output.with_name(
                                    f"{args.output.stem}.bak-{time.strftime('%Y%m%d-%H%M%S')}.json"
                                )
                            )
                        write_json(args.output, payload)
                        self.stored_map = self.load_stored_map()
                with self.lock:
                    self.active = False
                    self.final_zero = self.execute

        def close(self):
            self.stop.set()
            try:
                self.velocity.publish(Twist())
            except Exception:  # context already shut down
                pass
            if self.worker is not None:
                self.worker.join(timeout=2.0)

    rclpy.init(args=ros_args)
    node = LiveSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
