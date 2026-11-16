"""ROS 2 camera publisher and browser preview subscribed to compressed images."""

import argparse
import signal
import threading
import time
from array import array
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = b"""<!doctype html><html><head><meta name="viewport" content="width=device-width">
<title>Rover camera</title><style>body{background:#111;color:#eee;font:18px sans-serif;
margin:24px}img{max-width:100%;height:auto}a{color:#8cf}</style></head>
<body><h2>Rover live camera</h2><img src="/stream" alt="Live rover camera">
<p>Keep the driving terminal focused to use the keyboard controls.</p></body></html>"""


class FrameBuffer:
    """Latest image received by the ROS browser subscriber."""

    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.received = 0
        self.stopped = False

    def receive(self, message):
        with self.condition:
            self.frame = bytes(message.data)
            self.sequence += 1
            self.received = time.monotonic()
            self.condition.notify_all()

    def get_frame(self, previous=0):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence != previous or self.stopped, timeout=5)
            if self.stopped or self.sequence == previous or time.monotonic() - self.received > 5:
                raise RuntimeError("No recent ROS camera frames")
            return self.sequence, self.frame

    def close(self):
        with self.condition:
            self.stopped = True
            self.condition.notify_all()


def make_handler(camera):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGE)
                return
            if self.path not in ("/stream", "/snapshot.jpg"):
                self.send_error(404)
                return
            try:
                sequence, frame = camera.get_frame()
            except RuntimeError as exc:
                self.send_error(503, str(exc))
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-store")
            self.send_header(
                "Content-Type",
                "image/jpeg"
                if self.path == "/snapshot.jpg"
                else "multipart/x-mixed-replace; boundary=frame",
            )
            self.end_headers()
            try:
                if self.path == "/snapshot.jpg":
                    self.wfile.write(frame)
                    return
                while True:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode()
                        + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )
                    self.wfile.flush()
                    sequence, frame = camera.get_frame(sequence)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, RuntimeError):
                pass

        def log_message(self, *_):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15, choices=range(1, 31))
    parser.add_argument("--topic-prefix", default="/camera")
    parser.add_argument("--node-name", default="rover_camera")
    parser.add_argument("--frame-id", default="camera_optical_frame")
    parser.add_argument(
        "--preview-topic", default=None, help="Compressed ROS image topic shown in the browser"
    )
    args = parser.parse_args()
    prefix = args.topic_prefix.rstrip("/")
    args.preview_topic = args.preview_topic or prefix + "/image_raw/compressed"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init(args=[])
    capture = None
    server = None
    node = None
    frames = FrameBuffer()
    try:
        node = Node(args.node_name)
        raw_publisher = node.create_publisher(Image, prefix + "/image_raw", qos_profile_sensor_data)
        jpeg_publisher = node.create_publisher(
            CompressedImage, prefix + "/image_raw/compressed", qos_profile_sensor_data
        )
        node.create_subscription(
            CompressedImage, args.preview_topic, frames.receive, qos_profile_sensor_data
        )
        capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open camera {args.device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        def publish_frame():
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("Camera disconnected or failed to capture")
            raw = Image()
            raw.header.stamp = node.get_clock().now().to_msg()
            raw.header.frame_id = args.frame_id
            raw.height, raw.width = frame.shape[:2]
            raw.encoding = "bgr8"
            raw.is_bigendian = 0
            raw.step = raw.width * 3
            # ROS accepts a byte array directly; bytes trigger slow per-element validation.
            raw.data = array("B", frame.tobytes())
            raw_publisher.publish(raw)
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            compressed = CompressedImage()
            compressed.header = raw.header
            compressed.format = "bgr8; jpeg compressed bgr8"
            compressed.data = array("B", jpg.tobytes())
            jpeg_publisher.publish(compressed)

        node.create_timer(1 / args.fps, publish_frame)
        server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(frames))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(
            f"Publishing {prefix}/image_raw and {prefix}/image_raw/compressed; "
            f"ROS browser preview of {args.preview_topic} at http://127.0.0.1:{args.port}"
        )
        rclpy.spin(node)
    finally:
        frames.close()
        if server:
            server.shutdown()
            server.server_close()
        if capture:
            capture.release()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    def terminate(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        main()
    except KeyboardInterrupt:
        print("Camera stopped.")
