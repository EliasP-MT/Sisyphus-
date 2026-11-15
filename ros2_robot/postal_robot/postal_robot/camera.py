"""One USB camera per process, opened by its persistent /dev/v4l/by-id path.

Publishes sensor_msgs/CompressedImage (JPEG). On read failure the device is
released and reopened with a back-off; no stale frame is republished.
"""
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class Camera(Node):
    def __init__(self):
        super().__init__('camera', namespace='robot')
        for name, value in [('device', ''), ('frame_id', 'camera_optical_frame'), ('width', 640),
                            ('height', 480), ('fps', 5.), ('jpeg_quality', 60), ('fourcc', 'MJPG')]:
            self.declare_parameter(name, value)
        g = lambda n: self.get_parameter(n).value
        self.device, self.frame_id = g('device'), g('frame_id')
        if not self.device:
            raise ValueError('device parameter required (use /dev/v4l/by-id/...)')
        self.size, self.quality, self.fourcc = (g('width'), g('height')), g('jpeg_quality'), g('fourcc')
        self.capture, self.retry_at, self.failures, self.frames, self.last_frame = None, 0., 0, 0, None
        self.publisher = self.create_publisher(CompressedImage, 'image/compressed', qos_profile_sensor_data)
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1. / max(.5, min(30., g('fps'))), self.poll)
        self.create_timer(2., self.report)

    def _open(self):
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self._release(f'cannot open {self.device}')
            return False
        if self.fourcc:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_logger().info(f'Opened {self.device}')
        return True

    def _release(self, reason):
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.failures += 1
        self.retry_at = time.monotonic() + min(10., .5 * 2 ** min(self.failures, 5))
        self.get_logger().error(f'Camera unavailable: {reason}; retrying', throttle_duration_sec=5)

    def poll(self):
        if self.capture is None:
            if time.monotonic() < self.retry_at or not self._open():
                return
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self._release('frame read failed')
            return
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(self.quality)])
        if not ok:
            return
        self.failures, self.frames, self.last_frame = 0, self.frames + 1, time.monotonic()
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format, msg.data = 'jpeg', encoded.tobytes()
        self.publisher.publish(msg)

    def report(self):
        age = time.monotonic() - self.last_frame if self.last_frame else -1.
        healthy = self.capture is not None and 0 <= age < 2
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(DiagnosticStatus(
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
            name=f'postal/camera{self.get_namespace()}', hardware_id=self.device,
            message='streaming' if healthy else 'no recent frame',
            values=[KeyValue(key='frames', value=str(self.frames)), KeyValue(key='age_s', value=f'{age:.2f}')]))
        self.diag.publish(array)


def main():
    rclpy.init()
    node = None
    try:
        node = Camera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.capture is not None:
                node.capture.release()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
