"""One USB camera per process, using persistent /dev/v4l/by-id paths."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Camera(Node):
    def __init__(self):
        super().__init__('camera', namespace='robot')
        self.declare_parameter('device', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.capture = cv2.VideoCapture(self.get_parameter('device').value, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError('Camera unavailable')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.publisher = self.create_publisher(CompressedImage, 'image/compressed', qos_profile_sensor_data)
        self.timer = self.create_timer(.1, self.poll)

    def poll(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().error('Camera frame unavailable', throttle_duration_sec=2)
            return
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value
            msg.format, msg.data = 'jpeg', encoded.tobytes()
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Camera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.capture.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
