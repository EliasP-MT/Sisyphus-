"""Read-only hardware feedback node; one exclusive owner of the serial bus."""
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .bus import Bus, BusError


class Feedback(Node):
    def __init__(self):
        super().__init__('motor_feedback', namespace='robot')
        self.declare_parameter('device', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E114718-if00')
        self.declare_parameter('calibration', '')
        config = json.loads(Path(self.get_parameter('calibration').value).read_text())
        self.joints = {j['motor_id']: j for j in config['joints']}
        if set(self.joints) != set(range(1, 8)):
            raise ValueError('Expected verified seven-joint mapping')
        if config.get('hardware_verified') is not True:
            raise ValueError('Verified calibration required')
        self.bus = Bus(self.get_parameter('device').value)
        self.positions = self.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
        self.health = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.timer = self.create_timer(.1, self.poll)

    def poll(self):
        diagnostics = DiagnosticArray()
        for motor_id in range(1, 13):
            status = DiagnosticStatus(name=f'motor/{motor_id}', hardware_id=str(motor_id))
            try:
                raw, received = self.bus.read(motor_id, 56, 15)
                position = int.from_bytes(raw[:2], 'little')
                fault = raw[9]  # Address 65 / 0x41, not asynchronous-write flag.
                status.level = DiagnosticStatus.ERROR if fault else DiagnosticStatus.OK
                status.message = f'servo error 0x{fault:02x}' if fault else 'fresh encoder'
                status.values = [KeyValue(key='position_ticks', value=str(position)),
                                 KeyValue(key='received_monotonic', value=str(received)),
                                 KeyValue(key='voltage_V', value=str(raw[6] / 10)),
                                 KeyValue(key='temperature_C', value=str(raw[7]))]
                if motor_id in self.joints and not fault:
                    j = self.joints[motor_id]
                    if not 0 <= position <= 4095:
                        raise BusError('Invalid absolute encoder')
                    q = (position-j['zero_ticks'])*j['sign']/j.get('ticks_per_rad', 4096/(2*math.pi))
                    message = JointState()
                    message.header.stamp = self.get_clock().now().to_msg()
                    message.name, message.position = [j['name']], [q]
                    self.positions.publish(message)
            except (BusError, OSError) as exc:
                status.level, status.message = DiagnosticStatus.ERROR, str(exc)
            diagnostics.status.append(status)
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        self.health.publish(diagnostics)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Feedback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
