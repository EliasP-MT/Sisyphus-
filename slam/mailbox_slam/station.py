"""Read-only bridge for NormaCore's decoded ST3215 InferenceStateReader.

No connection or command is opened here. Reuse your station subscription and
pass a decoded frame plus a timestamp aligned to the image clock.
"""

import struct


def measured_positions(frame):
    positions = {}
    for bus in frame.get_buses() or []:
        descriptor = bus.get_bus()
        serial = descriptor.get_serial_number() if descriptor is not None else ""
        for motor in bus.get_motors() or []:
            state = bytes(motor.get_state() or b"")
            if not serial or len(state) < 0x3A:
                continue
            raw = struct.unpack_from("<H", state, 0x38)[0]
            # Same sign-bit convention as teleop.parse_motor; never goal position.
            position = ((4096 - (raw & 0x0FFF)) & 0x0FFF) if raw & 0x8000 else raw & 0x0FFF
            positions[(serial, motor.get_id())] = position
    return positions


def feed_station_frame(slam, frame, timestamp, *, joint_sources, wheel_sources=None):
    """Sources map configured names to (bus_serial, motor_id).

    If wheel feedback comes from another driver, omit wheel_sources and call
    add_wheel_sample separately. timestamp must be capture-aligned; network
    arrival time is NOT assumed to be a hardware timestamp.
    """
    positions = measured_positions(frame)

    def select(sources):
        selected = {}
        for name, source in sources.items():
            source = tuple(source)
            if source not in positions:
                raise ValueError(f"Missing measured station encoder for {name}: {source}")
            selected[str(name)] = positions[source]
        return selected

    joints = select(joint_sources)
    wheels = select(wheel_sources) if wheel_sources is not None else None
    # Preselect both buses before modifying the SLAM encoder state.
    slam.add_joint_sample(timestamp, joints)
    if wheels is not None:
        slam.add_wheel_sample(timestamp, wheels)
