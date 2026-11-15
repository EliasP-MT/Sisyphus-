#!/usr/bin/env python3
"""Webcam hand -> ST3215 joints through the official Station Python client."""
import argparse
import asyncio
import logging
import math
import sys
import time
from pathlib import Path


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def targets(hand):
    """Normalized image coordinates and pinch aperture (scale independent)."""
    wrist, thumb, index, knuckle = (hand[i] for i in (0, 4, 8, 9))
    scale = max(math.hypot(wrist.x-knuckle.x, wrist.y-knuckle.y), 0.01)
    pinch = math.hypot(thumb.x-index.x, thumb.y-index.y) / scale
    return [clamp(knuckle.x, 0, 1), clamp(1-knuckle.y, 0, 1),
            clamp((pinch-0.15)/1.1, 0, 1)]


def bounded_goal(position, signal, anchor_signal, anchor_position, lo, hi, dt):
    desired = clamp(anchor_position + (signal-anchor_signal)*300, lo, hi)
    step = 100 * min(dt, 0.1)  # encoder ticks/s, bounded after a delayed frame
    return clamp(desired, position-step, position+step)


class Station:
    def __init__(self, client, pb, commands, drivers, send, bus, ids):
        self.client, self.pb, self.commands, self.drivers, self.send = client, pb, commands, drivers, send
        self.bus, self.ids = bus, ids
        self.motors, self.received, self.entry_id = {}, 0, None
        self.queue = asyncio.Queue()
        self.errors = client.follow('st3215/inference', self.queue)

    async def read(self):
        while True:
            if not self.errors.empty():
                raise RuntimeError(str(self.errors.get_nowait()))
            entry = await asyncio.wait_for(self.queue.get(), 3)
            if entry is None:
                raise RuntimeError('Flux Station fermé')
            entry_id = bytes(entry.ID.ID)
            if entry_id == self.entry_id:
                continue
            self.entry_id = entry_id
            frame = self.pb.InferenceStateReader(entry.Data)
            buses = [b for b in frame.get_buses() or [] if b.get_bus()]
            if self.bus == 'auto':
                if len(buses) != 1:
                    self.motors = {}
                    continue
                self.bus = buses[0].get_bus().get_serial_number()
            self.motors = {}
            for bus in buses:
                if bus.get_bus().get_serial_number() == self.bus:
                    self.motors = {m.get_id(): m for m in bus.get_motors() or []}
            self.received = time.monotonic()

    def state(self):
        if time.monotonic()-self.received > 0.5:
            raise RuntimeError('État Station absent ou périmé')
        result = []
        for mid in self.ids:
            if mid not in self.motors:
                raise RuntimeError(f'Moteur {mid} absent (détectés : {list(self.motors)})')
            m = self.motors[mid]
            raw = bytes(m.get_state())
            if not 0 <= time.time_ns()-m.get_system_stamp_ns() <= 500_000_000:
                raise RuntimeError(f'Moteur {mid} : mesure périmée')
            lo, hi = m.get_range_min(), m.get_range_max()
            if not m.get_range_freezed() or not 0 <= lo < hi <= 4095:
                raise RuntimeError(f'Moteur {mid} : calibration figée requise dans Station')
            error = m.get_error()
            if (len(raw) < 0x42 or raw[0x41] or error.get_description()
                    or error.get_servo() or error.get_command_packet()):
                raise RuntimeError(f'Moteur {mid} : état incomplet ou erreur servo')
            value = int.from_bytes(raw[0x38:0x3a], 'little')
            pos = (- (value & 4095) if value & 0x8000 else value) & 4095
            if not lo <= pos <= hi:
                raise RuntimeError(f'Moteur {mid} hors de sa plage calibrée')
            result.append((pos, lo, hi))
        return result

    async def write(self, address, values, size):
        message = self.pb.Command(target_bus_serial=self.bus,
            sync_write=self.pb.ST3215SyncWriteCommand(address=address, motors=[
                self.pb.ST3215SyncWriteCommand_MotorWrite(motor_id=mid,
                    value=int(value).to_bytes(size, 'little'))
                for mid, value in zip(self.ids, values)]))
        cmd = self.commands.DriverCommand(
            type=self.drivers.StationCommandType.STC_ST3215_COMMAND, body=message.encode())
        await asyncio.wait_for(self.send(self.client, [cmd]), 0.5)


async def station_camera(args):
    """Read Station's existing USB capture, including VMware cameras without V4L2."""
    import cv2
    import numpy as np
    sys.path.insert(0, str(Path(args.sdk).resolve()))
    from software.station.shared.station_py import new_station_client
    from target.gen_python.protobuf.drivers.usbvideo import usbvideo
    client = await asyncio.wait_for(new_station_client(args.host, logging.getLogger('camera')), 10)
    queue = asyncio.Queue()
    errors = client.follow(args.camera_queue, queue)

    class Camera:
        image = None
        stamp = 0

        def isOpened(self):
            return self.image is not None

        def read(self):
            if worker.done():
                worker.result()
            if time.monotonic()-self.stamp > .5:
                raise RuntimeError('Flux webcam Station périmé')
            return True, self.image.copy()

        def release(self):
            worker.cancel()
            client.close_connection()

    camera = Camera()

    async def receive():
        last_id = None
        while True:
            if not errors.empty():
                raise RuntimeError(str(errors.get_nowait()))
            entry = await asyncio.wait_for(queue.get(), 3)
            if entry is None:
                raise RuntimeError('Flux caméra fermé')
            entry_id = bytes(entry.ID.ID)
            if entry_id == last_id:
                continue
            last_id = entry_id
            pack = usbvideo.RxEnvelopeReader(entry.Data).get_frames()
            stamps = pack.get_stamps() or []
            if not stamps or not 0 <= time.time_ns()-stamps[-1].get_local_stamp_ns() <= 500_000_000:
                continue
            fmt = pack.get_format()
            if fmt.get_kind() == 1:
                data = pack.get_frames_data() or []
                if not data:
                    continue
                image = cv2.imdecode(np.frombuffer(data[-1], np.uint8), cv2.IMREAD_COLOR)
            else:
                size = fmt.get_width()*fmt.get_height()*3
                if size <= 0:
                    continue
                data = pack.get_linear_data()
                if len(data) < size:
                    continue
                rgb = np.frombuffer(data[-size:], np.uint8).reshape(3, fmt.get_height(), fmt.get_width())
                image = cv2.cvtColor(rgb.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
            if image is not None:
                camera.image, camera.stamp = image, time.monotonic()

    worker = asyncio.create_task(receive())
    try:
        for _ in range(100):
            if worker.done():
                worker.result()
            if camera.isOpened():
                return camera
            await asyncio.sleep(.05)
        raise RuntimeError('Aucune image récente sur la queue caméra Station')
    except BaseException:
        camera.release()
        await asyncio.gather(worker, return_exceptions=True)
        raise


async def run(args):
    import cv2
    import mediapipe as mp
    station = task = None
    armed = False
    cap = (await station_camera(args) if args.camera_queue else
           cv2.VideoCapture(int(args.camera) if args.camera.isdigit() else args.camera))
    if not cap.isOpened():
        raise RuntimeError('Webcam inaccessible. Vérifie son transfert USB à la VM et ferme les autres captures.')
    try:
        if args.live:
            sys.path.insert(0, str(Path(args.sdk).resolve()))
            from software.station.shared.station_py import new_station_client, send_commands
            from target.gen_python.protobuf.drivers.st3215 import st3215
            from target.gen_python.protobuf.station import commands, drivers
            client = await asyncio.wait_for(new_station_client(args.host, logging.getLogger('station')), 10)
            station = Station(client, st3215, commands, drivers, send_commands, args.bus,
                              [args.base, args.shoulder, args.gripper])
            task = asyncio.create_task(station.read())
        status = 'APERÇU' if not args.live else 'PAUSE : ESPACE pour armer'
        previous, anchors = None, None
        last = time.monotonic()
        with mp.solutions.hands.Hands(max_num_hands=1, min_detection_confidence=0.7,
                                     min_tracking_confidence=0.7) as detector:
            while True:
                ok, frame = await asyncio.to_thread(cap.read)
                if not ok:
                    raise RuntimeError('Capture webcam interrompue')
                frame = cv2.flip(frame, 1)
                detection = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                hand = detection.multi_hand_landmarks[0] if detection.multi_hand_landmarks else None
                signal = targets(hand.landmark) if hand else None
                now = time.monotonic()
                dt, last = now-last, now
                if hand:
                    mp.solutions.drawing_utils.draw_landmarks(frame, hand, mp.solutions.hands.HAND_CONNECTIONS)
                if task and task.done():
                    task.result()
                if station:
                    try:
                        states = station.state()
                        if armed and signal is None:
                            raise RuntimeError('Main perdue : réarmement requis')
                        if armed:
                            # Relative to the hand and arm poses captured at arming: no initial jump.
                            previous = [bounded_goal(p, s, a, start, lo, hi, dt)
                                for p, s, a, (start, _, _), (_, lo, hi) in
                                zip(previous, signal, anchors, initial, states)]
                            await station.write(0x2a, [round(p) for p in previous], 2)
                            status = 'ACTIF : ESPACE pour pause'
                    except RuntimeError as exc:
                        if armed:
                            # Hold the last target; do not release a gravity-loaded arm.
                            armed = False
                        status = str(exc)
                cv2.putText(frame, status.encode('ascii', 'replace').decode(), (12, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
                cv2.putText(frame, 'Q / ESC: quitter | ESPACE: pause / armer', (12, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                if signal is not None:
                    cv2.putText(frame, f'X={signal[0]:.2f} Y={signal[1]:.2f} pince={signal[2]:.2f}',
                                (12, 80), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                cv2.imshow('Main -> bras (Station)', frame)
                key = cv2.waitKey(1) & 255
                if key in (ord('q'), 27):
                    break
                if key == 32 and station:
                    if armed:
                        armed, status = False, 'PAUSE'
                    elif signal is not None:
                        try:
                            initial = station.state()
                            previous, anchors = [s[0] for s in initial], signal[:]
                            await station.write(0x2e, [100]*3, 2)
                            await station.write(0x29, [10]*3, 1)
                            await station.write(0x2a, previous, 2)
                            await station.write(0x28, [1]*3, 1)
                            armed = True
                            last = time.monotonic()
                        except RuntimeError as exc:
                            status = str(exc)
                await asyncio.sleep(0.04)
    finally:
        # Keep torque/last goal on exit so a raised arm does not fall.
        if task:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', default='0', help='Index OpenCV ou chemin /dev/videoN')
    parser.add_argument('--camera-queue', help='Queue USB Station, alternative à --camera')
    parser.add_argument('--live', action='store_true', help='Autorise les commandes après ESPACE')
    parser.add_argument('--sdk', default=str(Path(__file__).parent / '.sdk'))
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--bus', default='auto')
    parser.add_argument('--base', type=int, default=1)
    parser.add_argument('--shoulder', type=int, default=2)
    parser.add_argument('--gripper', type=int, default=6)
    args = parser.parse_args()
    ids = [args.base, args.shoulder, args.gripper]
    if len(set(ids)) != 3 or any(not 1 <= mid <= 253 for mid in ids):
        parser.error('Choisis trois IDs moteurs distincts entre 1 et 253')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ImportError, OSError, asyncio.TimeoutError) as exc:
        parser.exit(1, f'Erreur : {exc}\n')


if __name__ == '__main__':
    main()
