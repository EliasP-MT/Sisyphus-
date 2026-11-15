#!/usr/bin/env python3
"""Leader -> follower teleoperation for two ST3215 arms on one NormaCore station.

The actual 50 Hz mirroring runs *inside the station* (its built-in
`motors-mirroring` driver, the same one the web UI dropdown uses). This tool:

  * shows which arm buses the station sees            (status)
  * figures out which arm is the leader by watching   (identify)
    which one you move, and remembers it in arms.json
  * safely starts mirroring: frees the leader, slowly  (start)
    brings the follower to the leader's pose, then
    hands over to the station's mirroring loop
  * stops mirroring                                    (stop)

Usage:
    python3 teleop.py status
    python3 teleop.py identify          # move the LEADER arm while it watches
    python3 teleop.py start             # Ctrl+C stops (follower goes limp!)
    python3 teleop.py start --detach    # leave mirroring running in the station
    python3 teleop.py stop
    python3 teleop.py spin --bus BUS_SERIAL --speed 500

Only the Python standard library is needed. The station must run with --tcp
(port 8888); the web UI on :8889 keeps working alongside this tool.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "vendor"))

from station_py import new_station_client, send_commands  # noqa: E402
from target.gen_python.protobuf.station import commands as station_commands, drivers  # noqa: E402
from target.gen_python.protobuf.drivers.st3215 import st3215  # noqa: E402
from target.gen_python.protobuf.drivers.motors_mirroring import mirroring  # noqa: E402

ARMS_FILE = HERE / "arms.json"

ST3215_QUEUE = "st3215/inference"
MIRRORING_QUEUE = "inference/mirroring"

# ST3215 RAM registers (offsets inside motor.get_state()).
REG_TORQUE_ENABLE = 0x28
REG_ACC = 0x29
REG_GOAL_POSITION = 0x2A
REG_GOAL_SPEED = 0x2E
REG_PRESENT_POSITION = 0x38
REG_STATUS = 0x40
REG_PRESENT_CURRENT = 0x45
REG_OPERATING_MODE = 0x21       # EEPROM: 0=position, 1=wheel
EXTRA_SERVO_ID = 10

MAX_STEPS = 4096
# Must match the station's mirroring (software/drivers/motors-mirroring/src/config.rs)
# so the soft-start pose equals the pose mirroring will hold.
SAFETY_MARGIN = 20

log = logging.getLogger("teleop")


# --------------------------------------------------------------------------- #
# Motor state parsing
# --------------------------------------------------------------------------- #

@dataclass
class Motor:
    id: int
    position: int
    goal: int
    torque: bool
    current: int
    status: int
    range_min: int
    range_max: int
    calibrated: bool  # range frozen by the station's calibration

    @property
    def has_range(self) -> bool:
        return steps_range(self.range_min, self.range_max) > SAFETY_MARGIN * 2

    @property
    def percent(self) -> float:
        return normalize(self.position, self.range_min, self.range_max)


def _u16(b: bytes, addr: int) -> int:
    return struct.unpack_from("<H", b, addr)[0] if len(b) >= addr + 2 else 0


def _u8(b: bytes, addr: int) -> int:
    return b[addr] if len(b) > addr else 0


def _position(raw: int) -> int:
    """ST3215 positions carry a sign bit; fold them into 0..4095."""
    if raw & 0x8000:
        return (MAX_STEPS - (raw & 0x0FFF)) & 0x0FFF
    return raw & 0x0FFF


def parse_motor(m) -> Motor:
    s = bytes(m.get_state() or b"")
    return Motor(
        id=m.get_id(),
        position=_position(_u16(s, REG_PRESENT_POSITION)),
        goal=_position(_u16(s, REG_GOAL_POSITION)),
        torque=_u8(s, REG_TORQUE_ENABLE) != 0,
        current=_u16(s, REG_PRESENT_CURRENT),
        status=_u8(s, REG_STATUS),
        range_min=m.get_range_min(),
        range_max=m.get_range_max(),
        calibrated=bool(m.get_range_freezed()),
    )


def parse_buses(frame) -> dict[str, dict[int, Motor]]:
    out: dict[str, dict[int, Motor]] = {}
    for b in frame.get_buses() or []:
        info = b.get_bus()
        serial = info.get_serial_number() if info else ""
        if not serial:
            continue
        out[serial] = {mm.id: mm for mm in (parse_motor(m) for m in b.get_motors() or [])}
    return out


# Same mapping as the station's mirroring (normalize.rs / mirror.rs).
def steps_range(lo: int, hi: int) -> int:
    return hi - lo if hi >= lo else (MAX_STEPS - lo) + hi


def normalize(pos: int, lo: int, hi: int) -> float:
    size = steps_range(lo, hi)
    if size == 0:
        return 50.0
    if hi >= lo:
        rel = 0 if pos < lo else size if pos > hi else pos - lo
    elif pos >= lo:
        rel = pos - lo
    elif pos <= hi:
        rel = (MAX_STEPS - lo) + pos
    else:
        rel = 0 if (lo - pos) < (pos - hi) else size
    return max(0.0, min(100.0, rel / size * 100.0))


def mapped_goal(leader: Motor, follower: Motor) -> int:
    usable = steps_range(follower.range_min, follower.range_max) - SAFETY_MARGIN * 2
    offset = int(leader.percent / 100.0 * usable) + SAFETY_MARGIN
    return (follower.range_min + offset) % MAX_STEPS


def circ_dist(a: int, b: int) -> int:
    d = abs(a - b) % MAX_STEPS
    return MAX_STEPS - d if d > MAX_STEPS // 2 else d


# --------------------------------------------------------------------------- #
# Station connection
# --------------------------------------------------------------------------- #

class Station:
    def __init__(self, client):
        self.client = client
        self.buses: dict[str, dict[int, Motor]] = {}
        self.frame_time = 0.0
        self.frames = 0
        self._task: asyncio.Task | None = None

    @classmethod
    async def connect(cls, host: str) -> "Station":
        try:
            client = await new_station_client(host, logging.getLogger("station_py"))
        except Exception as e:
            raise SystemExit(
                f"Cannot connect to station at {host} (TCP port 8888). "
                f"Is it running with `./station --web --tcp`? ({e!r})"
            )
        st = cls(client)
        st._task = asyncio.create_task(st._follow())
        return st

    async def _follow(self):
        q: asyncio.Queue = asyncio.Queue()
        errors = self.client.follow(ST3215_QUEUE, q)
        while True:
            if not errors.empty():
                raise RuntimeError(f"st3215 stream error: {errors.get_nowait()}")
            entry = await q.get()
            if entry is None:
                raise RuntimeError("st3215 stream closed")
            try:
                frame = st3215.InferenceStateReader(memoryview(bytes(entry.Data)))
                self.buses = parse_buses(frame)
                self.frame_time = time.monotonic()
                self.frames += 1
            except Exception:
                log.exception("failed to decode st3215 frame")

    async def wait_frame(self, timeout: float = 5.0):
        start = self.frames
        deadline = time.monotonic() + timeout
        while self.frames == start:
            if self._task and self._task.done():
                self._task.result()
            if time.monotonic() > deadline:
                raise SystemExit("No motor data from the station (is an arm plugged in?)")
            await asyncio.sleep(0.02)

    async def mirroring_state(self) -> tuple[dict[str, int], list[tuple[str, list[str]]]]:
        """Latest (modes, [(source, [targets])]) from the station's mirroring driver."""
        qr = self.client.read_from_tail(MIRRORING_QUEUE, b"\x01", 1, 1, 4)
        last = None
        try:
            while True:
                entry = await asyncio.wait_for(qr.data.get(), 3.0)
                if entry is None:
                    break
                last = entry
        except asyncio.TimeoutError:
            pass
        if last is None:
            return {}, []
        state = mirroring.RxEnvelopeReader(memoryview(bytes(last.Data))).get_state()
        modes = {m.get_id().get_unique_id(): int(m.get_mode()) for m in state.get_modes() or []}
        pairs = [
            (m.get_source().get_id().get_unique_id(),
             [t.get_id().get_unique_id() for t in m.get_targets() or []])
            for m in state.get_mirroring() or []
        ]
        return modes, pairs

    # -- commands ---------------------------------------------------------- #

    async def _send(self, cmds):
        cmds = [c for c in cmds if c is not None]
        if cmds:
            await send_commands(self.client, cmds)

    @staticmethod
    def _sync_write(bus: str, address: int, writes: list[tuple[int, bytes]]):
        if not writes:
            return None
        cmd = st3215.Command(
            target_bus_serial=bus,
            sync_write=st3215.ST3215SyncWriteCommand(
                address=address,
                motors=[st3215.ST3215SyncWriteCommand_MotorWrite(motor_id=i, value=v) for i, v in writes],
            ),
        )
        return station_commands.DriverCommand(
            type=drivers.StationCommandType.STC_ST3215_COMMAND, body=cmd.encode()
        )

    async def set_torque(self, bus: str, ids, on: bool):
        await self._send([self._sync_write(bus, REG_TORQUE_ENABLE, [(i, bytes([1 if on else 0])) for i in ids])])

    async def move(self, bus: str, goals: dict[int, int], speed: int, accel: int):
        ids = list(goals)
        await self._send([
            self._sync_write(bus, REG_GOAL_SPEED, [(i, speed.to_bytes(2, "little")) for i in ids]),
            self._sync_write(bus, REG_ACC, [(i, bytes([accel])) for i in ids]),
            self._sync_write(bus, REG_GOAL_POSITION, [(i, g.to_bytes(2, "little")) for i, g in goals.items()]),
        ])

    async def spin(self, bus: str, speed: int):
        """Run the extra STS3215 indefinitely in wheel mode.

        STS3215 wheel-mode speed is unsigned magnitude plus bit 15 for
        direction.  Torque must be off while changing the EEPROM mode.
        """
        if not -1000 <= speed <= 1000:
            raise ValueError("spin speed must be between -1000 and 1000")
        await self.set_torque(bus, [EXTRA_SERVO_ID], False)
        await self._send([self._sync_write(
            bus, REG_OPERATING_MODE, [(EXTRA_SERVO_ID, bytes([1]))]
        )])
        raw_speed = abs(speed) | (0x8000 if speed < 0 else 0)
        await self._send([self._sync_write(
            bus, REG_GOAL_SPEED, [(EXTRA_SERVO_ID, raw_speed.to_bytes(2, "little"))]
        )])
        await self.set_torque(bus, [EXTRA_SERVO_ID], True)

    async def _mirror_cmd(self, ctype, source: str, targets: list[str] = ()):
        bus = lambda s: mirroring.MirroringBus(type=mirroring.BusType.MBT_ST3215, unique_id=s)  # noqa: E731
        cmd = mirroring.Command(type=ctype, source=bus(source), targets=[bus(t) for t in targets])
        await self._send([station_commands.DriverCommand(
            type=drivers.StationCommandType.STC_MOTOR_MIRRORING_COMMAND, body=cmd.encode()
        )])

    async def start_mirror(self, leader: str, follower: str):
        await self._mirror_cmd(mirroring.CommandType.CT_START_MIRROR, leader, [follower])

    async def stop_mirror(self, bus: str):
        await self._mirror_cmd(mirroring.CommandType.CT_STOP_MIRROR, bus)


# --------------------------------------------------------------------------- #
# arms.json
# --------------------------------------------------------------------------- #

def load_arms() -> dict:
    try:
        return json.loads(ARMS_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_arms(leader: str, follower: str | None):
    ARMS_FILE.write_text(json.dumps({"leader": leader, "follower": follower}, indent=2) + "\n")
    print(f"Saved roles to {ARMS_FILE}")


def resolve_roles(args, buses: dict) -> tuple[str, str]:
    saved = load_arms()
    leader = args.leader or saved.get("leader")
    follower = args.follower or saved.get("follower")
    present = sorted(buses)

    # Only one side known: if exactly two buses, the other one is the other role.
    if len(present) == 2:
        if leader and not follower and leader in present:
            follower = next(s for s in present if s != leader)
        elif follower and not leader and follower in present:
            leader = next(s for s in present if s != follower)

    if not leader or not follower:
        raise SystemExit(
            f"Don't know which arm is the leader. Buses seen: {present or 'none'}.\n"
            "Run `python3 teleop.py identify` (move the leader arm while it watches),\n"
            "or pass --leader SERIAL --follower SERIAL."
        )
    if leader == follower:
        raise SystemExit("Leader and follower must be different buses.")
    missing = [f"{r} {s}" for r, s in (("leader", leader), ("follower", follower)) if s not in buses]
    if missing:
        raise SystemExit(
            f"Not connected: {', '.join(missing)}. Buses seen: {present or 'none'}.\n"
            "Plug the arm in and attach it to WSL: usbipd list / usbipd attach --wsl --busid <id>"
        )
    return leader, follower


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #

MODE_NAMES = {1: "LEADER", 2: "FOLLOWER"}


async def cmd_status(st: Station, args):
    await st.wait_frame()
    modes, pairs = await st.mirroring_state()
    saved = load_arms()
    print(f"Station: {args.host}   buses: {len(st.buses)}")
    for serial, motors in sorted(st.buses.items()):
        role = MODE_NAMES.get(modes.get(serial, 0))
        tag = role or ("leader (arms.json)" if serial == saved.get("leader")
                       else "follower (arms.json)" if serial == saved.get("follower") else "unassigned")
        print(f"\n  bus #{serial}  [{tag}]  {len(motors)} motors")
        print("    id   pos     %     torque  current  calibrated")
        for m in sorted(motors.values(), key=lambda m: m.id):
            cal = "yes" if m.calibrated else ("range only" if m.has_range else "NO")
            print(f"    {m.id:<3} {m.position:>5}  {m.percent:5.1f}  {'on ' if m.torque else 'off'}     "
                  f"{m.current:>5}    {cal}")
    print()
    if pairs:
        for src, tgts in pairs:
            print(f"Mirroring ACTIVE: #{src} -> {', '.join('#' + t for t in tgts)}")
    else:
        print("Mirroring: off")


async def cmd_identify(st: Station, args):
    await st.wait_frame()
    if not st.buses:
        raise SystemExit("No arm buses connected.")
    print(f"Buses seen: {', '.join('#' + s for s in sorted(st.buses))}")
    print(f">>> Move the LEADER arm around now (watching for {args.seconds:.0f}s)...")

    motion = {s: 0 for s in st.buses}
    prev = {s: {i: m.position for i, m in ms.items()} for s, ms in st.buses.items()}
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        await asyncio.sleep(0.05)
        for s, ms in st.buses.items():
            p = prev.setdefault(s, {})
            for i, m in ms.items():
                if i in p:
                    motion[s] = motion.get(s, 0) + circ_dist(m.position, p[i])
                p[i] = m.position

    for s, total in sorted(motion.items(), key=lambda kv: -kv[1]):
        print(f"  #{s}: moved {total} steps")
    leader, top = max(motion.items(), key=lambda kv: kv[1])
    if top < 200:
        raise SystemExit("Didn't see an arm move enough. Run identify again and move the leader more.")
    others = [s for s in st.buses if s != leader]
    if len(others) == 1 and motion[others[0]] > top * 0.5:
        raise SystemExit("Both arms moved a similar amount. Move ONLY the leader and try again.")
    follower = others[0] if len(others) == 1 else None
    print(f"\nLeader:   #{leader}")
    print(f"Follower: #{follower}" if follower else
          "Follower: (not connected yet. It will be the other bus once plugged in)")
    save_arms(leader, follower)


async def soft_start(st: Station, leader: str, follower: str, ids: list[int], args):
    """Bring the follower to the leader's pose slowly before mirroring takes over.

    The station's mirroring picks speed by distance (up to max speed), so a
    big pose mismatch would make the follower whip across. Here we track the
    leader at a gentle speed until the follower is close.
    """
    print(f"Soft start: moving follower to leader pose at speed {args.soft_speed} "
          f"(max {args.soft_timeout:.0f}s)...")
    await st.set_torque(follower, ids, True)
    deadline = time.monotonic() + args.soft_timeout
    err = None
    while time.monotonic() < deadline:
        L, F = st.buses.get(leader, {}), st.buses.get(follower, {})
        if not L or not F:
            raise SystemExit("Lost an arm bus during soft start.")
        goals = {i: mapped_goal(L[i], F[i]) for i in ids}
        err = {i: circ_dist(F[i].position, goals[i]) for i in ids}
        if max(err.values()) <= args.soft_tolerance:
            print(f"  follower aligned (max error {max(err.values())} steps)")
            return
        await st.move(follower, goals, args.soft_speed, args.soft_accel)
        await asyncio.sleep(0.15)
    worst = max(err, key=err.get)
    print(f"  soft start timed out (worst motor {worst}: {err[worst]} steps off). "
          "Continuing; the station's current protection will stop it if blocked.")


async def cmd_start(st: Station, args):
    await st.wait_frame()
    leader, follower = resolve_roles(args, st.buses)
    L, F = st.buses[leader], st.buses[follower]

    ids = sorted(set(L) & set(F))
    if not ids:
        raise SystemExit(f"No common motor IDs: leader {sorted(L)}, follower {sorted(F)}")
    if set(L) != set(F):
        print(f"WARNING: motor IDs differ (leader {sorted(L)}, follower {sorted(F)}). "
              f"Mirroring only {ids}.")
    bad = [f"{name} motor {i}" for name, ms in (("leader", L), ("follower", F))
           for i in ids if not ms[i].has_range]
    if bad:
        raise SystemExit("Uncalibrated motors (no range): " + ", ".join(bad) +
                         ".\nCalibrate both arms in the web UI first.")
    uncal = [f"{name} {i}" for name, ms in (("leader", L), ("follower", F)) for i in ids if not ms[i].calibrated]
    if uncal:
        print(f"WARNING: calibration not finalized for: {', '.join(uncal)}")

    print(f"Leader   #{leader}  ->  Follower #{follower}   motors {ids}")
    _, pairs = await st.mirroring_state()
    already = any(src == leader and follower in t for src, t in pairs)

    if already:
        print("Mirroring is already active.")
    else:
        for src, tgts in pairs:  # don't fight another mapping on the same buses
            if {src, *tgts} & {leader, follower}:
                print(f"Stopping existing mirroring #{src} -> {tgts}")
                await st.stop_mirror(src)
                await asyncio.sleep(0.5)
        if not args.keep_leader_torque:
            await st.set_torque(leader, list(L), False)  # leader must be free to move by hand
        if not args.no_soft_start:
            await soft_start(st, leader, follower, ids, args)
        await st.start_mirror(leader, follower)

        for _ in range(20):
            await asyncio.sleep(0.15)
            _, pairs = await st.mirroring_state()
            if any(src == leader and follower in t for src, t in pairs):
                break
        else:
            raise SystemExit("Station did not confirm mirroring. Check the station log.")
        print("Mirroring ACTIVE. Move the leader arm.")

    if args.detach:
        print("Detached: mirroring keeps running in the station. "
              "Stop it with `python3 teleop.py stop` or from the web UI.")
        return

    print("Ctrl+C to stop. The follower will go LIMP on stop, so support it or rest it first.\n")
    try:
        last_print = 0.0
        while True:
            await asyncio.sleep(0.2)
            if time.monotonic() - st.frame_time > 1.0:
                print("WARNING: no motor data for >1s")
                continue
            if leader not in st.buses or follower not in st.buses:
                print("WARNING: an arm bus disappeared (USB unplugged?)")
                continue
            if time.monotonic() - last_print >= args.print_every:
                last_print = time.monotonic()
                L, F = st.buses[leader], st.buses[follower]
                errs = {i: circ_dist(F[i].position, mapped_goal(L[i], F[i])) for i in ids if i in L and i in F}
                worst = max(errs, key=errs.get)
                hot = [i for i in ids if i in F and F[i].current >= 100]
                print(f"tracking error: max {errs[worst]:>4} steps (motor {worst})"
                      + (f"   current-limited: {hot}" if hot else ""))
                _, pairs = await st.mirroring_state()
                if not any(src == leader and follower in t for src, t in pairs):
                    print("Mirroring was stopped elsewhere (web UI?). Exiting.")
                    return
    except asyncio.CancelledError:
        pass
    finally:
        print("\nStopping mirroring...")
        await st.stop_mirror(leader)
        await asyncio.sleep(0.3)
        print("Stopped.")


async def cmd_stop(st: Station, args):
    await st.wait_frame()
    _, pairs = await st.mirroring_state()
    sources = [args.leader] if args.leader else [src for src, _ in pairs] or \
        ([load_arms()["leader"]] if load_arms().get("leader") else [])
    if not sources:
        print("Mirroring is not active.")
        return
    for src in sources:
        print(f"Stopping mirroring from #{src} (follower torque goes off)")
        await st.stop_mirror(src)
    await asyncio.sleep(0.5)
    _, pairs = await st.mirroring_state()
    print("Mirroring: off" if not pairs else f"Still active: {pairs}")


# --------------------------------------------------------------------------- #

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="localhost", help="station host[:port], default localhost:8888")
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="show buses, motors and mirroring state")

    s = sub.add_parser("identify", help="detect the leader arm by watching which one you move")
    s.add_argument("--seconds", type=float, default=8.0)

    s = sub.add_parser("start", help="start leader -> follower mirroring")
    s.add_argument("--leader", help="leader bus serial (default: arms.json)")
    s.add_argument("--follower", help="follower bus serial (default: arms.json / the other bus)")
    s.add_argument("--detach", action="store_true", help="exit and leave mirroring running")
    s.add_argument("--no-soft-start", action="store_true", help="skip slow alignment before mirroring")
    s.add_argument("--soft-speed", type=int, default=400, help="alignment speed (station mirroring max is 3300)")
    s.add_argument("--soft-accel", type=int, default=10)
    s.add_argument("--soft-tolerance", type=int, default=60, help="steps; alignment done below this")
    s.add_argument("--soft-timeout", type=float, default=10.0)
    s.add_argument("--keep-leader-torque", action="store_true", help="don't disable leader torque")
    s.add_argument("--print-every", type=float, default=2.0)

    s = sub.add_parser("stop", help="stop mirroring")
    s.add_argument("--leader", help="source bus to stop (default: whatever is active)")

    s = sub.add_parser("spin", help="run extra servo 10 continuously in wheel mode")
    s.add_argument("--bus", required=True, help="ST3215 bus serial")
    s.add_argument("--speed", type=int, required=True,
                   help="speed -1000..1000; negative reverses direction, zero stops")

    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    async def run():
        st = await Station.connect(args.host)
        if args.cmd == "spin":
            await st.spin(args.bus, args.speed)
            print(f"Servo {EXTRA_SERVO_ID} wheel mode: speed {args.speed}")
        else:
            await {"status": cmd_status, "identify": cmd_identify,
                   "start": cmd_start, "stop": cmd_stop}[args.cmd](st, args)

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
