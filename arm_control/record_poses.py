#!/usr/bin/env python3
"""Record named encoder poses from Station; never send motor commands."""
import argparse
import asyncio
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'teleop'))
from teleop import Station


async def capture(host, bus):
    station = await Station.connect(host)
    try:
        samples = []
        for _ in range(3):
            await station.wait_frame()
            motors = station.buses.get(bus, {})
            if any(i not in motors for i in range(1, 9)):
                raise ValueError('Missing arm joints: expected IDs 1–8')
            sample = {i: asdict(motors[i]) for i in range(1, 9)}
            for i, m in sample.items():
                if m['status'] or not m['calibrated']:
                    raise ValueError(f'ID {i}: servo error or unfrozen calibration')
                lo, hi, pos = m['range_min'], m['range_max'], m['position']
                if lo == hi:
                    raise ValueError(f'ID {i}: empty calibration range')
                inside = lo <= pos <= hi if lo < hi else pos >= lo or pos <= hi
                if not inside:
                    raise ValueError(f'ID {i}: encoder outside saved range')
            samples.append(sample)
            await asyncio.sleep(.15)
        for i in range(1, 9):
            positions = [sample[i]['position'] for sample in samples]
            if max(positions) - min(positions) > 3:
                raise ValueError(f'ID {i} moved during capture; support and hold still')
        return samples[-1]
    finally:
        if station._task:
            station._task.cancel()
            await asyncio.gather(station._task, return_exceptions=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1:18888')
    parser.add_argument('--bus', default='5B8E114718')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).with_name('poses.local.json'))
    parser.add_argument('names', nargs='+')
    args = parser.parse_args()
    if len(set(args.names)) != len(args.names):
        parser.error('Pose names must be unique')
    if args.output.exists():
        data = json.loads(args.output.read_text())
        if data.get('schema') != 'encoder-poses-v1' or data.get('bus') != args.bus:
            parser.error('Existing output has incompatible schema or bus')
    else:
        data = {'schema': 'encoder-poses-v1', 'bus': args.bus,
                'model_calibrated': False, 'poses': {}}
    if any(name in data['poses'] for name in args.names):
        parser.error('A requested pose already exists; use another name or --output')
    try:
        for name in args.names:
            input(f'Place and support arm in "{name}"; hold still, then press Enter: ')
            motors = asyncio.run(capture(args.host, args.bus))
            data['poses'][name] = {
                'recorded_at': datetime.now(timezone.utc).isoformat(),
                'motors': motors,
                'ticks': {i: m['position'] for i, m in motors.items()}}
            temporary = args.output.with_name(args.output.name + '.tmp')
            with temporary.open('x') as stream:
                json.dump(data, stream, indent=2)
                stream.write('\n')
            temporary.replace(args.output)
            print(f'Saved {name}: {data["poses"][name]["ticks"]}', flush=True)
        print(f'Recorded in {args.output}. No movement commands sent.')
    except (ValueError, OSError, RuntimeError) as exc:
        parser.exit(1, f'Recording stopped: {exc}\n')
    except (KeyboardInterrupt, EOFError):
        parser.exit(130, '\nStopped; previously saved poses retained.\n')


if __name__ == '__main__':
    main()
