#!/usr/bin/env python3
"""Offline joint-space preview using Station ranges. Never connects to hardware.

Uses a bounded range-to-URDF mapping, not measured physical zero offsets.
This is a model preview, not clearance certification or an executable motor plan.
"""
import argparse
import json
from pathlib import Path

from planner import Arm, load_config


def pose_angles(arm, pose):
    angles = []
    for i, (lower, upper) in enumerate(arm.bounds, 1):
        motor = pose['motors'][str(i)]
        lo, hi, ticks = motor['range_min'], motor['range_max'], motor['position']
        if not motor['calibrated'] or not 0 <= lo < hi <= 4095 or not lo <= ticks <= hi:
            raise ValueError(f'ID {i}: invalid range or encoder')
        # Preserve Station's decreasing direction, keep both URDF endpoints.
        fraction = (ticks - lo) / (hi - lo)
        # Physical comparison by the operator: J3 bends opposite to the
        # default Station direction. Keep this explicit, not inferred.
        angle = (lower + fraction * (upper - lower) if i == 3
                 else upper - fraction * (upper - lower))
        angles.append(float(angle))
    return arm.check(angles)


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', type=Path, default=root/'poses.local.json')
    parser.add_argument('--urdf', type=Path,
                        default=root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('start')
    parser.add_argument('target')
    args = parser.parse_args()
    data = json.loads(args.poses.read_text())
    if data.get('schema') != 'encoder-poses-v1':
        parser.error('Unsupported recorded pose format')
    arm = Arm(args.urdf, load_config(root/'elrobot.example.json'))
    start, end = [pose_angles(arm, data['poses'][name]) for name in (args.start, args.target)]
    rows = arm.segment(start, end)
    result = dict(schema='station-range-preview-v1', start=args.start, target=args.target,
                  mapping='bounded normalized Station ranges; J3 direction reversed by operator; other alignment unverified',
                  hardware_executable=False, collision_checked=False, samples=rows)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(f'Preview planned: {len(rows)} points, {rows[-1]["t_s"]:.2f} seconds')
    print('No hardware commands. Collision clearance and physical mapping unverified.')


if __name__ == '__main__':
    main()
