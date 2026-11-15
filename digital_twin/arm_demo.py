"""Exercise the existing XYZ planner against the MuJoCo arm, without Station.

Run: arm_control/.venv/bin/python -m digital_twin.arm_demo --viewer --repeat
"""
import argparse
import json
import time
from contextlib import nullcontext
from pathlib import Path

import numpy as np

from .common import HERE
from .env import PostalEnv
from .task import ArmController


def run(viewer=False, repeat=False, output=None):
    directory = Path(output or HERE / '.runtime' / f'arm-demo-{time.time_ns()}')
    directory.mkdir(parents=True, exist_ok=False)
    env = PostalEnv()
    motion = ArmController(env.arm, env.c)
    start = env.arm.ik([0, .13, .2], np.zeros(7))
    env.reset(seed=3, options={'arm_q': start})
    context = nullcontext(None)
    if viewer:
        import mujoco.viewer
        context = mujoco.viewer.launch_passive(env.model, env.data)
        with context.lock():
            context.cam.lookat[:] = env.data.xpos[env.base_id] + [0, 0, .18]
            context.cam.distance = 1.15
            context.cam.azimuth = 135
            context.cam.elevation = -22
    targets = [[.06, .19, .25], [-.06, .19, .25], [0, .13, .2]]
    report = dict(status='running', cycles=0, collision_steps=0,
                  limit_violations=0, max_tracking_error_rad=0., legs=[])
    print(f'Results: {directory}', flush=True)
    try:
        with context as window, (directory / 'trace.jsonl').open('w') as trace:
            while True:
                for target in targets:
                    motion.update_scene(env.ground_truth_pose(), False)
                    plan = env.arm.plan(env.arm_position(), [target])
                    index = len(report['legs'])
                    (directory / f'plan-{index}.json').write_text(json.dumps(plan))
                    times = np.array([r['t_s'] for r in plan['samples']])
                    positions = np.array([r['q_rad'] for r in plan['samples']])
                    origin = env.data.time
                    print(f'Target {target}: {times[-1]:.2f}s, '
                          f'{len(times)} samples, collisions checked', flush=True)
                    while env.data.time - origin < times[-1] + 1.5:
                        tick = time.monotonic()
                        elapsed = env.data.time - origin
                        command = np.array([np.interp(elapsed, times, positions[:, j]) for j in range(7)])
                        obs, _, unstable, _, info = env.step(dict(
                            twist=np.zeros(3), arm=command, gripper=[0.]))
                        measured = obs['joints']
                        error = float(np.max(np.abs(measured - command)))
                        report['max_tracking_error_rad'] = max(report['max_tracking_error_rad'], error)
                        report['collision_steps'] += bool(info['collision_pairs'])
                        report['limit_violations'] += int(np.any(measured < env.arm.bounds[:, 0] - .002)
                                                        or np.any(measured > env.arm.bounds[:, 1] + .002))
                        trace.write(json.dumps(dict(time_s=float(env.data.time), leg=index,
                            command_rad=command.tolist(), measured_rad=measured.tolist(),
                            tcp_m=env.local_tcp().tolist(), contacts=info['collision_pairs'])) + '\n')
                        if unstable or report['limit_violations'] or info['collision_pairs']:
                            raise RuntimeError(f'MuJoCo validation failed: {info}')
                        if window:
                            if not window.is_running():
                                report['status'] = 'viewer_closed'
                                return report
                            window.sync()
                            time.sleep(max(0., env.c['control_period_s'] - (time.monotonic() - tick)))
                    distance = float(np.linalg.norm(env.local_tcp() - target))
                    leg = dict(target_m=target, tcp_error_m=distance,
                               collision_checked=plan['collision_checked'], duration_s=float(times[-1]))
                    report['legs'].append(leg)
                    print(f'Measured TCP error: {distance * 1000:.2f} mm', flush=True)
                    if distance > .01:
                        raise RuntimeError('Measured TCP error exceeds 10 mm')
                report['cycles'] += 1
                report['status'] = 'passed'
                (directory / 'report.json').write_text(json.dumps(report, indent=2))
                if not repeat:
                    return report
    except BaseException as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        (directory / 'report.json').write_text(json.dumps(report, indent=2))
        motion.close()
        env.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--repeat', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.viewer, args.repeat, args.output), indent=2))
