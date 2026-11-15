#!/usr/bin/env python3
"""Run postal task, navigation loop or sensor-failure scenarios, entirely in simulation."""
import argparse
import importlib
import json
import time
from contextlib import nullcontext
from pathlib import Path

import numpy as np

from .common import HERE, config
from .env import PostalEnv
from .navigation import HolonomicNavigator
from .slam import RangeSlam
from .task import PostalFSM, State


def resolve(spec, default):
    if spec is None: return default
    module, symbol = spec.split(':')
    return getattr(importlib.import_module(module), symbol)


def run(settings=None, scenario='mission', seed=1, output=None, viewer=False, realtime=False,
        controller=None, slam_factory=None, arm_config=None, max_steps=None, snapshot=None):
    c = settings or config()
    if not c['letters'] and scenario == 'mission': scenario = 'navigation'
    directory = Path(output or HERE/'.runtime/runs'/f'{scenario}-{seed}-{time.time_ns()}')
    directory.mkdir(parents=True, exist_ok=False)
    env = PostalEnv(c, arm_config=arm_config)
    actor = resolve(controller, PostalFSM)(c, env.arm, env.pickup_q)
    mapper = resolve(slam_factory, RangeSlam)(c)
    navigator = HolonomicNavigator(c)
    if scenario in ('navigation', 'loop'):
        observation, _ = env.reset(seed=seed, options={'arm_q': actor.carry_q})
    else: observation, _ = env.reset(seed=seed)
    mapper.update(observation)
    if viewer:
        import mujoco.viewer
        context = mujoco.viewer.launch_passive(env.model, env.data)
        with context.lock():
            context.cam.lookat[:] = env.data.xpos[env.base_id]
    else: context = nullcontext(None)
    goals = [b['approach_pose'] for b in c['mailboxes']]
    if scenario == 'loop': goals.append(c['start_pose'])
    goal_index, feedback, records = 0, {}, []
    status, error, collisions, instabilities, limit_violations = 'running', None, 0, 0, 0
    start_wall = time.monotonic()
    try:
        with context as window, (directory/'trace.jsonl').open('w') as trace:
            for step in range(max_steps or int(c['task']['max_time_s']/c['control_period_s'])):
                tick = time.monotonic()
                if scenario == 'dropout' and observation['time'][0] >= 2: env.sensor_dropout = True
                if scenario in ('navigation', 'loop'):
                    twist, arrived = navigator.command(goals[goal_index], mapper, observation)
                    action = dict(twist=twist, arm=actor.carry_q, gripper=np.array([0.0]))
                    events = []
                    if arrived:
                        goal_index += 1
                        navigator.goal = None
                        if goal_index == len(goals): status = 'done'; break
                else:
                    feedback['delivered'] = dict(env.delivered)
                    action, events = actor.update(observation, mapper, feedback)
                    if actor.finished:
                        status, error = actor.state.value.lower(), actor.error
                        break
                feedback = {}
                for event in events:
                    if event['type'] == 'grasp':
                        feedback['grasped' if env.grasp(event['letter']) else 'grasp_failed'] = True
                    elif event['type'] == 'release': feedback['released'] = env.release()
                observation, _, terminated, truncated, info = env.step(action)
                mapper.update(observation)
                collisions += int(bool(info['collision_pairs']))
                instabilities += int(terminated)
                limit_violations += int(np.any(observation['joints'] < env.arm.bounds[:, 0]-.002)
                                        or np.any(observation['joints'] > env.arm.bounds[:, 1]+.002))
                record = dict(time_s=float(observation['time'][0]), state=actor.state.value if scenario not in ('navigation', 'loop') else navigator.status,
                    pose=mapper.pose.tolist(), odometry_pose=mapper.odometry_pose.tolist(),
                    ground_truth_pose=info['ground_truth_pose'], quality=mapper.quality,
                    twist=action['twist'].tolist(), contacts=info['collision_pairs'],
                    joints=observation['joints'].tolist(), arm_command=action['arm'].tolist(),
                    held=env.held, delivered=dict(env.delivered))
                records.append(record)
                trace.write(json.dumps(record, allow_nan=False)+'\n')
                if window:
                    window.sync()
                    if not window.is_running(): status = 'viewer_closed'; break
                if terminated or truncated:
                    status, error = ('unstable', 'base tipped or fell') if terminated else ('timeout', 'scenario time limit')
                    break
                if info['collision_pairs']:
                    status, error = 'collision', str(info['collision_pairs'])
                    break
                if realtime or viewer: time.sleep(max(0, c['control_period_s']/2-(time.monotonic()-tick)))
            else: status, error = 'timeout', 'step limit'
        positions = np.array([r['pose'][:2] for r in records])
        truth = np.array([r['ground_truth_pose'][:2] for r in records])
        odometry = np.array([r['odometry_pose'][:2] for r in records])
        rmse = lambda values: float(np.sqrt(np.mean(np.sum((values-truth)**2, axis=1)))) if len(records) else None
        report = dict(scenario=scenario, seed=seed, status=status, error=error,
            simulated_time_s=float(env.data.time), wall_time_s=time.monotonic()-start_wall,
            delivered=dict(env.delivered), goals_reached=goal_index, fsm_transitions=actor.history,
            slam_rmse_m=rmse(positions), odometry_rmse_m=rmse(odometry),
            loop_closures=mapper.loop_closures, keyframes=len(mapper.keyframes),
            observed_map_fraction=float(np.mean(np.abs(mapper.map.logodds) > .4)),
            collision_steps=collisions, instability_steps=instabilities, joint_limit_violation_steps=limit_violations,
            hardware_verified=c['hardware_verified'], profile=c['profile'],
            virtual_range_sensor=c['lidar']['virtual_only'], semantic_labels='scenario fixtures; no vision',
            model_xml=str(env.xml_path), output=str(directory))
        report['arm_limits_rad'] = env.arm.bounds.tolist()
        report['arm_command_limits_rad'] = env.arm.command_bounds.tolist()
        report['arm_calibration_applied'] = env.arm.c['hardware_verified']
        (directory/'config.json').write_text(json.dumps(c, indent=2)+'\n')
        (directory/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        np.savez_compressed(directory/'map.npz', logodds=mapper.map.logodds,
                            resolution=mapper.map.resolution, origin=mapper.map.origin)
        save_map(directory/'map.png', mapper, records, c)
        if snapshot:
            from PIL import Image
            Image.fromarray(env.render()).save(snapshot)
        return report
    finally:
        actor.close()
        env.close()


def save_map(path, mapper, records, c):
    from PIL import Image, ImageDraw
    grid = mapper.map.ros_grid()
    values = np.where(grid == -1, 160, np.where(grid == 100, 35, 245)).astype(np.uint8)
    image = Image.fromarray(values).convert('RGB').resize((values.shape[1]*5, values.shape[0]*5))
    draw = ImageDraw.Draw(image)
    def pixel(xy):
        row, col = mapper.map.pixel(xy)
        return col*5, row*5
    if len(records) > 1:
        draw.line([pixel(r['ground_truth_pose'][:2]) for r in records], fill=(20, 160, 40), width=3)
        draw.line([pixel(r['pose'][:2]) for r in records], fill=(30, 80, 230), width=2)
    for box in c['mailboxes']:
        x, y = pixel(box['approach_pose'][:2]); draw.ellipse((x-5, y-5, x+5, y+5), fill=(230, 50, 30))
    image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, 24), fill='white')
    draw.text((8, 6), 'SLAM bleu | verite verte | buts rouges | noir occupe / gris inconnu', fill='black')
    image.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--arm-config', type=Path, help='Calibration réelle optionnelle pour resserrer les butées')
    parser.add_argument('--scenario', choices=['mission', 'navigation', 'loop', 'dropout'], default='mission')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--realtime', action='store_true')
    parser.add_argument('--controller', help='Module:classe remplaçant la FSM de référence')
    parser.add_argument('--slam', help='Module:classe remplaçant RangeSlam')
    parser.add_argument('--snapshot', type=Path, help='Vue PNG en fin de scénario ; MUJOCO_GL=egl en mode sans écran')
    args = parser.parse_args()
    report = run(config(args.config), args.scenario, args.seed, args.output, args.viewer,
                 args.realtime, args.controller, args.slam, args.arm_config, snapshot=args.snapshot)
    print(json.dumps(report, indent=2))
    expected = report['status'] == 'done' or (args.scenario == 'dropout' and report['status'] == 'fault' and 'périmé' in report['error'])
    raise SystemExit(0 if expected else 1)


if __name__ == '__main__': main()
