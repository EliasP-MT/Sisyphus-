"""Simulation-to-execution workflow. Execution reuses the guarded planner executor."""
import asyncio
import copy
import hashlib
import json
from pathlib import Path
import secrets
import threading
import time

from planner import Arm, joints_to_ticks, ticks_to_joints


class Motions:
    def __init__(self, root, host, scene=None, verified_config=None):
        self.root, self.host = root, host
        self.scene, self.verified_config = scene, verified_config
        self.plans = {}
        self.lock = threading.Lock()
        self.status = {'stage': 'idle', 'message': 'No real execution requested'}

    def progress(self, message):
        self.status = {'stage': 'working', 'message': message}

    def preview(self, config, current, target):
        # Allow offline numerical evaluation without approving hardware.
        numeric = copy.deepcopy(config)
        numeric['hardware_verified'] = True
        reasons = []
        for j, numeric_joint in zip(config['joints'], numeric['joints']):
            i = str(j['motor_id'])
            for label, motors in [('current arm', current), ('target pose', target['motors'])]:
                value = motors[i]['position']
                if not j['min_ticks'] <= value <= j['max_ticks']:
                    if label == 'current arm':
                        measured = motors[i]
                        lo, hi = measured.get('range_min'), measured.get('range_max')
                        if lo is not None and hi is not None and lo < hi and lo <= value <= hi:
                            reasons.append(f'J{i} starts outside the operating margin; simulation only. Reposition inside {j["min_ticks"]}–{j["max_ticks"]} ticks before deployment')
                            numeric_joint['min_ticks'] = min(j['min_ticks'], value)
                            numeric_joint['max_ticks'] = max(j['max_ticks'], value)
                            continue
                    raise ValueError(f"J{i} {label}: {value} ticks outside configured limits {j['min_ticks']}–{j['max_ticks']}; reposition or correct the configuration before planning")
        arm = Arm(self.root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf', numeric)
        start = ticks_to_joints(numeric, [current[str(i)]['position'] for i in range(1, 8)])
        end = ticks_to_joints(numeric, [target['motors'][str(i)]['position'] for i in range(1, 8)])
        arm.check(start)
        arm.check(end)
        world = None
        try:
            if self.scene:
                from collision import CollisionWorld
                world = CollisionWorld(self.root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf', arm, self.scene)
                path = world.joint_path(start, end)
            else:
                path = [start, end]
                reasons.append('No registered collision scene configured')
            rows = []
            for a, b in zip(path, path[1:]):
                segment = arm.segment(a, b)
                offset = rows[-1]['t_s'] if rows else 0
                for row in segment[1:] if rows else segment:
                    row['t_s'] += offset
                    rows.append(row)
            plan = dict(samples=rows, collision_checked=bool(world),
                        targets_xyz_m=[arm.fk(end)[:3, 3].tolist()])
            if world:
                plan['collision_scene_sha256'] = world.fingerprint
                world.validate_plan(plan)
                quantized = copy.deepcopy(plan)
                for row in quantized['samples']:
                    row['q_rad'] = ticks_to_joints(numeric, joints_to_ticks(numeric, row['q_rad'])).tolist()
                world.validate_plan(quantized)
                try:
                    world.require_hardware_ready()
                except ValueError as exc:
                    reasons.append(str(exc))
            if not self.verified_config:
                reasons.append('No independently verified calibration configured')
            else:
                verified = json.loads(Path(self.verified_config).read_text())
                expected = copy.deepcopy(config)
                expected['hardware_verified'] = True
                if verified != expected:
                    reasons.append('Current adjustments differ from verified configuration')
            if any(not m['torque'] for m in current.values()):
                reasons.append('Arm torque is off; prepare holding torque in Station before replanning')
            if self.scene and abs(current['8']['position'] - target['motors']['8']['position']) > 3:
                reasons.append('Gripper differs from target; this workflow moves seven arm joints only')
            token = secrets.token_urlsafe(24)
            record = dict(config=numeric, plan=plan, created=time.monotonic(),
                          reasons=reasons, claimed=False,
                          scene_hash=hashlib.sha256(Path(self.scene).read_bytes()).hexdigest() if self.scene else None,
                          verified_hash=hashlib.sha256(Path(self.verified_config).read_bytes()).hexdigest() if self.verified_config else None)
            with self.lock:
                self.plans = {k: v for k, v in self.plans.items() if time.monotonic()-v['created'] < 600}
                self.plans[token] = record
            return dict(token=token, samples=rows, duration=rows[-1]['t_s'],
                        executable=not reasons, reasons=reasons, gripper='held unchanged')
        finally:
            if world:
                world.close()

    def execute(self, token):
        self.progress('Checking simulation confirmation')
        with self.lock:
            record = self.plans.get(token)
            if not record or record['claimed']:
                raise ValueError('Unknown or already used preview; simulate again')
            elapsed = time.monotonic()-record['created']
            if elapsed < record['plan']['samples'][-1]['t_s']:
                raise ValueError('Finish the simulation first')
            if elapsed > 600 or record['reasons']:
                raise ValueError('; '.join(record['reasons']) or 'Preview expired')
            record['claimed'] = True
        if hashlib.sha256(Path(self.scene).read_bytes()).hexdigest() != record['scene_hash']:
            raise ValueError('Scene changed; simulate again')
        if hashlib.sha256(Path(self.verified_config).read_bytes()).hexdigest() != record['verified_hash']:
            raise ValueError('Calibration changed; simulate again')

        async def run():
            from station import connection, execute
            from collision import CollisionWorld
            c = record['config']
            urdf = self.root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'
            arm = Arm(urdf, c)
            self.progress('Loading and verifying rover collision geometry')
            with CollisionWorld(urdf, arm, self.scene) as world:
                arm.collision = world
                self.progress('Connecting to Station and reading fresh arm feedback')
                async with connection(c, self.root/'.sdk', self.host) as station:
                    comparison=world.scene.get('simplification',{}).get('comparison',{})
                    period=world.scene.get('execution_command_period_s',.06)
                    streamed=(world.scene.get('execution_mode')=='streamed'
                              and comparison.get('missed_source_collisions')==[]
                              and comparison.get('timings',{}).get('proxy',{}).get('steady_p95_ms',float('inf'))<period*500)
                    await execute(station, arm, record['plan'], progress=self.progress,
                                  paced=not streamed, command_period=period if streamed else None)
            return {'status': 'arrived', 'gripper': 'unchanged'}
        try:
            result = asyncio.run(run())
            self.status = {'stage': 'done', 'message': 'Real arm arrived'}
            return result
        except BaseException as exc:
            self.status = {'stage': 'error', 'message': str(exc)}
            raise
