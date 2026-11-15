#!/usr/bin/env python3
"""Mise au point ElRobot : découverte, références, calibration et petits essais."""
import argparse
import asyncio
import copy
import json
import logging
import math
import sys
import time
from datetime import datetime, timezone
from contextlib import nullcontext
from pathlib import Path

import numpy as np
from ruckig import InputParameter, Result, Ruckig, Trajectory

from planner import Arm, JOINTS, TICKS_PER_RAD, load_config, ticks_to_joints, vector
from station import connection, execute, measured

HERE = Path(__file__).resolve().parent


def snapshot(station):
    states = station.state()  # existing freshness/error/frozen range validation
    motors = []
    for mid, (position, lo, hi) in zip(station.ids, states):
        raw = bytes(station.motors[mid].get_state())
        if (int.from_bytes(raw[0x3a:0x3c], 'little') & 0x7fff) > 10:
            raise RuntimeError('Bras en mouvement : attendre avant la mesure')
        motors.append(dict(motor_id=mid, ticks=position, min_ticks=lo, max_ticks=hi,
                           torque_on=raw[0x28] == 1))
    return dict(schema='elrobot-reference-v1', recorded_at=datetime.now(timezone.utc).isoformat(),
                bus=station.bus, motors=motors)


async def stable_snapshot(station):
    first = snapshot(station)
    await asyncio.sleep(0.15)
    second = snapshot(station)
    if any(abs(a['ticks']-b['ticks']) > 2 for a, b in zip(first['motors'], second['motors'])):
        raise RuntimeError('Pose instable entre deux mesures')
    return second


async def discover(sdk, host):
    """Read all buses without selecting motor IDs or requiring calibration."""
    sys.path.insert(0, str(Path(sdk).resolve()))
    from software.station.shared.station_py import new_station_client
    from target.gen_python.protobuf.drivers.st3215 import st3215
    client = await asyncio.wait_for(new_station_client(host, logging.getLogger('commission')), 10)
    queue = asyncio.Queue()
    errors = client.follow('st3215/inference', queue)
    try:
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if not errors.empty():
                raise RuntimeError(str(errors.get_nowait()))
            entry = await asyncio.wait_for(queue.get(), max(0.01, deadline-time.monotonic()))
            if entry is None:
                raise RuntimeError('Flux Station fermé')
            buses = []
            for bus in st3215.InferenceStateReader(entry.Data).get_buses() or []:
                if not bus.get_bus():
                    continue
                motors = []
                for motor in bus.get_motors() or []:
                    raw = bytes(motor.get_state())
                    age = (time.time_ns()-motor.get_system_stamp_ns())/1e9
                    value = int.from_bytes(raw[0x38:0x3a], 'little') if len(raw) >= 0x3a else None
                    ticks = ((-(value & 4095) if value & 0x8000 else value) & 4095) if value is not None else None
                    motors.append(dict(motor_id=motor.get_id(), ticks=ticks,
                        fresh=0 <= age <= 0.5, range_frozen=motor.get_range_freezed(),
                        min_ticks=motor.get_range_min(), max_ticks=motor.get_range_max(),
                        status=raw[0x40] if len(raw) > 0x40 else None))
                buses.append(dict(bus=bus.get_bus().get_serial_number(), motors=motors))
            if buses and any(m['fresh'] for b in buses for m in b['motors']):
                return dict(buses=buses, writes_sent=0)
        raise RuntimeError('Aucune mesure moteur récente')
    finally:
        client.close_connection()


def reference_ticks(config, reference):
    if reference['schema'] != 'elrobot-reference-v1' or reference['bus'] != config['bus']:
        raise ValueError('Référence incompatible (schéma ou bus)')
    motors = reference['motors']
    ids = [j['motor_id'] for j in config['joints']]
    if [m['motor_id'] for m in motors] != ids:
        raise ValueError('Les IDs de la référence ne correspondent pas au mapping')
    ticks = vector([m['ticks'] for m in motors], 7, 'encodeurs de référence')
    for p, m in zip(ticks, motors):
        lo, hi = m['min_ticks'], m['max_ticks']
        if not (0 <= lo < hi <= 4095 and lo <= p <= hi):
            raise ValueError('Référence hors plage encodeur ou avec plage inversée')
    return ticks


def fit_calibration(config, references, urdf):
    if len(references) < 2:
        raise ValueError('Au moins deux poses de référence mesurées sont nécessaires')
    model_config = copy.deepcopy(config)
    model_config['hardware_verified'] = False
    arm = Arm(urdf, model_config)
    angles = np.array([arm.check(vector(r['known_q_rad'], 7, 'angles de référence')) for r in references])
    ticks = np.array([reference_ticks(config, r) for r in references])
    candidate = copy.deepcopy(config)
    candidate['hardware_verified'] = False  # a fit is not a physical validation
    report = []
    for i, joint in enumerate(candidate['joints']):
        if np.ptp(angles[:, i]) < 0.1 or np.ptp(ticks[:, i]) < 32:
            raise ValueError(f"{joint['name']}: variation insuffisante (>= 0.1 rad et 32 ticks requis)")
        solutions = []
        for sign in (-1, 1):
            zero = round(float(np.mean(ticks[:, i]-sign*TICKS_PER_RAD*angles[:, i])))
            residual = ticks[:, i]-(zero+sign*TICKS_PER_RAD*angles[:, i])
            solutions.append((float(np.max(np.abs(residual))), sign, zero))
        error, sign, zero = min(solutions)
        if error > 12 or not 0 <= zero <= 4095:
            raise ValueError(f"{joint['name']}: références incohérentes ou franchissement 4095/0 ({error:.1f} ticks)")
        lo = max(r['motors'][i]['min_ticks'] for r in references)+20
        hi = min(r['motors'][i]['max_ticks'] for r in references)-20
        if lo >= hi or np.any(ticks[:, i] < lo) or np.any(ticks[:, i] > hi):
            raise ValueError(f"{joint['name']}: références trop près des limites (marge 20 ticks)")
        joint.update(zero_ticks=zero, sign=sign, min_ticks=lo, max_ticks=hi)
        report.append(dict(joint=joint['name'], sign=sign, zero_ticks=zero, max_error_ticks=error))
    checked = copy.deepcopy(candidate)
    checked['hardware_verified'] = True
    calibrated_arm = Arm(urdf, checked)
    for row in ticks:
        calibrated_arm.check(ticks_to_joints(checked, row))
    candidate['calibration_report'] = report
    return candidate


def fit_tcp(config, references, urdf):
    if len(references) < 3:
        raise ValueError('Au moins trois poses avec TCP mesuré sont nécessaires')
    mount_config = copy.deepcopy(config)
    mount_config['tcp_xyz_m'] = [0, 0, 0]
    arm = Arm(urdf, mount_config)
    offsets, rotations = [], []
    for r in references:
        q = ticks_to_joints(config, reference_ticks(config, r))
        pose = arm.fk(q)
        rotations.append(pose[:3, :3])
        xyz = vector(r['measured_tcp_xyz_m'], 3, 'TCP mesuré dans base_link')
        offsets.append(pose[:3, :3].T @ (xyz-pose[:3, 3]))
    if max(np.linalg.norm(r-rotations[0]) for r in rotations) < 0.1:
        raise ValueError('Changer suffisamment l’orientation du bras entre les mesures TCP')
    mean = np.mean(offsets, axis=0)
    error = float(np.max(np.linalg.norm(np.asarray(offsets)-mean, axis=1)))
    if np.linalg.norm(mean) > 0.3 or error > 0.005:
        raise ValueError('Mesures TCP incohérentes (> 5 mm) ou décalage > 30 cm')
    result = copy.deepcopy(config)
    result.update(tcp_xyz_m=mean.tolist(), hardware_verified=False,
                  tcp_fit_max_residual_m=error)
    return result


def check_pose(config, reference, urdf):
    candidate = copy.deepcopy(config)
    candidate['hardware_verified'] = True  # numerical evaluation only; never saved as approved
    arm = Arm(urdf, candidate)
    q = ticks_to_joints(candidate, reference_ticks(candidate, reference))
    pose = arm.fk(q)
    joint_error = (float(np.max(np.abs(q-vector(reference['known_q_rad'], 7, 'angles connus'))))
                   if 'known_q_rad' in reference else None)
    tcp_error = (float(np.linalg.norm(pose[:3, 3]-vector(reference['measured_tcp_xyz_m'], 3, 'TCP mesuré')))
                 if 'measured_tcp_xyz_m' in reference else None)
    if joint_error is None and tcp_error is None:
        raise ValueError('La pose doit contenir des angles connus ou un TCP mesuré')
    return dict(q_from_encoders_rad=q.tolist(), tcp_from_encoders_m=pose[:3, 3].tolist(),
                max_joint_error_rad=joint_error, tcp_error_m=tcp_error,
                within_reference_tolerances=(joint_error is None or joint_error <= 0.03)
                and (tcp_error is None or tcp_error <= config['position_tolerance_m']),
                hardware_verified=config['hardware_verified'])


def jog_samples(position, delta):
    if type(delta) is not int or not 1 <= abs(delta) <= 16:
        raise ValueError('Jog limité à 1..16 ticks, positifs ou négatifs')
    inp = InputParameter(1)
    inp.current_position, inp.target_position = [float(position)], [float(position+delta)]
    inp.current_velocity = inp.current_acceleration = [0.0]
    inp.target_velocity = inp.target_acceleration = [0.0]
    inp.max_velocity, inp.max_acceleration, inp.max_jerk = [20.0], [40.0], [160.0]
    trajectory = Trajectory(1)
    if Ruckig(1).calculate(inp, trajectory) < Result.Working:
        raise ValueError('Ruckig refuse le jog')
    n = max(1, math.ceil(trajectory.duration/0.02))
    return [(float(t), round(trajectory.at_time(float(t))[0][0]))
            for t in np.linspace(0, trajectory.duration, n+1)]


async def jog(station, joint, delta, live):
    initial = await stable_snapshot(station)
    index = station.ids.index(joint['motor_id'])
    motor = initial['motors'][index]
    samples = jog_samples(motor['ticks'], delta)
    target = motor['ticks']+delta
    if not motor['min_ticks']+20 <= target <= motor['max_ticks']-20:
        raise ValueError('Jog hors limites Station avec marge de 20 ticks')
    report = dict(joint=joint['name'], motor_id=joint['motor_id'], bus=station.bus,
                  start_ticks=motor['ticks'], target_ticks=target, delta_ticks=delta,
                  executed=False, collision_checked=False)
    if not live:
        return report
    if not all(m['torque_on'] for m in initial['motors']):
        raise RuntimeError('Activer et vérifier le couple du bras dans Station avant le jog')
    # Only the selected ID is ever included in a write; the reader still checks all seven.
    writer = copy.copy(station)
    writer.ids = [joint['motor_id']]
    await writer.write(0x2e, [20], 2)
    await writer.write(0x29, [5], 1)
    begin, previous = time.monotonic(), motor['ticks']
    for step, (t, goal) in enumerate(samples):
        await asyncio.sleep(max(0, begin+t-time.monotonic()))
        if time.monotonic()-(begin+t) > 0.1:
            raise RuntimeError('Jog interrompu : boucle en retard')
        state = station.state()[index]
        raw = bytes(station.motors[joint['motor_id']].get_state())
        if (raw[0x28] != 1 or abs(state[0]-previous) > (2 if step == 0 else 12)
                or not state[1]+20 <= goal <= state[2]-20):
            raise RuntimeError('Jog interrompu : couple, suivi ou limites modifiés')
        await writer.write(0x2a, [goal], 2)
        previous = goal
    deadline = time.monotonic()+2
    while time.monotonic() < deadline:
        actual = station.state()[index][0]
        if abs(actual-target) <= 2:
            report.update(executed=True, final_ticks=actual)
            return report
        await asyncio.sleep(0.02)
    raise RuntimeError('Jog : position finale non atteinte')


def nudge_plan(arm, start, delta_mm):
    delta = vector(delta_mm, 3, 'delta XYZ en millimètres')/1000
    if not 0 < np.linalg.norm(delta) <= 0.005+1e-12:
        raise ValueError('Le petit essai XYZ doit être compris entre 0 et 5 mm')
    target = arm.fk(start)[:3, 3]+delta
    plan = arm.plan(start, [target], mode='linear')
    if any(np.max(np.abs(np.asarray(s['q_rad'])-start)) > 0.05 for s in plan['samples']):
        raise ValueError('Variation articulaire > 0.05 rad ; choisir une pose moins singulière')
    return plan


async def online(args, config):
    if args.command == 'discover':
        return await discover(args.sdk, args.host)
    if config['bus'] in ('auto', 'REPLACE_WITH_ARM_BUS_SERIAL', ''):
        raise ValueError('Indiquer le numéro exact du bus du bras dans la configuration')
    async with connection(config, args.sdk, args.host) as station:
        if args.command == 'capture':
            result = await stable_snapshot(station)
            if args.angles_rad is not None:
                model = copy.deepcopy(config)
                model['hardware_verified'] = False
                result['known_q_rad'] = Arm(args.urdf, model).check(args.angles_rad).tolist()
            if args.measured_tcp_xyz is not None:
                result['measured_tcp_xyz_m'] = vector(args.measured_tcp_xyz, 3, 'TCP mesuré').tolist()
            return result
        if args.command == 'jog':
            return await jog(station, config['joints'][JOINTS.index(args.joint)], args.delta_ticks, args.execute)
        cautious = copy.deepcopy(config)
        cautious.update(max_velocity_rad_s=min(config['max_velocity_rad_s'], 0.05),
                        max_acceleration_rad_s2=min(config['max_acceleration_rad_s2'], 0.1),
                        max_jerk_rad_s3=min(config['max_jerk_rad_s3'], 0.3))
        arm = Arm(args.urdf, cautious)
        if args.scene:
            from collision import CollisionWorld
            context = CollisionWorld(args.urdf, arm, args.scene)
        else:
            context = nullcontext(None)
        with context as world:
            arm.collision = world
            if args.execute:
                world.require_hardware_ready()
            start = measured(station, cautious, powered=args.execute, stationary=True)
            plan = await asyncio.to_thread(nudge_plan, arm, start, args.delta_mm)
            actual = None
            if args.execute:
                actual = await execute(station, arm, plan)
            return dict(executed=args.execute, collision_checked=bool(world), start_q_rad=start.tolist(),
                        target_xyz_m=plan['final_xyz_m'], duration_s=plan['samples'][-1]['t_s'],
                        measured_q_rad=actual.tolist() if actual is not None else None,
                        measured_xyz_from_encoders_m=arm.fk(actual)[:3, 3].tolist() if actual is not None else None)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=HERE/'elrobot.example.json')
    p.add_argument('--sdk', type=Path, default=HERE/'.sdk')
    p.add_argument('--urdf', type=Path, default=HERE/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf')
    p.add_argument('--host', default='localhost')
    p.add_argument('--scene', type=Path, help='Scène de collision, obligatoire pour nudge --execute')
    p.add_argument('--output', type=Path, help='Nouveau fichier JSON ; aucun écrasement')
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('discover', help='Lister tous les bus/IDs, lecture seule')
    capture = sub.add_parser('capture', help='Enregistrer une pose stable, lecture seule')
    capture.add_argument('--angles-rad', nargs=7, type=float, help='Angles URDF MESURÉS, pas supposés')
    capture.add_argument('--measured-tcp-xyz', nargs=3, type=float, help='Position TCP MESURÉE dans base_link (m)')
    for name in ('fit-calibration', 'fit-tcp'):
        fit = sub.add_parser(name, help='Calcul hors ligne depuis des captures')
        fit.add_argument('--references', type=Path, nargs='+', required=True)
    check = sub.add_parser('check-pose', help='Comparer un candidat à une pose indépendante, hors ligne')
    check.add_argument('--reference', type=Path, required=True)
    jog_parser = sub.add_parser('jog', help='Un seul moteur, au plus 16 ticks (~1.4 degré)')
    jog_parser.add_argument('--joint', choices=JOINTS, required=True)
    jog_parser.add_argument('--delta-ticks', type=int, default=8)
    jog_parser.add_argument('--execute', action='store_true')
    nudge = sub.add_parser('nudge', help='Petit déplacement XYZ dans base_link, au plus 5 mm')
    nudge.add_argument('--delta-mm', type=float, nargs=3, required=True)
    nudge.add_argument('--execute', action='store_true')
    args = p.parse_args()
    if args.command == 'nudge' and args.execute and not args.scene:
        p.error('nudge --execute exige --scene')
    try:
        if args.output and args.output.exists():
            raise ValueError('Le fichier de sortie existe déjà ; choisir un nouveau nom')
        config = load_config(args.config)
        if args.command == 'check-pose':
            result = check_pose(config, json.loads(args.reference.read_text()), args.urdf)
        elif args.command in ('fit-calibration', 'fit-tcp'):
            references = [json.loads(path.read_text()) for path in args.references]
            function = fit_calibration if args.command == 'fit-calibration' else fit_tcp
            result = function(config, references, args.urdf)
        else:
            result = asyncio.run(online(args, config))
        data = json.dumps(result, indent=2, allow_nan=False)+'\n'
        if args.output:
            with args.output.open('x') as f:
                f.write(data)
        print(data, end='')
    except KeyboardInterrupt:
        p.exit(130, 'Interrompu ; dernière consigne et couple conservés.\n')
    except (ValueError, RuntimeError, OSError, ImportError, KeyError, TypeError, asyncio.TimeoutError) as exc:
        p.exit(1, f'Erreur : {exc}\n')


if __name__ == '__main__':
    main()
