"""Reuse the repository's ST3215 transport; add arm mapping and execution guards."""
import asyncio
import copy
import importlib.util
import logging
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path

import numpy as np

from planner import joints_to_ticks, require_calibration, ticks_to_joints

_spec = importlib.util.spec_from_file_location(
    'hand_control_transport', Path(__file__).resolve().parents[1] / 'hand_control/control.py')
_transport = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_transport)
Station = _transport.Station  # existing freshness, calibration, bus/error and sync-write handling


@asynccontextmanager
async def connection(config, sdk, host):
    if not (Path(sdk) / 'software/station/shared/station_py').is_dir():
        raise ValueError('SDK NormaCore absent ; utiliser setup.sh ou --sdk')
    sys.path.insert(0, str(Path(sdk).resolve()))
    from software.station.shared.station_py import new_station_client, send_commands
    from target.gen_python.protobuf.drivers.st3215 import st3215
    from target.gen_python.protobuf.station import commands, drivers
    client = await asyncio.wait_for(new_station_client(host, logging.getLogger('arm')), 10)
    station = Station(client, st3215, commands, drivers, send_commands, config['bus'],
                      [j['motor_id'] for j in config['joints']])
    worker = asyncio.create_task(station.read())
    try:
        for _ in range(150):
            if worker.done():
                worker.result()
            if station.received:
                break
            await asyncio.sleep(0.02)
        else:
            raise RuntimeError('Aucun état Station reçu')
        await fresh_feedback(station)
        yield station
    finally:
        worker.cancel()
        await asyncio.gather(worker, return_exceptions=True)
        client.close_connection()


async def fresh_feedback(station, timeout=3.0):
    """Yield to the reader after CPU/file work; never relax sample-age checks."""
    deadline = time.monotonic()+timeout
    while True:
        await asyncio.sleep(.02)
        try:
            return station.state()
        except RuntimeError as exc:
            if 'périm' not in str(exc):
                raise
            if time.monotonic() >= deadline:
                raise RuntimeError('Fresh motor feedback did not resume; no motion started') from exc


def measured(station, config, *, powered=False, stationary=False):
    require_calibration(config)
    states = station.state()
    for j, (pos, lo, hi) in zip(config['joints'], states):
        if j['min_ticks'] < lo or j['max_ticks'] > hi:
            raise RuntimeError(f"{j['name']}: limites configurées hors calibration Station")
        raw = bytes(station.motors[j['motor_id']].get_state())
        if powered and raw[0x28] != 1:
            raise RuntimeError('Couple inactif : préparer le bras dans Station avant exécution')
        if stationary and (int.from_bytes(raw[0x3a:0x3c], 'little') & 0x7fff) > 10:
            raise RuntimeError('Le bras doit être immobile au départ')
    return ticks_to_joints(config, [s[0] for s in states])


async def execute(station, arm, plan, progress=None, paced=False, command_period=None):
    progress = progress or (lambda message: None)
    progress('Checking calibrated targets and quantized collision path')
    c = arm.c
    world = arm.collision
    if world is None or not plan.get('collision_checked'):
        raise ValueError('Exécution refusée sans scène et trajectoire contrôlées pour les collisions')
    await asyncio.to_thread(world.require_hardware_ready)
    # Validate every quantized setpoint BEFORE any write.
    goals = [joints_to_ticks(c, s['q_rad']) for s in plan['samples']]
    for goal in goals:
        arm.check(ticks_to_joints(c, goal))
    quantized = copy.deepcopy(plan)
    for sample, goal in zip(quantized['samples'], goals):
        sample['q_rad'] = ticks_to_joints(c, goal).tolist()
    await asyncio.to_thread(world.validate_plan, quantized)
    await asyncio.to_thread(world.require_hardware_ready)
    await fresh_feedback(station)
    actual = measured(station, c, powered=True, stationary=True)
    progress('Checking that the real arm still matches the simulation start')
    previous = np.asarray(plan['samples'][0]['q_rad'])
    if np.max(np.abs(actual-previous)) > 0.02:
        raise RuntimeError('Le bras a bougé pendant le calcul ; recalculer le plan')
    if not world.motion_valid(actual, quantized['samples'][0]['q_rad']):
        raise RuntimeError('Départ mesuré en collision')
    quantized_final = arm.fk(ticks_to_joints(c, goals[-1]))[:3, 3]
    if np.linalg.norm(quantized_final-plan['targets_xyz_m'][-1]) > c['position_tolerance_m']:
        raise RuntimeError('Précision insuffisante après conversion encodeur')
    await station.write(0x2e, [c['servo_speed_ticks_s']]*7, 2)
    await station.write(0x29, [c['servo_acceleration']]*7, 1)
    begin = time.monotonic()
    progress('Sending trajectory to real arm')
    commands = list(zip(plan['samples'], goals))
    if command_period is not None and not .02 <= command_period <= .1:
        raise ValueError('Invalid command period')
    if paced or command_period is not None:
        # Use already validated points, never synthesize shortcuts. The full
        # path between these points was collision-checked above. Servo speed
        # and acceleration remain capped; extra checking time becomes a hold.
        selected = [commands[0]]
        for command in commands[1:-1]:
            if command[0]['t_s']-selected[-1][0]['t_s'] >= (.1 if paced else command_period):
                selected.append(command)
        if commands[-1] is not selected[-1]:
            selected.append(commands[-1])
        commands = selected
    for index, (sample, goal) in enumerate(commands):
        if index % 10 == 0:
            progress(f'Real motion: {index + 1}/{len(commands)} checked targets' + (' · paced mode' if paced else ''))
        deadline = (time.monotonic()+.1 if paced else begin+sample['t_s'])
        await asyncio.sleep(max(0, deadline-time.monotonic()))
        if not paced and time.monotonic()-deadline > 0.1:
            raise RuntimeError('Boucle de commande en retard ; arrêt de l’envoi')
        if paced:
            await fresh_feedback(station)
        actual = measured(station, c, powered=True)
        if paced:
            for attempt in range(3):
                checked = actual.copy()
                await asyncio.to_thread(world.require_pose, checked, 'Position mesurée')
                await fresh_feedback(station)
                actual = measured(station, c, powered=True)
                if np.max(np.abs(actual-checked)) <= .005:
                    break
            else:
                raise RuntimeError('Arm moving during collision check; stopped advancing targets')
            # Validate the actual-to-next edge, not only the nominal path.
            target = ticks_to_joints(c, goal)
            if not await asyncio.to_thread(world.motion_valid, actual, target):
                raise RuntimeError('Measured-to-target segment in collision')
            await fresh_feedback(station)
            latest = measured(station, c, powered=True)
            if np.max(np.abs(latest-actual)) > .005:
                raise RuntimeError('Arm moved while validating next step; replan required')
            actual = latest
        else:
            world.require_pose(actual, 'Position mesurée')
        if np.max(np.abs(actual-previous)) > c['tracking_error_rad']:
            raise RuntimeError('Erreur de suivi excessive ; arrêt de l’envoi')
        await station.write(0x2a, goal, 2)
        previous = np.asarray(sample['q_rad'])
    # Check real arrival rather than declaring success after the last send.
    final = np.asarray(plan['samples'][-1]['q_rad'])
    timeout = time.monotonic()+3
    progress('Verifying final arrival')
    while time.monotonic() < timeout:
        q = measured(station, c, powered=True)
        if np.max(np.abs(q-final)) > c['tracking_error_rad']:
            raise RuntimeError('Erreur de suivi à l’arrivée')
        if np.linalg.norm(arm.fk(q)[:3, 3]-plan['targets_xyz_m'][-1]) <= c['position_tolerance_m']:
            return q
        await asyncio.sleep(c['dt_s'])
    raise RuntimeError('Position finale non atteinte dans la tolérance')

# No torque changes on error or exit. The last target may still finish moving.
# A network disconnect cannot guarantee a physical stop.
