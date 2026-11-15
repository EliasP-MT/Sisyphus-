"""Reuse the repository's ST3215 transport; add arm mapping and execution guards."""
import asyncio
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
        yield station
    finally:
        worker.cancel()
        await asyncio.gather(worker, return_exceptions=True)
        client.close_connection()


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


async def execute(station, arm, plan):
    c = arm.c
    # Validate every quantized setpoint BEFORE any write.
    goals = [joints_to_ticks(c, s['q_rad']) for s in plan['samples']]
    for goal in goals:
        arm.check(ticks_to_joints(c, goal))
    actual = measured(station, c, powered=True, stationary=True)
    previous = np.asarray(plan['samples'][0]['q_rad'])
    if np.max(np.abs(actual-previous)) > 0.02:
        raise RuntimeError('Le bras a bougé pendant le calcul ; recalculer le plan')
    quantized_final = arm.fk(ticks_to_joints(c, goals[-1]))[:3, 3]
    if np.linalg.norm(quantized_final-plan['targets_xyz_m'][-1]) > c['position_tolerance_m']:
        raise RuntimeError('Précision insuffisante après conversion encodeur')
    await station.write(0x2e, [c['servo_speed_ticks_s']]*7, 2)
    await station.write(0x29, [c['servo_acceleration']]*7, 1)
    begin = time.monotonic()
    for sample, goal in zip(plan['samples'], goals):
        deadline = begin+sample['t_s']
        await asyncio.sleep(max(0, deadline-time.monotonic()))
        if time.monotonic()-deadline > 0.1:
            raise RuntimeError('Boucle de commande en retard ; arrêt de l’envoi')
        actual = measured(station, c, powered=True)
        if np.max(np.abs(actual-previous)) > c['tracking_error_rad']:
            raise RuntimeError('Erreur de suivi excessive ; arrêt de l’envoi')
        await station.write(0x2a, goal, 2)
        previous = np.asarray(sample['q_rad'])
    # Check real arrival rather than declaring success after the last send.
    final = np.asarray(plan['samples'][-1]['q_rad'])
    timeout = time.monotonic()+3
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
