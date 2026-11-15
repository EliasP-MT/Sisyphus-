#!/usr/bin/env python3
"""Planifier une cible XYZ ElRobot dans base_link ; Station est facultative."""
import argparse
import asyncio
import json
from pathlib import Path

from scipy.spatial.transform import Rotation

from planner import Arm, load_config, require_calibration


async def online(args, c, arm):
    from station import connection, execute, measured
    async with connection(c, args.sdk, args.host) as station:
        if args.command == 'state':
            states = station.state()
            return {'bus': station.bus, 'motors': [dict(motor_id=j['motor_id'],
                    ticks=s[0], min_ticks=s[1], max_ticks=s[2])
                    for j, s in zip(c['joints'], states)]}
        require_calibration(c)
        start = measured(station, c, powered=args.execute, stationary=True)
        plan = await asyncio.to_thread(arm.plan, start, args.via+[args.target], args.mode,
                                       Rotation.from_euler('xyz', args.rpy).as_matrix() if args.rpy else None)
        if args.execute:
            await execute(station, arm, plan)
            plan['execution'] = 'arrived_within_tolerance'
        return plan


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(root/'elrobot.example.json'))
    parser.add_argument('--urdf', default=str(root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'))
    parser.add_argument('--sdk', default=str(root/'.sdk'))
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--scene', type=Path, help='Scène CAD/obstacles, obligatoire pour exécuter')
    parser.add_argument('--preview', type=Path, help='Exporter une vue PNG de la scène (avec --scene)')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('state', help='Lire les encodeurs/calibrations Station, sans écriture')
    fk = sub.add_parser('fk', help='Calculer la pose du TCP hors ligne')
    fk.add_argument('--joints', nargs=7, type=float, required=True, metavar='RAD')
    plan = sub.add_parser('plan', help='Calculer tout le chemin avant toute écriture')
    start = plan.add_mutually_exclusive_group(required=True)
    start.add_argument('--start', nargs=7, type=float, metavar='RAD')
    start.add_argument('--from-station', action='store_true')
    plan.add_argument('--target', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    plan.add_argument('--via', nargs=3, type=float, action='append', default=[], metavar=('X', 'Y', 'Z'))
    plan.add_argument('--rpy', nargs=3, type=float, help='Orientation cible roll/pitch/yaw en radians')
    plan.add_argument('--mode', choices=['joint', 'linear'], default='joint')
    plan.add_argument('--output', type=Path, help='Enregistrer le plan JSON pour inspection')
    plan.add_argument('--execute', action='store_true', help='Envoyer le plan au bras (avec --from-station)')
    args = parser.parse_args()
    if args.command == 'plan' and args.execute and not args.from_station:
        parser.error('--execute exige --from-station, jamais une pose de départ supposée')
    if args.command == 'plan' and args.execute and not args.scene:
        parser.error('--execute exige --scene pour le contrôle de collisions')
    if args.preview and (not args.scene or args.command == 'state'):
        parser.error('--preview exige --scene et une commande fk ou plan')
    world = None
    try:
        c = load_config(args.config)
        arm = None if args.command == 'state' else Arm(args.urdf, c)
        if args.scene and arm:
            from collision import CollisionWorld
            world = CollisionWorld(args.urdf, arm, args.scene)
            arm.collision = world
            if args.command == 'plan' and args.execute:
                world.require_hardware_ready()
        if args.command == 'state' or (args.command == 'plan' and args.from_station):
            result = asyncio.run(online(args, c, arm))
        elif args.command == 'fk':
            result = {'frame': c['frame'], 'tcp_transform': arm.fk(args.joints).tolist()}
            if world:
                result['collision_reason'] = world.collision(args.joints)
                result['collision_checked'] = True
        else:
            rotation = Rotation.from_euler('xyz', args.rpy).as_matrix() if args.rpy else None
            result = arm.plan(args.start, args.via+[args.target], args.mode, rotation)
        if args.preview:
            world.save_preview(args.preview, args.joints if args.command == 'fk' else result['samples'][-1]['q_rad'])
        if args.command == 'plan':
            if args.output:
                args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(json.dumps({k: v for k, v in result.items() if k != 'samples'}, indent=2, allow_nan=False))
            print(f"{len(result['samples'])} points ; durée {result['samples'][-1]['t_s']:.3f} s")
        else:
            print(json.dumps(result, indent=2, allow_nan=False))
    except KeyboardInterrupt:
        parser.exit(130, 'Interrompu ; dernière consigne et couple conservés.\n')
    except (ValueError, RuntimeError, OSError, ImportError, KeyError, TypeError, asyncio.TimeoutError) as exc:
        parser.exit(1, f'Erreur : {exc}\n')
    finally:
        if world:
            world.close()


if __name__ == '__main__':
    main()
