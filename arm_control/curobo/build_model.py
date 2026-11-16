"""Build the cuRobo robot model (collision spheres + self-collision matrix) for the ElRobot arm.

Run with the cuRobo venv:  ~/hack/curobo-src/.venv/bin/python arm_control/curobo/build_model.py
The gripper joints are frozen at the planning scene's opening (gripper_angle_rad = 0 -> all zero),
exactly like collision.py, so cuRobo plans the seven arm joints only."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SIM = HERE.parent / '.sdk/hardware/elrobot/simulation'
URDF = SIM / 'elrobot_follower.urdf'
FROZEN = ('rev_motor_08', 'rev_motor_08_1', 'rev_motor_08_2')


def _T(origin):
    xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def _set(elem, T):
    origin = elem.find('origin')
    if origin is None:
        origin = ET.SubElement(elem, 'origin')
    origin.set('xyz', ' '.join(f'{v:.12g}' for v in T[:3, 3]))
    origin.set('rpy', ' '.join(f'{v:.12g}' for v in Rotation.from_matrix(T[:3, :3]).as_euler('xyz')))


def align_axes(root):
    """cuRobo supports axis-aligned joints only. Rotate each moving joint frame by R so that its axis
    becomes +/-x, y or z, and pre-multiply everything expressed in the child link frame by R^-1:
    the kinematics and geometry are exactly unchanged (only the child link frame is re-expressed)."""
    links = {l.get('name'): l for l in root.findall('link')}
    for joint in root.findall('joint'):
        axis = joint.find('axis')
        if joint.get('type') == 'fixed' or axis is None:
            continue
        a = np.array([float(v) for v in axis.get('xyz').split()])
        a /= np.linalg.norm(a)
        k = int(np.argmax(np.abs(a)))
        e = np.zeros(3)
        e[k] = 1.0
        # R maps e onto a (axis sign kept by choosing e = +axis direction of a)
        e *= np.sign(a[k])
        v, c = np.cross(e, a), float(np.dot(e, a))
        if np.linalg.norm(v) < 1e-12:
            continue
        V = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + V + V @ V / (1 + c)
        Rt = np.eye(4); Rt[:3, :3] = R
        Ri = np.linalg.inv(Rt)
        _set(joint, _T(joint.find('origin')) @ Rt)
        axis.set('xyz', ' '.join(str(int(x)) for x in e))
        child = joint.find('child').get('link')
        for tag in ('visual', 'collision', 'inertial'):
            for g in links[child].findall(tag):
                _set(g, Ri @ _T(g.find('origin')))
        for j2 in root.findall('joint'):
            if j2.find('parent').get('link') == child:
                _set(j2, Ri @ _T(j2.find('origin')))
    return root


def frozen_urdf(out):
    root = ET.parse(URDF).getroot()
    for joint in root.findall('joint'):
        if joint.get('name') in FROZEN:
            joint.set('type', 'fixed')
            for tag in ('limit', 'mimic', 'axis', 'dynamics'):
                for e in joint.findall(tag):
                    joint.remove(e)
    align_axes(root)
    for mesh in root.iter('mesh'):
        mesh.set('filename', str((SIM / mesh.get('filename')).resolve()))
    ET.ElementTree(root).write(out)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--density', type=float, default=1.5)
    ap.add_argument('--output', default=str(HERE / 'elrobot.yml'))
    a = ap.parse_args()
    from curobo.robot_builder import RobotBuilder
    urdf = frozen_urdf(HERE / 'elrobot_arm.generated.urdf')
    b = RobotBuilder(urdf_path=str(urdf), asset_path=str(SIM), tool_frames=['Gripper_Base_v1_1'])
    b.fit_collision_spheres(sphere_density=a.density, compute_metrics=True)
    # MorphIt occasionally collapses on a thin link (Joint_05: 0.3 % coverage); refit it denser.
    for name, m in list((b.link_metrics or {}).items()):
        for n in (16, 24, 32):
            if b.link_metrics[name].coverage >= 0.9:
                break
            b.refit_link_spheres(name, num_spheres=n, compute_metrics=True)
        if b.link_metrics[name].coverage < 0.9:
            raise SystemExit(f'{name}: sphere coverage {b.link_metrics[name].coverage:.1%} < 90 %')
    for name, m in (b.link_metrics or {}).items():
        print(f'{name:28s} n={m.num_spheres:3d} cover={m.coverage*100:5.1f}% protr={m.protrusion_dist_mean*1000:5.2f}mm gap={m.surface_gap_mean*1000:5.2f}mm')
    b.compute_collision_matrix(prune_collisions=True, num_samples=4000)
    cfg = b.build()
    b.save(cfg, a.output)
    print('spheres', b.num_spheres, '->', a.output)


if __name__ == '__main__':
    main()
