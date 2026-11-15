"""Build MJCF from the official URDF, supplied CAD and passive-roller wheels."""
import copy
import fcntl
import hashlib
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from arm_control.planner import Arm, JOINTS, load_config
from .common import HERE, ROOT, URDF


def fmt(values): return ' '.join(f'{float(x):.10g}' for x in values)


def quat(rpy):
    q = Rotation.from_euler('xyz', rpy).as_quat()
    return fmt(q[[3, 0, 1, 2]])


def arm_model(c, arm_config=None):
    settings = load_config(arm_config or ROOT/'arm_control/elrobot.example.json')
    settings['tcp_xyz_m'] = c['tcp_xyz_m']
    settings['cartesian_step_m'] = min(settings['cartesian_step_m'], .005)
    settings['max_ik_step_rad'] = min(settings['max_ik_step_rad'], .12)
    settings['linear_ik_regularization'] = c['linear_ik_regularization']
    arm = Arm(URDF, settings)
    arm.command_bounds = arm.bounds+np.array([c['joint_limit_margin_rad'], -c['joint_limit_margin_rad']])
    if np.any(arm.command_bounds[:, 0] >= arm.command_bounds[:, 1]): raise ValueError('Marge de butée trop grande')
    for link, bounds in zip([l for l, active in zip(arm.chain.links, arm.chain.active_links_mask) if active], arm.command_bounds):
        link.bounds = tuple(bounds)
    return arm


def read_obj(path):
    vertices, faces = [], []
    for line in Path(path).read_text().splitlines():
        if line.startswith('v '): vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '): faces.append([int(x.split('/')[0])-1 for x in line.split()[1:4]])
    return np.asarray(vertices), np.asarray(faces, dtype=np.int32)


def mailbox_geometry(mailbox):
    name, (x, y, z) = mailbox['name'], mailbox['front_xyz_m']
    width, height, depth = mailbox['slot_width_m'], mailbox['slot_height_m'], mailbox['depth_m']
    parts = [(f'mailbox_{name}_{suffix}', pos, size) for suffix, pos, size in [
        ('left', [x-width/2-.015, y+depth/2, z], [.03, depth, .18]),
        ('right', [x+width/2+.015, y+depth/2, z], [.03, depth, .18]),
        ('bottom', [x, y+depth/2, z-.14], [width, depth, .03]),
        ('front_lower', [x, y-.01, (z-height/2+z-.155)/2], [width, .02, .155-height/2]),
        ('top', [x, y+depth/2, z+height/2+.015], [width, depth, .03]),
        ('back', [x, y+depth+.01, z], [width+.06, .02, .18])]]
    yaw = mailbox.get('yaw_rad', 0)
    r = Rotation.from_euler('z', yaw)
    origin = np.array([x, y, z])
    return [(n, (r.apply(np.asarray(p)-origin)+origin).tolist(), size) for n, p, size in parts]


def convex_parts(source, cache):
    """Reuse CoACD; preserve CAD cavities instead of a single convex hull."""
    import coacd
    coacd.set_log_level('error')
    digest = hashlib.sha256(Path(source).read_bytes()+b'coacd-1.0.5-threshold-.035-v1').hexdigest()[:16]
    directory = cache/digest
    directory.mkdir(parents=True, exist_ok=True)
    with (directory/'build.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return _convex_parts_locked(source, directory, coacd)


def _convex_parts_locked(source, directory, coacd):
    manifest = directory/'parts.json'
    if manifest.exists():
        return [directory/name for name in json.loads(manifest.read_text())]
    v, f = read_obj(source)
    parts = coacd.run_coacd(coacd.Mesh(v, f), threshold=0.035, max_convex_hull=32,
        preprocess_mode='off', resolution=1000, mcts_iterations=30, mcts_nodes=10, mcts_max_depth=2, seed=0)
    paths = []
    for i, (vertices, faces) in enumerate(parts):
        path = directory/f'hull_{i:03d}.obj'
        with path.open('w') as output:
            for vertex in vertices: output.write('v '+fmt(vertex)+'\n')
            for face in faces: output.write('f '+' '.join(str(int(x)+1) for x in face)+'\n')
        paths.append(path)
    manifest.write_text(json.dumps([p.name for p in paths]))
    return paths


def add_wheels(base, actuator, contact, c):
    radius, distance = c['wheel_radius_m'], c['base_radius_m']
    roller_radius = radius*0.16
    for i, angle in enumerate(np.deg2rad(c['wheel_mount_deg'])):
        axis = np.array([np.cos(angle), np.sin(angle), 0])
        tangent = np.array([-np.sin(angle), np.cos(angle), 0])
        position = -distance*axis
        position[2] = radius-c['base_height_m']
        wheel = ET.SubElement(base, 'body', name=f'wheel_{i}', pos=fmt(position))
        ET.SubElement(wheel, 'inertial', mass='0.05', pos='0 0 0', diaginertia='0.00006 0.00006 0.00006')
        ET.SubElement(wheel, 'joint', name=f'wheel_joint_{i}', axis=fmt(axis), damping='0.001', armature='0.00002')
        ET.SubElement(wheel, 'geom', type='cylinder', size=fmt([radius*.82, .012]),
            quat=fmt(np.roll(Rotation.align_vectors([axis], [[0, 0, 1]])[0].as_quat(), 1)),
            contype='0', conaffinity='0', group='2', rgba='.12 .12 .14 1', mass='0')
        ET.SubElement(actuator, 'velocity', name=f'wheel_motor_{i}', joint=f'wheel_joint_{i}', kv='1.0',
            ctrlrange=fmt([-c['wheel_speed_rad_s'], c['wheel_speed_rad_s']]),
            forcerange=fmt([-c['wheel_torque_nm'], c['wheel_torque_nm']]))
        for j in range(c['roller_count']):
            phi = 2*np.pi*j/c['roller_count']
            centre = (radius-roller_radius)*(np.cos(phi)*tangent+np.sin(phi)*np.array([0, 0, 1]))
            direction = -np.sin(phi)*tangent+np.cos(phi)*np.array([0, 0, 1])
            roller = ET.SubElement(wheel, 'body', name=f'roller_{i}_{j}', pos=fmt(centre))
            ET.SubElement(roller, 'joint', name=f'roller_joint_{i}_{j}', axis=fmt(direction), damping='0.000002', armature='0.0000001')
            ET.SubElement(roller, 'geom', name=f'roller_geom_{i}_{j}', type='capsule',
                fromto=fmt(np.r_[-direction*.007, direction*.007]), size=str(roller_radius),
                mass='0.003', contype='2', conaffinity='5', group='2', friction='.9 .00002 .00001', rgba='.16 .16 .18 1')
            ET.SubElement(contact, 'exclude', body1='platform', body2=f'roller_{i}_{j}')


def add_arm(base, asset, actuator, contact, equality, c, arm, cache):
    source = ET.parse(URDF).getroot()
    extension = ET.SubElement(source, 'mujoco')
    ET.SubElement(extension, 'compiler', meshdir=str(URDF.parent/'assets'),
                  discardvisual='false', fusestatic='false', balanceinertia='true', boundinertia='1e-8')
    with tempfile.TemporaryDirectory(prefix='postal-urdf-') as directory:
        path = Path(directory)/'arm.urdf'
        ET.ElementTree(source).write(path)
        imported = mujoco.MjModel.from_xml_path(str(path))
        converted = Path(directory)/'arm.xml'
        mujoco.mj_saveLastXML(str(converted), imported)
        xml = ET.parse(converted).getroot()
    for mesh in xml.find('asset'):
        mesh.set('file', str(URDF.parent/'assets'/mesh.get('file')))
        asset.append(mesh)
    mount = ET.SubElement(base, 'body', name='arm_mount', pos=fmt(c['arm_mount_xyz_m']))
    for body in xml.find('worldbody'): mount.append(body)
    for i, name in enumerate(JOINTS):
        joint = mount.find(f".//joint[@name='{name}']")
        joint.set('range', fmt(arm.bounds[i]))
        joint.set('damping', '0.08')
        joint.set('armature', '0.0002')
        joint.set('margin', '.002')
        joint.set('solreflimit', '.004 1')
        ET.SubElement(actuator, 'position', name=f'arm_motor_{i}', joint=name, kp='120', kv='3',
                      ctrlrange=fmt(arm.command_bounds[i]), forcerange='-2.94 2.94')
    ET.SubElement(actuator, 'position', name='gripper_motor', joint='rev_motor_08', kp='15', kv='.15',
                  ctrlrange='0 2.2028', forcerange='-2.94 2.94')
    gear_joint = mount.find(".//joint[@name='rev_motor_08']")
    gear_joint.set('armature', '.0001')
    gear_joint.set('damping', '.01')
    for name, multiplier in [('rev_motor_08_1', -.0115), ('rev_motor_08_2', .0115)]:
        ET.SubElement(equality, 'joint', joint1=name, joint2='rev_motor_08', polycoef=f'0 {multiplier} 0 0 0', solref='.004 1')
    tip = mount.find(".//body[@name='Gripper_Base_v1_1']")
    ET.SubElement(tip, 'site', name='tcp', pos=fmt(c['tcp_xyz_m']), size='.006', rgba='0 1 0 1')
    ET.SubElement(tip, 'body', name='grasp_frame', pos=fmt(c['tcp_xyz_m']))
    for i, geom in enumerate(mount.iter('geom')):
        geom.set('name', f'arm_geom_{i}')
        if geom.get('contype') == '0':
            geom.set('group', '2')
        else:
            geom.set('contype', '4'); geom.set('conaffinity', '5'); geom.set('group', '2')
    # Match the planner's intentional fixed/adjacent/mimic-mechanism exclusions.
    parents, groups = {}, {'base_link': 'base_link'}
    for joint in source.findall('joint'):
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        parents[child] = parent
        groups[child] = groups[parent] if joint.get('type') == 'fixed' else child
    gripper_links = []
    for link in groups:
        current = link
        while current in parents and current != 'Gripper_Base_v1_1': current = parents[current]
        if current == 'Gripper_Base_v1_1':
            groups[link] = groups['Gripper_Base_v1_1']; gripper_links.append(link)
    neighbours = {frozenset((groups[a], groups[b])) for a, b in parents.items()}
    names = list(groups)
    for i, a in enumerate(names):
        for b in names[:i]:
            if groups[a] == groups[b] or frozenset((groups[a], groups[b])) in neighbours:
                ET.SubElement(contact, 'exclude', body1=a, body2=b)
    return tip, gripper_links


def add_cad(base, asset, c, cache):
    directory = ROOT/'arm_control/assets/full_robot_collision'
    manifest = json.loads((directory/'manifest.json').read_text())
    body = ET.SubElement(base, 'body', name='cad_chassis_backpack', pos=fmt(c['cad_xyz_m']), quat=quat(c['cad_rpy_rad']))
    colours = {1: '.24 .27 .30 1', 8: '.72 .75 .78 1', 7: '.55 .58 .62 1',
               4: '.12 .13 .15 1', 3: '.75 .67 .40 1', 6: '.45 .48 .52 1'}
    for solid in manifest['solids']:
        source = directory/solid['mesh']
        visual_name = f'cad_visual_{solid["id"]}'
        ET.SubElement(asset, 'mesh', name=visual_name, file=str(source))
        ET.SubElement(body, 'geom', name=visual_name, type='mesh', mesh=visual_name, contype='0', conaffinity='0',
                      group='2', mass='0', rgba=colours.get(solid['id'], '.65 .68 .72 1'))
        for j, hull in enumerate(convex_parts(source, cache/'convex')):
            name = f'cad_hull_{solid["id"]}_{j}'
            ET.SubElement(asset, 'mesh', name=name, file=str(hull))
            ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name,
                          contype='4', conaffinity='5', group='3', mass='0', rgba='.15 .5 .65 0')


def build(c, arm_config=None, cache=None):
    cache = Path(cache or HERE/'.runtime')
    cache.mkdir(parents=True, exist_ok=True)
    if not URDF.is_file(): raise ValueError('URDF absent : lancer arm_control/setup.sh')
    arm = arm_model(c, arm_config)
    root = ET.Element('mujoco', model='postal_elrobot_kiwi')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true', balanceinertia='true', boundinertia='1e-8')
    ET.SubElement(root, 'option', timestep=str(c['timestep_s']), integrator='implicitfast', cone='elliptic', gravity='0 0 -9.81', iterations='40')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', solref='.008 1', solimp='.9 .95 .001', friction='.8 .005 .0001')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='floor_tex', type='2d', builtin='checker', width='256', height='256', rgb1='.8 .8 .8', rgb2='.7 .7 .7')
    ET.SubElement(asset, 'material', name='floor_mat', texture='floor_tex', texrepeat='8 6')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='1 1 4', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .1', material='floor_mat', group='0', contype='1', conaffinity='6', friction='.9 .00005 .00001')
    actuator, contact, equality = ET.SubElement(root, 'actuator'), ET.SubElement(root, 'contact'), ET.SubElement(root, 'equality')
    base = ET.SubElement(world, 'body', name='platform', pos=fmt([*c['start_pose'][:2], c['base_height_m']]), quat=quat([0, 0, c['start_pose'][2]]))
    ET.SubElement(base, 'freejoint', name='base_free')
    ET.SubElement(base, 'inertial', mass=str(c['base_mass_kg']), pos='0 0 0', diaginertia='.012 .012 .018')
    ET.SubElement(base, 'geom', name='kiwi_deck', type='cylinder', size='.108 .008',
                  mass='0', contype='4', conaffinity='5', group='2', rgba='.18 .22 .25 1')
    ET.SubElement(base, 'geom', name='arm_pedestal', type='cylinder', pos='0 .04 .035', size='.025 .035',
                  mass='0', contype='0', conaffinity='0', group='2', rgba='.4 .42 .45 1')
    for x in (-.05, .05):
        ET.SubElement(base, 'geom', name=f'bag_bracket_{x}', type='box', pos=fmt([x, -.115, .03]),
                      size='.008 .035 .008', mass='0', contype='0', conaffinity='0', group='2', rgba='.3 .32 .35 1')
    ET.SubElement(base, 'site', name='imu', size='.004', rgba='1 0 0 1')
    ET.SubElement(base, 'site', name='lidar', pos=fmt(c['lidar']['xyz_m']), size='.012', rgba='.9 .1 .1 1')
    add_wheels(base, actuator, contact, c)
    tip, gripper_links = add_arm(base, asset, actuator, contact, equality, c, arm, cache)
    add_cad(base, asset, c, cache)
    # The CAD placement is nominal; base/root mounting contacts are intentional.
    for name in ['base_link', 'ST3215_1_v1_1']:
        ET.SubElement(contact, 'exclude', body1='cad_chassis_backpack', body2=name)
    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'gyro', name='gyro', site='imu')
    ET.SubElement(sensor, 'accelerometer', name='accelerometer', site='imu')
    def box(name, position, size, colour='.65 .65 .7 1'):
        return ET.SubElement(world, 'geom', name=name, type='box', pos=fmt(position), size=fmt(np.array(size)/2),
                             rgba=colour, group='0', contype='1', conaffinity='6')
    x, y = c['room_size_m']
    for name, pos, size in [('south', [x/2, -.04, .1], [x+.16, .08, .2]),
                            ('north', [x/2, y+.04, .1], [x+.16, .08, .2]),
                            ('west', [-.04, y/2, .1], [.08, y, .2]),
                            ('east', [x+.04, y/2, .1], [.08, y, .2])]: box(name, pos, size)
    for obstacle in c['obstacles']:
        box(obstacle['name'], [*obstacle['xy_m'], obstacle['size_m'][2]/2], obstacle['size_m'])
    for mailbox in c['mailboxes']:
        for name, pos, size in mailbox_geometry(mailbox): box(name, pos, size, '.25 .55 .25 1')
    # Reachable task fixtures: reference poses define the printed letter support.
    pickup_q = arm.check(c['pickup_joint_rad'])
    pickup = arm.fk(pickup_q)
    for i, letter in enumerate(c['letters']):
        offset = np.asarray(c['letter_center_from_tcp_m'])+np.array([0, 0, .006*i])
        centre = np.asarray(c['arm_mount_xyz_m'])+pickup[:3, 3]+pickup[:3, :3] @ offset
        pose = np.asarray(c['start_pose'][:2])+Rotation.from_euler('z', c['start_pose'][2]).apply(np.r_[centre[:2], 0])[:2]
        orientation = Rotation.from_euler('z', c['start_pose'][2])*Rotation.from_matrix(pickup[:3, :3])
        body = ET.SubElement(world, 'body', name=letter['id'], pos=fmt([*pose, c['base_height_m']+centre[2]]),
                             quat=fmt(np.roll(orientation.as_quat(), 1)))
        ET.SubElement(body, 'freejoint', name=f'{letter["id"]}_free')
        ET.SubElement(body, 'geom', name=f'{letter["id"]}_geom', type='box', size='.05 .05 .001',
                      mass='.005', contype='8', conaffinity='1', group='2', rgba='.95 .85 .5 1')
        ET.SubElement(equality, 'weld', name=f'{letter["id"]}_storage', body1='platform', body2=letter['id'])
        ET.SubElement(equality, 'weld', name=f'{letter["id"]}_grasp', body1='grasp_frame', body2=letter['id'], active='false')
        for link in gripper_links: ET.SubElement(contact, 'exclude', body1=link, body2=letter['id'])
    ET.SubElement(root, 'statistic', center='1.8 1.45 .2', extent='3')
    ET.indent(root)
    text = ET.tostring(root, encoding='unicode')
    key = hashlib.sha256(text.encode()).hexdigest()[:16]
    xml = cache/f'postal-{key}.xml'
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', dir=cache, delete=False) as temporary:
        temporary.write(text)
        temporary_path = Path(temporary.name)
    temporary_path.replace(xml)
    model = mujoco.MjModel.from_xml_path(str(xml))
    return model, arm, xml, pickup_q
