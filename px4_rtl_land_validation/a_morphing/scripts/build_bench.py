"""构建 A 无人机近似变构台架。

这是运动学台架，不是最终样机，也不是飞行模型。它把 A 整机按四个机臂做近似分组，
用等效 yaw/pitch 关节表达 H/V 构形；关节位置、限位、质量和角度均为可替换假设。
"""
import csv
import json
import math
import pathlib
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
CANDIDATE = ROOT.parent / 'candidate_flight'
A_PACKAGE = ROOT.parent / 'A-UAV-20261115'
MODEL_DIR = ROOT / 'models' / 'a_morph_bench'
MESH_DIR = MODEL_DIR / 'meshes'
CONFIG_DIR = ROOT / 'config'
WORLD_DIR = ROOT / 'worlds'
for d in (MESH_DIR, CONFIG_DIR, WORLD_DIR):
    d.mkdir(parents=True, exist_ok=True)

STL_DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
ACTIVE_PROPELLERS = {78, 80, 82, 84}
CENTRAL_KEYWORDS = ('激光雷达', '香橙派', '飞控', '电池', '底板', '固定板', '转盘', '舵机', '舵盘', '夹爪', '机身')
HINGE_RADIUS_M = 0.075
HINGE_Z_M = 0.0085
H_ANGLE_DEG = 25.0
V_ANGLE_DEG = 25.0
ROTOR_MASS_KG = 0.009
MESH_STRIDE = {'body': 12, 'arm': 8, 'prop': 3}


def load_json(path):
    return json.loads(pathlib.Path(path).read_text(encoding='utf-8'))


def write_json(path, data):
    pathlib.Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def read_stl(path):
    raw = pathlib.Path(path).read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    return np.frombuffer(raw, dtype=STL_DTYPE, count=count, offset=84).copy()


def write_stl(path, triangles, header):
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    with pathlib.Path(path).open('wb') as f:
        f.write(header.encode('ascii')[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(triangles)))
        f.write(triangles.tobytes())


def cad_to_flu(points_mm, R):
    return (np.asarray(points_mm, dtype=float) @ R.T) / 1000.0


def rotor_poses_from_candidate():
    tree = ET.parse(CANDIDATE / 'models' / 'candidate_rigid' / 'model.sdf')
    poses = {}
    for i in range(4):
        link = tree.find(f"model/link[@name='rotor_{i}']")
        pose = [float(v) for v in link.findtext('pose').split()]
        poses[i] = np.array(pose[:3])
    return poses


def load_mass_by_part(parts):
    mass_by_part = {}
    unmatched = 0.0
    path = A_PACKAGE / 'parameters' / 'component_mass_properties.csv'
    with path.open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        mass = float(row['mass_kg'])
        comp = row['cad_component']
        if comp == 'ASSUMED_CAD_ORIGIN':
            unmatched += mass
            continue
        base = re.sub(r'_\d+\.step(?:-\d+)?$', '', comp.split('/')[-1])
        pos = np.array([float(row['x_m']), float(row['y_m']), float(row['z_m'])]) * 1000.0
        candidates = []
        for part in parts:
            if base in part['name'] or part['name'] in base:
                c = np.asarray(part['uniform_volume_centroid_cad_mm'], dtype=float)
                candidates.append((float(np.linalg.norm(c - pos)), part))
        if not candidates:
            unmatched += mass
            continue
        distance, part = min(candidates, key=lambda item: item[0])
        if distance > 50.0:
            unmatched += mass
            continue
        mass_by_part[part['index']] = mass_by_part.get(part['index'], 0.0) + mass
    return mass_by_part, unmatched


def part_flu_centroid(part, R):
    return cad_to_flu(part['uniform_volume_centroid_cad_mm'], R)


def group_arms(parts, rotor_poses, R):
    groups = {i: [] for i in range(4)}
    for part in parts:
        idx = part['index']
        if idx in ACTIVE_PROPELLERS:
            continue
        name = part['name']
        if any(key in name for key in CENTRAL_KEYWORDS):
            continue
        p = part_flu_centroid(part, R)
        if math.hypot(p[0], p[1]) <= HINGE_RADIUS_M:
            continue
        nearest = min(range(4), key=lambda i: float(np.linalg.norm(p[:2] - rotor_poses[i][:2])))
        groups[nearest].append(idx)
    return groups


def arm_geometry(ids, parts, mass_by_part, R):
    mass = sum(mass_by_part.get(i, 0.0) for i in ids)
    if mass > 0:
        com = sum((mass_by_part.get(i, 0.0) * part_flu_centroid(parts[i], R) for i in ids), np.zeros(3)) / mass
    else:
        com = sum((part_flu_centroid(parts[i], R) for i in ids), np.zeros(3)) / max(len(ids), 1)
    inertia = np.zeros(3)
    for i in ids:
        m = mass_by_part.get(i, 0.0)
        d = part_flu_centroid(parts[i], R) - com
        inertia += m * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    return mass, com, inertia + np.array([1e-5, 1e-5, 1e-5])


def add(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def vec(values):
    return ' '.join(f'{float(v):.12g}' for v in values)


def build_sdf(config, body_mesh, arm_meshes, prop_meshes):
    sdf = ET.Element('sdf', version='1.9')
    world = add(sdf, 'world', name='a_morph_bench')
    add(world, 'gravity', '0 0 0')
    physics = add(world, 'physics', name='bench', type='ignored')
    add(physics, 'max_step_size', 0.004)
    add(physics, 'real_time_factor', 1)
    for filename, name in [
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ]:
        add(world, 'plugin', filename=filename, name=name)
    light = add(world, 'light', name='sun', type='directional')
    add(light, 'pose', '0 0 4 0 0 0')
    add(light, 'direction', '-0.5 0.2 -1')
    add(light, 'diffuse', '0.9 0.9 0.9 1')
    scene = add(world, 'scene')
    add(scene, 'ambient', '0.6 0.6 0.6 1')
    add(scene, 'background', '0.85 0.88 0.92 1')
    add(scene, 'grid', 'true')
    floor = add(world, 'model', name='display_floor')
    add(floor, 'static', 'true')
    fl = add(floor, 'link', name='surface')
    add(fl, 'pose', '0 0 -0.10 0 0 0')
    fv = add(fl, 'visual', name='floor')
    fg = add(fv, 'geometry')
    plane = add(fg, 'plane')
    add(plane, 'normal', '0 0 1')
    add(plane, 'size', '4 4')
    fmat = add(fv, 'material')
    add(fmat, 'ambient', '0.7 0.73 0.76 1')
    add(fmat, 'diffuse', '0.7 0.73 0.76 1')

    include = add(world, 'include')
    add(include, 'uri', 'model://a_morph_bench')
    add(include, 'pose', '0 0 0.35 0 0 0')

    model = add(world, 'model', name='_inline_model_placeholder')
    world.remove(model)
    return sdf


def build_model(config, body_mesh, arm_meshes, prop_meshes):
    sdf = ET.Element('sdf', version='1.9')
    model = add(sdf, 'model', name='a_morph_bench')
    add(model, 'self_collide', 'false')

    base = add(model, 'link', name='base_link')
    inertial = add(base, 'inertial')
    add(inertial, 'mass', config['body']['mass_kg'])
    add(inertial, 'pose', vec([*config['body']['com_flu_m'], 0, 0, 0]))
    inertia = add(inertial, 'inertia')
    for key, value in zip(['ixx', 'iyy', 'izz'], config['body']['inertia_diag_kg_m2']):
        add(inertia, key, value)
    visual = add(base, 'visual', name='body')
    geom = add(visual, 'geometry')
    mesh = add(geom, 'mesh')
    add(mesh, 'uri', 'model://a_morph_bench/meshes/body.stl')
    material = add(visual, 'material')
    add(material, 'ambient', '0.32 0.45 0.62 1')
    add(material, 'diffuse', '0.32 0.45 0.62 1')

    fixed = add(model, 'joint', name='bench_fixed', type='fixed')
    add(fixed, 'parent', 'world')
    add(fixed, 'child', 'base_link')

    for arm in config['arms']:
        i = arm['index']
        hinge = np.asarray(arm['hinge_flu_m'], dtype=float)
        yaw_frame = add(model, 'link', name=f'yaw_frame_{i}')
        add(yaw_frame, 'pose', vec([*hinge, 0, 0, 0]))
        yi = add(yaw_frame, 'inertial')
        add(yi, 'mass', 0.001)
        yt = add(yi, 'inertia')
        for key in ('ixx', 'iyy', 'izz'):
            add(yt, key, 1e-7)

        arm_link = add(model, 'link', name=f'arm_{i}')
        add(arm_link, 'pose', vec([*hinge, 0, 0, 0]))
        ai = add(arm_link, 'inertial')
        add(ai, 'mass', arm['mass_kg'])
        add(ai, 'pose', vec([*(np.asarray(arm['com_flu_m']) - hinge), 0, 0, 0]))
        at = add(ai, 'inertia')
        for key, value in zip(('ixx', 'iyy', 'izz'), arm['inertia_diag_kg_m2']):
            add(at, key, value)
        av = add(arm_link, 'visual', name=f'arm_{i}')
        ag = add(av, 'geometry')
        am = add(ag, 'mesh')
        add(am, 'uri', f'model://a_morph_bench/meshes/arm_{i}.stl')
        amat = add(av, 'material')
        add(amat, 'ambient', '0.35 0.57 0.70 1')
        add(amat, 'diffuse', '0.35 0.57 0.70 1')

        yaw_joint = add(model, 'joint', name=f'yaw_{i}', type='revolute')
        add(yaw_joint, 'parent', 'base_link')
        add(yaw_joint, 'child', f'yaw_frame_{i}')
        yaw_axis = add(yaw_joint, 'axis')
        add(yaw_axis, 'xyz', vec(arm['yaw_axis']))
        yaw_limit = add(yaw_axis, 'limit')
        add(yaw_limit, 'lower', arm['limits']['yaw'][0])
        add(yaw_limit, 'upper', arm['limits']['yaw'][1])
        add(yaw_limit, 'effort', 10)
        add(yaw_limit, 'velocity', 0.8)

        pitch_joint = add(model, 'joint', name=f'pitch_{i}', type='revolute')
        add(pitch_joint, 'parent', f'yaw_frame_{i}')
        add(pitch_joint, 'child', f'arm_{i}')
        pitch_axis = add(pitch_joint, 'axis')
        add(pitch_axis, 'xyz', vec(arm['pitch_axis']))
        pitch_limit = add(pitch_axis, 'limit')
        add(pitch_limit, 'lower', arm['limits']['pitch'][0])
        add(pitch_limit, 'upper', arm['limits']['pitch'][1])
        add(pitch_limit, 'effort', 10)
        add(pitch_limit, 'velocity', 0.8)

        rotor = config['rotors'][i]
        rotor_link = add(model, 'link', name=f'rotor_{i}')
        add(rotor_link, 'pose', vec([*rotor['position_flu_m'], 0, 0, 0]))
        ri = add(rotor_link, 'inertial')
        add(ri, 'mass', ROTOR_MASS_KG)
        rt = add(ri, 'inertia')
        add(rt, 'ixx', 1e-5)
        add(rt, 'iyy', 1e-5)
        add(rt, 'izz', 2e-5)
        rv = add(rotor_link, 'visual', name='prop')
        rg = add(rv, 'geometry')
        rm = add(rg, 'mesh')
        add(rm, 'uri', f'model://a_morph_bench/meshes/prop_{i}.stl')
        rmat = add(rv, 'material')
        add(rmat, 'ambient', '0.9 0.44 0.14 1')
        add(rmat, 'diffuse', '0.9 0.44 0.14 1')

        rotor_joint = add(model, 'joint', name=f'rotor_{i}_joint', type='fixed')
        add(rotor_joint, 'parent', f'arm_{i}')
        add(rotor_joint, 'child', f'rotor_{i}')

    for joint in [f'{kind}_{i}' for i in range(4) for kind in ('yaw', 'pitch')]:
        plugin = add(model, 'plugin', filename='gz-sim-joint-position-controller-system',
                     name='gz::sim::systems::JointPositionController')
        add(plugin, 'joint_name', joint)
        add(plugin, 'use_velocity_commands', 'true')
        add(plugin, 'cmd_max', 0.35)
        add(plugin, 'initial_position', 0)
        add(plugin, 'topic', f'/a_uav/joint/{joint}/cmd_pos')

    jsp = add(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
              name='gz::sim::systems::JointStatePublisher')
    add(jsp, 'topic', '/a_uav/joint_states')
    return sdf


def main():
    inventory = load_json(CANDIDATE / 'cad_inventory.json')
    settings = load_json(CANDIDATE / 'settings.json')
    flight = load_json(CANDIDATE / 'flight_config.json')
    parts = inventory['parts']
    R = np.asarray(flight['R_flu_from_cad'], dtype=float)
    rotor_poses = rotor_poses_from_candidate()
    mass_by_part, unmatched_mass = load_mass_by_part(parts)
    arm_groups = group_arms(parts, rotor_poses, R)

    arms = []
    arm_meshes = {}
    total_arm_mass = 0.0
    for i, ids in arm_groups.items():
        mass, com, inertia = arm_geometry(ids, parts, mass_by_part, R)
        total_arm_mass += mass
        pose = rotor_poses[i]
        radial = pose[:2] / np.linalg.norm(pose[:2])
        hinge = np.array([radial[0] * HINGE_RADIUS_M, radial[1] * HINGE_RADIUS_M, HINGE_Z_M])
        sx = 1.0 if pose[0] >= 0 else -1.0
        sy = 1.0 if pose[1] >= 0 else -1.0
        h_yaw = -sx * sy * math.radians(H_ANGLE_DEG)
        pitch_axis = np.array([radial[1], -radial[0], 0.0])
        arms.append({
            'index': i,
            'rotor_index': i,
            'parts': ids,
            'mass_kg': mass,
            'com_flu_m': com.tolist(),
            'inertia_diag_kg_m2': inertia.tolist(),
            'hinge_flu_m': hinge.tolist(),
            'yaw_axis': [0, 0, 1],
            'pitch_axis': pitch_axis.tolist(),
            'limits': {'yaw': [-0.8, 0.8], 'pitch': [-0.10, 0.70]},
            'X': {'yaw_rad': 0.0, 'pitch_rad': 0.0},
            'H': {'yaw_rad': h_yaw, 'pitch_rad': 0.0},
            'V': {'yaw_rad': 0.0, 'pitch_rad': math.radians(V_ANGLE_DEG)},
        })
        triangles = []
        for idx in ids:
            part = parts[idx]
            tri = read_stl(CANDIDATE / part['mesh'])
            tri = tri.copy()
            tri['vertices'] = (tri['vertices'] @ R.T) / 1000.0 - hinge
            tri['normal'] = tri['normal'] @ R.T
            triangles.append(tri)
        arm_meshes[i] = (np.concatenate(triangles)[::MESH_STRIDE['arm']] if triangles else np.empty(0, dtype=STL_DTYPE))

    active_ids = set(ACTIVE_PROPELLERS)
    body_ids = [p['index'] for p in parts if p['index'] not in active_ids and p['index'] not in {i for ids in arm_groups.values() for i in ids}]
    body_triangles = []
    for idx in body_ids:
        tri = read_stl(CANDIDATE / parts[idx]['mesh']).copy()
        tri['vertices'] = (tri['vertices'] @ R.T) / 1000.0
        tri['normal'] = tri['normal'] @ R.T
        body_triangles.append(tri)
    body_mesh = (np.concatenate(body_triangles)[::MESH_STRIDE['body']] if body_triangles else np.empty(0, dtype=STL_DTYPE))

    total_rotor_mass = 4 * ROTOR_MASS_KG
    body_mass = flight['total_mass_kg'] - total_arm_mass - total_rotor_mass
    if body_mass <= 0:
        raise SystemExit('body mass <= 0; mass grouping is invalid')
    total_com = np.asarray(flight['center_of_mass_flu_m'], dtype=float)
    weighted = flight['total_mass_kg'] * total_com
    for arm in arms:
        weighted -= arm['mass_kg'] * np.asarray(arm['com_flu_m'])
        weighted -= ROTOR_MASS_KG * rotor_poses[arm['index']]
    body_com = weighted / body_mass

    rotors = []
    for i in range(4):
        pose = rotor_poses[i]
        src = flight['rotors'][i]
        rotors.append({
            'index': i,
            'position_flu_m': pose.tolist(),
            'position_frd_m': [float(pose[0]), float(-pose[1]), float(-pose[2])],
            'axis_frd': src['axis_frd'],
            'spin': src['spin'],
            'km_m': src['km_m'],
            'motor_constant_N_per_rad_s2': flight['motor_constant_N_per_rad_s2'],
            'max_rad_s': flight['max_rad_s'],
        })

    config = {
        'status': 'A reference morphing bench; approximate joints/mass groups; NOT final aircraft or flight validation',
        'source': {
            'cad_inventory_sha256': inventory['sha256'],
            'flight_config': 'candidate_flight/flight_config.json',
            'mass_properties_json': flight.get('mass_properties_source_sha256'),
            'frame': flight.get('frame_status'),
        },
        'frame': 'FLU; CAD->FLU rotation recorded from candidate_flight/settings.json',
        'R_flu_from_cad': R.tolist(),
        'total_mass_kg': flight['total_mass_kg'],
        'rotor_mass_kg': ROTOR_MASS_KG,
        'body': {
            'mass_kg': body_mass,
            'com_flu_m': body_com.tolist(),
            'inertia_diag_kg_m2': flight['base_inertia_kg_m2'],
            'parts': body_ids,
        },
        'arms': arms,
        'rotors': rotors,
        'unmatched_mass_at_cad_origin_kg': unmatched_mass,
        'hinge_radius_m': HINGE_RADIUS_M,
        'hinge_z_m': HINGE_Z_M,
        'mesh_decimation': {'stride': MESH_STRIDE, 'purpose': 'low-detail visual meshes for repository size; kinematics do not depend on mesh resolution'},
        'H_angle_deg': H_ANGLE_DEG,
        'V_angle_deg': V_ANGLE_DEG,
        'assumptions': [
            '四机臂由 CAD 部件按最近旋翼和径向距离近似分组；不是实际铰链识别。',
            'H 为前后收窄的水平构形，V 为上反角；25 度是可替换演示值，不是实测限位。',
            '等效 yaw+pitch 两个仿真自由度由同一构形输入协调，不代表实物有多个独立舵机。',
            '台架固定、零重力、无碰撞、无电机推力；只验证运动学与反馈链路。',
            '质量来自 BOM 近似；0.393 kg 未定位质量仍在机身，5 个 CAD 实例无 BOM 质量。',
            '机身和机臂惯量是近似值；本台架不用于飞行或动态变构结论。',
        ],
    }

    write_stl(MESH_DIR / 'body.stl', body_mesh, 'A morphing body; provisional FLU; NOT flight model')
    for i, triangles in arm_meshes.items():
        write_stl(MESH_DIR / f'arm_{i}.stl', triangles, f'A morphing arm {i}; provisional FLU')
    for i in range(4):
        prop = CANDIDATE / 'models' / 'candidate_rigid' / 'meshes' / f'prop_{i}.stl'
        tri = read_stl(prop)[::MESH_STRIDE['prop']]
        write_stl(MESH_DIR / f'prop_{i}.stl', tri, f'A morphing prop {i}; low-detail visual')

    write_json(CONFIG_DIR / 'morph-config.json', config)
    model_sdf = build_model(config, body_mesh, arm_meshes, None)
    ET.indent(model_sdf)
    ET.ElementTree(model_sdf).write(MODEL_DIR / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (MODEL_DIR / 'model.config').write_text(
        '<?xml version="1.0"?>\n<model><name>a_morph_bench</name><version>0.1</version>'
        '<sdf version="1.9">model.sdf</sdf><description>A morphing kinematic bench; approximate, not flight model</description></model>\n',
        encoding='utf-8')
    world_sdf = build_sdf(config, body_mesh, arm_meshes, None)
    ET.indent(world_sdf)
    ET.ElementTree(world_sdf).write(WORLD_DIR / 'a_morph_bench.sdf', encoding='utf-8', xml_declaration=True)
    print(json.dumps({
        'model': str(MODEL_DIR / 'model.sdf'),
        'world': str(WORLD_DIR / 'a_morph_bench.sdf'),
        'body_mass_kg': round(body_mass, 6),
        'arm_masses_kg': [round(a['mass_kg'], 6) for a in arms],
        'unmatched_mass_kg': round(unmatched_mass, 6),
        'total_mass_check_kg': round(body_mass + total_arm_mass + total_rotor_mass + 0.0, 6),
        'arm_parts': {a['index']: len(a['parts']) for a in arms},
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
