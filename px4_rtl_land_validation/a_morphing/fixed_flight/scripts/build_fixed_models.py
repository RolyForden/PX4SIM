"""生成固定 H/V 小角构形的独立 PX4/Gazebo 飞行模型。

这是基于 A 近似质量和几何的固定构形演示，不是最终样机，也不是实机参数。
"""
import copy
import json
import math
import pathlib
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
MORPH_ROOT = ROOT.parent
CANDIDATE = MORPH_ROOT.parent / 'candidate_flight'
CONFIG = json.loads((ROOT.parent / 'config' / 'morph-config.json').read_text(encoding='utf-8'))
FLIGHT = json.loads((CANDIDATE / 'flight_config.json').read_text(encoding='utf-8'))
STL_DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
ANGLE_DEG = 10.0
AIRFRAME_IDS = {'H': 4998, 'V': 4997}
ROTOR_MASS = CONFIG['rotor_mass_kg']
I3 = np.eye(3)


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


def add(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def vec(a):
    return ' '.join(f'{float(v):.12g}' for v in a)


def rot_axis(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def quat_from_matrix(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def translate_inertia(I, mass, d):
    return I + mass * (float(d @ d) * I3 - np.outer(d, d))


def arm_rotation(arm, mode):
    if mode == 'H':
        angle = math.radians(ANGLE_DEG) * (1 if arm['H']['yaw_rad'] >= 0 else -1)
        return rot_axis([0, 0, 1], angle)
    if mode == 'V':
        return rot_axis(arm['pitch_axis'], math.radians(ANGLE_DEG))
    return I3


def build_mode(mode):
    model_name = f'a_fixed_{mode.lower()}{int(ANGLE_DEG)}'
    model_dir = ROOT / 'models' / model_name
    mesh_dir = model_dir / 'meshes'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    airframe_id = AIRFRAME_IDS[mode]

    body = read_stl(MORPH_ROOT / 'models' / 'a_morph_bench' / 'meshes' / 'body.stl')
    body = body.copy()
    arm_tris = []
    arm_coms = []
    arm_inertias = []
    rotor_positions = []
    rotor_axes = []
    for arm in CONFIG['arms']:
        hinge = np.asarray(arm['hinge_flu_m'], dtype=float)
        R = arm_rotation(arm, mode)
        tri = read_stl(MORPH_ROOT / 'models' / 'a_morph_bench' / 'meshes' / f"arm_{arm['index']}.stl").copy()
        tri['vertices'] = hinge + tri['vertices'] @ R.T
        tri['normal'] = tri['normal'] @ R.T
        arm_tris.append(tri)
        com = hinge + R @ (np.asarray(arm['com_flu_m']) - hinge)
        arm_coms.append(com)
        arm_inertias.append(R @ np.diag(arm['inertia_diag_kg_m2']) @ R.T)
        rotor0 = np.asarray(CONFIG['rotors'][arm['index']]['position_flu_m'], dtype=float)
        rotor_positions.append(hinge + R @ (rotor0 - hinge))
        axis_flu = R @ np.array([0.0, 0.0, 1.0])
        rotor_axes.append(axis_flu)

    merged = np.concatenate([body] + arm_tris)
    write_stl(mesh_dir / 'body.stl', merged, f'{model_name} fixed {mode} body; provisional FLU; NOT flight certified')
    for i in range(4):
        prop = MORPH_ROOT / 'models' / 'a_morph_bench' / 'meshes' / f'prop_{i}.stl'
        (mesh_dir / f'prop_{i}.stl').write_bytes(prop.read_bytes())

    body_mass = CONFIG['body']['mass_kg']
    body_com = np.asarray(CONFIG['body']['com_flu_m'], dtype=float)
    body_inertia = np.diag(CONFIG['body']['inertia_diag_kg_m2'])
    masses = [body_mass] + [a['mass_kg'] for a in CONFIG['arms']] + [ROTOR_MASS] * 4
    coms = [body_com] + arm_coms + rotor_positions
    total_mass = sum(masses)
    total_com = sum((m * c for m, c in zip(masses, coms)), np.zeros(3)) / total_mass
    inertia = translate_inertia(body_inertia, body_mass, body_com - total_com)
    for arm, com, I in zip(CONFIG['arms'], arm_coms, arm_inertias):
        inertia += translate_inertia(I, arm['mass_kg'], com - total_com)
    rotor_point_inertia = []
    for rp in rotor_positions:
        d = rp - total_com
        I = translate_inertia(np.zeros((3, 3)), ROTOR_MASS, d)
        rotor_point_inertia.append(I)
        inertia += I
    base_mass = total_mass - 4 * ROTOR_MASS
    base_inertia = inertia - sum(rotor_point_inertia, np.zeros((3, 3)))
    base_inertia = (base_inertia + base_inertia.T) / 2

    airframe_lines = [
        '#!/bin/sh', f'# A fixed {mode} {ANGLE_DEG:g} deg; approximate fixed-shape flight model',
        '. ${R}etc/init.d/rc.mc_defaults', 'PX4_SIMULATOR=gz',
        f'PX4_GZ_WORLD=candidate_world', f'PX4_SIM_MODEL={model_name}',
        'param set SIM_GZ_EN 1', 'param set CA_AIRFRAME 0', 'param set CA_ROTOR_COUNT 4',
    ]
    for i, (rp, axis) in enumerate(zip(rotor_positions, rotor_axes)):
        rel = rp - total_com
        r_frd = [rel[0], -rel[1], -rel[2]]
        a_frd = [axis[0], -axis[1], -axis[2]]
        cfg = CONFIG['rotors'][i]
        for key, value in zip(('PX', 'PY', 'PZ'), r_frd):
            airframe_lines.append(f'param set CA_ROTOR{i}_{key} {value:.9f}')
        for key, value in zip(('AX', 'AY', 'AZ'), a_frd):
            airframe_lines.append(f'param set CA_ROTOR{i}_{key} {value:.9f}')
        airframe_lines.append(f'param set CA_ROTOR{i}_KM {cfg["km_m"]:.9f}')
        for key, value in [('FUNC', 101 + i), ('MIN', 150), ('MAX', 1000)]:
            airframe_lines.append(f'param set SIM_GZ_EC_{key}{i+1} {value}')
    airframe_lines += ['param set MPC_THR_HOVER 0.5', 'param set NAV_DLL_ACT 2']
    airframe_path = ROOT / 'airframes' / f'{airframe_id}_{model_name}'
    airframe_path.write_text('\n'.join(airframe_lines) + '\n', encoding='utf-8')
    airframe_path.chmod(0o755)

    # Model SDF: one rigid body plus four oriented rotor links.
    candidate_tree = ET.parse(CANDIDATE / 'models' / 'candidate_rigid' / 'model.sdf')
    candidate_base = candidate_tree.find("model/link[@name='base_link']")
    sdf = ET.Element('sdf', version='1.9')
    model = add(sdf, 'model', name=model_name)
    add(model, 'pose', '0 0 0.35 0 0 0')
    add(model, 'self_collide', 'false')
    base = add(model, 'link', name='base_link')
    inertial = add(base, 'inertial')
    add(inertial, 'mass', base_mass)
    add(inertial, 'pose', vec([*total_com, 0, 0, 0]))
    tensor = add(inertial, 'inertia')
    for key, i, j in [('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2)]:
        add(tensor, key, base_inertia[i, j])
    for sensor in candidate_base.findall('sensor'):
        base.append(copy.deepcopy(sensor))
    visual = add(base, 'visual', name='body')
    geom = add(visual, 'geometry')
    mesh = add(geom, 'mesh')
    add(mesh, 'uri', f'model://{model_name}/meshes/body.stl')
    material = add(visual, 'material')
    add(material, 'ambient', '0.35 0.48 0.62 1')
    add(material, 'diffuse', '0.35 0.48 0.62 1')
    collision = add(base, 'collision', name='body_envelope')
    cg = add(collision, 'geometry')
    box = add(cg, 'box')
    add(box, 'size', '0.35 0.35 0.12')

    plugin_template = candidate_tree.findall('model/plugin')
    for i, (rp, axis) in enumerate(zip(rotor_positions, rotor_axes)):
        cfg = CONFIG['rotors'][i]
        link = add(model, 'link', name=f'rotor_{i}')
        q = quat_from_matrix(arm_rotation(CONFIG['arms'][i], mode))
        add(link, 'pose', f'{vec([*rp, 0, 0, 0]).split()[0]}')
        # Replace the placeholder orientation with a full pose.
        link.find('pose').text = vec([*rp, *quat_as_rpy(q)])
        li = add(link, 'inertial')
        add(li, 'mass', ROTOR_MASS)
        it = add(li, 'inertia')
        add(it, 'ixx', 1e-5); add(it, 'iyy', 1e-5); add(it, 'izz', 2e-5)
        lv = add(link, 'visual', name='prop')
        lg = add(lv, 'geometry')
        lm = add(lg, 'mesh')
        add(lm, 'uri', f'model://{model_name}/meshes/prop_{i}.stl')
        mat = add(lv, 'material')
        add(mat, 'ambient', '0.9 0.44 0.14 1')
        add(mat, 'diffuse', '0.9 0.44 0.14 1')
        joint = add(model, 'joint', name=f'rotor_{i}_joint', type='revolute')
        add(joint, 'parent', 'base_link')
        add(joint, 'child', f'rotor_{i}')
        ax = add(joint, 'axis')
        add(ax, 'xyz', vec(axis))
        lim = add(ax, 'limit')
        add(lim, 'lower', -1e16); add(lim, 'upper', 1e16)
        plugin = copy.deepcopy(plugin_template[i])
        plugin.find('jointName').text = f'rotor_{i}_joint'
        plugin.find('linkName').text = f'rotor_{i}'
        plugin.find('motorNumber').text = str(i)
        if plugin.find('turningDirection') is not None:
            plugin.find('turningDirection').text = cfg['spin']
        model.append(plugin)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(model_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (model_dir / 'model.config').write_text(
        f'<?xml version="1.0"?>\n<model><name>{model_name}</name><version>0.1</version>'
        f'<sdf version="1.9">model.sdf</sdf><description>A fixed {mode} {ANGLE_DEG:g} degree approximate flight model; not final aircraft</description></model>\n',
        encoding='utf-8')

    out = {
        'status': 'fixed-shape approximate A model; NOT final aircraft or flight validation',
        'model': model_name, 'airframe_id': airframe_id, 'mode': mode, 'angle_deg': ANGLE_DEG,
        'source': {'config': 'a_morphing/config/morph-config.json', 'mass_source': FLIGHT.get('mass_properties_status')},
        'mass_kg': total_mass, 'com_flu_m': total_com.tolist(),
        'inertia_com_flu_kg_m2': inertia.tolist(),
        'rotors': [
            {'index': i, 'position_flu_m': rp.tolist(),
             'position_frd_m': [(rp - total_com)[0], -(rp - total_com)[1], -(rp - total_com)[2]],
             'axis_frd': [a[0], -a[1], -a[2]], 'km_m': CONFIG['rotors'][i]['km_m'],
             'spin': CONFIG['rotors'][i]['spin']}
            for i, (rp, a) in enumerate(zip(rotor_positions, rotor_axes))],
        'assumptions': CONFIG['assumptions'] + [
            f'固定构形角 {ANGLE_DEG:g} 度，只用于首轮小角飞行准备。',
            '质量/惯量由近似机臂分组重算；不是实测整机。',
            '旋翼推力轴按机臂旋转后的几何方向设置，旋向和动力系数仍为借用/示意值。',
        ],
    }
    (ROOT / 'config' / f'{model_name}.json').write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
    return out


def quat_as_rpy(q):
    # SDF pose uses RPY; convert quaternion to roll/pitch/yaw.
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    results = [build_mode(mode) for mode in ('H', 'V')]
    print(json.dumps([{k: r[k] for k in ('model', 'airframe_id', 'mode', 'angle_deg', 'mass_kg')} for r in results], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
