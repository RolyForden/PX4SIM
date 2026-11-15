"""根据实测 X/H/V 旋翼位姿计算推力效能矩阵。

这是离线检查，不是 PX4 实时分配器，也不是飞行可行性证明。
"""
import itertools
import json
import math
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / 'config' / 'morph-config.json').read_text(encoding='utf-8'))
SEQUENCE = json.loads((ROOT / 'results' / 'sequence.json').read_text(encoding='utf-8'))


def rotate_axis(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def arm_com_after(arm, mode_name):
    joint = next(s['actual_deg'] for s in SEQUENCE['sequence'] if s['mode'] == mode_name)
    yaw = math.radians(joint[f"yaw_{arm['index']}"])
    pitch = math.radians(joint[f"pitch_{arm['index']}"])
    hinge = np.asarray(arm['hinge_flu_m'], dtype=float)
    com0 = np.asarray(arm['com_flu_m'], dtype=float)
    R = rotate_axis([0, 0, 1], yaw) @ rotate_axis(arm['pitch_axis'], pitch)
    return hinge + R @ (com0 - hinge)


def total_com(mode_name):
    body = CONFIG['body']
    mass = body['mass_kg'] * np.asarray(body['com_flu_m'], dtype=float)
    for arm in CONFIG['arms']:
        mass += arm['mass_kg'] * arm_com_after(arm, mode_name)
    for rotor in CONFIG['rotors']:
        mass += CONFIG['rotor_mass_kg'] * np.asarray(rotor['position_flu_m'], dtype=float)
    return mass / CONFIG['total_mass_kg']


def allocate(B, target, lower, upper, weights):
    B = np.asarray(B, dtype=float)
    target = np.asarray(target, dtype=float)
    n = B.shape[1]
    lower = np.broadcast_to(lower, (n,))
    upper = np.broadcast_to(upper, (n,))
    weights = np.asarray(weights, dtype=float)
    best = None
    for active in itertools.product([-1, 0, 1], repeat=n):
        active = np.array(active)
        fixed = active != 0
        free = ~fixed
        t = np.where(active < 0, lower, upper).copy()
        rhs = target - B[:, fixed] @ t[fixed]
        if free.any():
            t[free] = np.linalg.lstsq(weights[:, None] * B[:, free], weights * rhs, rcond=None)[0]
        if np.any(t < lower - 1e-9) or np.any(t > upper + 1e-9):
            continue
        residual = B @ t - target
        score = float(np.linalg.norm(weights * residual))
        if best is None or score < best[0] - 1e-12:
            best = (score, t.copy(), residual)
    if best is None:
        raise ValueError('Invalid thrust bounds')
    return best


def main():
    entries = {s['mode']: s for s in SEQUENCE['sequence']}
    max_thrust = CONFIG['rotors'][0]['motor_constant_N_per_rad_s2'] * CONFIG['rotors'][0]['max_rad_s'] ** 2
    weights = np.array([5, 5, 5, 1, 1, 1.0])
    output = {'status': 'offline allocation check from measured bench poses; not PX4 realtime allocation',
              'max_thrust_per_rotor_N': max_thrust, 'weights': weights.tolist(), 'modes': {}}
    for mode_name in ('X', 'H', 'V'):
        state = entries[mode_name]
        com_flu = total_com(mode_name)
        com = np.array([com_flu[0], -com_flu[1], -com_flu[2]])
        rotors = state['rotors']
        r = np.array([x['position_frd_m'] for x in rotors], dtype=float)
        d = np.array([x['axis_frd'] for x in rotors], dtype=float)
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
        km = np.array([CONFIG['rotors'][i]['km_m'] for i in range(4)], dtype=float)
        B = np.vstack([(np.cross(r - com, d) - km[:, None] * d).T, d.T])
        target = np.array([0, 0, 0, 0, 0, -CONFIG['total_mass_kg'] * 9.8])
        score, thrust, residual = allocate(B, target, 0, max_thrust, weights)
        output['modes'][mode_name] = {
            'com_flu_m': com_flu.tolist(),
            'com_frd_m': com.tolist(),
            'B_tau_xyz_force_xyz_per_N': B.tolist(),
            'rank': int(np.linalg.matrix_rank(B)),
            'condition_number': float(np.linalg.cond(B)),
            'hover_thrust_N': thrust.tolist(),
            'max_thrust_margin_N': float(max_thrust - max(thrust)),
            'hover_residual': residual.tolist(),
            'hover_weighted_residual_norm': score,
            'max_error_deg': state['max_error_deg'],
        }
    (ROOT / 'results' / 'allocation.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: {'rank': v['rank'], 'hover_thrust_N': [round(x, 4) for x in v['hover_thrust_N']],
                          'residual': round(v['hover_weighted_residual_norm'], 6)}
                      for k, v in output['modes'].items()}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
