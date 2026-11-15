"""固定构形离线悬停可行性检查；不是 PX4 实时分配器或飞行验证。"""
import itertools
import json
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
MORPH = ROOT.parent
CANDIDATE = MORPH.parent / 'candidate_flight'
OUT = ROOT / 'results' / 'hover-feasibility.json'


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


def candidate_x():
    cfg = json.loads((CANDIDATE / 'flight_config.json').read_text(encoding='utf-8'))
    return {
        'mode': 'X',
        'mass_kg': cfg['total_mass_kg'],
        'com_frd_m': [cfg['center_of_mass_flu_m'][0], -cfg['center_of_mass_flu_m'][1], -cfg['center_of_mass_flu_m'][2]],
        'rotors': [{'position_frd_m': r['position_frd_m'], 'axis_frd': r['axis_frd'], 'km_m': r['km_m']} for r in cfg['rotors']],
        'max_thrust_N': cfg['motor_constant_N_per_rad_s2'] * cfg['max_rad_s'] ** 2,
        'source': 'candidate_flight/flight_config.json (X rigid baseline)',
    }


def fixed(mode):
    path = ROOT / 'config' / f'a_fixed_{mode.lower()}10.json'
    cfg = json.loads(path.read_text(encoding='utf-8'))
    return {
        'mode': mode,
        'mass_kg': cfg['mass_kg'],
        'com_frd_m': [cfg['com_flu_m'][0], -cfg['com_flu_m'][1], -cfg['com_flu_m'][2]],
        'rotors': [{'position_frd_m': r['position_frd_m'], 'axis_frd': r['axis_frd'], 'km_m': r['km_m']} for r in cfg['rotors']],
        'max_thrust_N': 8.54858e-06 * 1000 ** 2,
        'source': str(path.relative_to(ROOT.parent.parent)),
    }


def check(item):
    r = np.array([x['position_frd_m'] for x in item['rotors']], dtype=float)
    d = np.array([x['axis_frd'] for x in item['rotors']], dtype=float)
    d = d / np.linalg.norm(d, axis=1, keepdims=True)
    km = np.array([x['km_m'] for x in item['rotors']], dtype=float)
    com = np.array(item['com_frd_m'], dtype=float)
    B = np.vstack([(np.cross(r, d) - km[:, None] * d).T, d.T])
    max_t = float(item['max_thrust_N'])
    target = np.array([0, 0, 0, 0, 0, -item['mass_kg'] * 9.8])
    weights = np.array([5, 5, 5, 1, 1, 1.0])
    score, thrust, residual = allocate(B, target, 0, max_t, weights)
    return {
        'mode': item['mode'],
        'source': item['source'],
        'mass_kg': item['mass_kg'],
        'B_tau_xyz_force_xyz_per_N': B.tolist(),
        'rank': int(np.linalg.matrix_rank(B)),
        'condition_number': float(np.linalg.cond(B)),
        'hover_thrust_N': thrust.tolist(),
        'hover_residual': residual.tolist(),
        'weighted_residual_norm': score,
        'max_thrust_per_rotor_N': max_t,
        'max_thrust_margin_N': float(max_t - max(thrust)),
        'feasible_under_assumed_limits': bool(score < 0.05 and max(thrust) <= max_t),
    }


def main():
    results = [check(candidate_x()), check(fixed('H')), check(fixed('V'))]
    out = {
        'status': 'offline hover feasibility under assumed mass/thrust; NOT PX4 realtime allocation or flight proof',
        'assumptions': [
            '单桨推力上限 = 借用 motor constant × max rad/s²，不代表真实电机/桨。',
            '输入是旋翼实际推力 N；不是 PWM 或归一化油门。',
            '质量/重心来自 A 近似固定构形配置；不是实测整机。',
        ],
        'modes': {r['mode']: r for r in results},
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({r['mode']: {'rank': r['rank'], 'cond': round(r['condition_number'], 2),
                                  'thrust_N': [round(x, 3) for x in r['hover_thrust_N']],
                                  'margin_N': round(r['max_thrust_margin_N'], 3),
                                  'residual': round(r['weighted_residual_norm'], 6),
                                  'feasible': r['feasible_under_assumed_limits']} for r in results}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
