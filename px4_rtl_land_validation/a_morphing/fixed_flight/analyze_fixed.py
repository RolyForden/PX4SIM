"""分析固定构形真实 ULog；拒绝把失败或不完整测试标成通过。"""
import argparse
import json
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

ROOT = pathlib.Path(__file__).resolve().parent


def analyze(name):
    ulog = ROOT / 'logs' / f'{name}.ulg'
    events = json.loads((ROOT / 'logs' / f'{name}_events.json').read_text(encoding='utf-8'))
    config = json.loads((ROOT / 'logs' / f'{name}_model_config.json').read_text(encoding='utf-8'))
    assert events[-1]['event'] == 'test_complete', 'Cannot mark incomplete/failed run as pass'
    u = ULog(str(ulog))
    pos = u.get_dataset('vehicle_local_position').data
    att = u.get_dataset('vehicle_attitude').data
    land = u.get_dataset('vehicle_land_detected').data
    status = u.get_dataset('vehicle_status').data
    origin = next(e['origin'] for e in events if e['event'] == 'origin')
    t = pos['timestamp'].astype(float) / 1e6
    height = origin['z'] - pos['z']
    dist = np.hypot(pos['x'] - origin['x'], pos['y'] - origin['y'])
    q = np.array([att[f'q[{i}]'] for i in range(4)])
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    ta = att['timestamp'].astype(float) / 1e6
    roll_i = np.interp(t, ta, roll)
    pitch_i = np.interp(t, ta, pitch)
    arm = np.interp(t, status['timestamp'].astype(float) / 1e6, status['arming_state']) > 1.5
    airborne = land['landed'] == 0
    assert airborne.any(), 'No airborne detection'
    first_air = land['timestamp'][np.flatnonzero(airborne)[0]]
    contact_ids = np.flatnonzero((land['timestamp'] > first_air) & (land['landed'] == 1))
    assert len(contact_ids), 'No contact after airborne'
    contact = land['timestamp'][contact_ids[0]] / 1e6
    disarm_ids = np.flatnonzero((status['timestamp'] / 1e6 > contact) & (status['arming_state'] == 1))
    assert len(disarm_ids), 'No disarm after contact'
    disarm = status['timestamp'][disarm_ids[0]] / 1e6
    hover_event = next(e for e in events if e['event'] == 'hover_confirmed')
    land_event = next(e for e in events if e['event'] == 'land_trigger')
    stable = ((abs(height - 5) < .5) & (abs(pos['vz']) < .3) & (dist < 1)
              & (abs(roll_i) < np.radians(10)) & (abs(pitch_i) < np.radians(10)) & arm)
    limit = land_event['boot_ms'] / 1000 + .1
    stable &= t < limit
    best = 0
    start = None
    prev = None
    for ti, ok in zip(t, stable):
        if ok:
            if start is None or (prev is not None and ti - prev > .2):
                start = ti
            best = max(best, ti - start)
            prev = ti
        else:
            start = None
            prev = None
    result = {
        'run': name,
        'status': f'fixed {config["mode"]} {config["angle_deg"]:g} deg approximate flight passed; NOT final-aircraft validation',
        'model': config['model'], 'airframe_id': config['airframe_id'],
        'max_relative_height_m': float(height[arm].max()),
        'max_horizontal_displacement_m': float(dist[arm].max()),
        'longest_stable_hover_sim_s': float(best),
        'land_command_to_contact_sim_s': float(contact - land_event['boot_ms'] / 1000),
        'contact_to_disarm_sim_s': float(disarm - contact),
        'max_abs_roll_pitch_deg_above_1m': [
            float(np.degrees(abs(roll_i[arm & (height > 1)])).max()),
            float(np.degrees(abs(pitch_i[arm & (height > 1)])).max()),
        ],
        'ulog_dropout_count': len(u.dropouts),
        'checks': {'airborne': True, 'hover_event': True, 'stable_hover_5s': bool(best >= 5),
                   'contact_after_airborne': True, 'disarm_after_contact': True, 'complete': True},
        'assumptions': config['assumptions'],
    }
    assert result['checks']['stable_hover_5s'], 'Insufficient stable hover'
    out = ROOT / 'results'
    out.mkdir(exist_ok=True)
    (out / f'{name}_summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    tm = t - t[0]
    axes[0].plot(tm, height, label='ULog relative height')
    sp = u.get_dataset('vehicle_local_position_setpoint').data
    axes[0].plot(sp['timestamp'].astype(float) / 1e6 - t[0], origin['z'] - sp['z'], label='Height setpoint', alpha=.7)
    axes[0].set_ylabel('Height (m)'); axes[0].legend()
    axes[1].plot(tm, pos['vz']); axes[1].set_ylabel('Down velocity (m/s)')
    axes[2].plot(tm, np.degrees(roll_i), label='roll')
    axes[2].plot(tm, np.degrees(pitch_i), label='pitch')
    axes[2].set_ylabel('Angle (deg)'); axes[2].set_xlabel('PX4 simulation time from log start (s)'); axes[2].legend()
    for ax in axes:
        ax.grid(alpha=.25); ax.axvline(contact - t[0], color='green', linestyle='--', alpha=.6)
    fig.suptitle(f'{config["model"]} | fixed {config["mode"]} {config["angle_deg"]:g} deg | actual ULog')
    fig.tight_layout(); fig.savefig(out / f'{name}.png', dpi=140)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    analyze(parser.parse_args().name)
