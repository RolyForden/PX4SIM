"""Aggregate the six accepted flights and plot native ULog measurements."""
from pathlib import Path
import json, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

R = Path(__file__).resolve().parent
O = R / 'results'
names = ['empty_01', 'empty_02_retry1', 'empty_03',
         'load50g_01', 'load50g_02', 'load50g_03']
rows, manifest, groups = [], [], {}
fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
for name in names:
    d = R / 'runs' / name
    row = json.loads((d / 'summary.json').read_text())
    assert row['passed'], name
    outcome = json.loads((d / 'outcome.json').read_text())
    events = {e['event']: e for e in outcome['events']}
    path = d / outcome['ulog']['file']
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    assert sha == row['ulog_sha256']
    manifest.append({'trial': name, 'file': str(path.relative_to(R)),
                     'bytes': path.stat().st_size, 'sha256': sha})
    u = ULog(str(path))
    p = u.get_dataset('vehicle_local_position').data
    a = u.get_dataset('vehicle_attitude').data
    m = u.get_dataset('actuator_motors').data
    origin = events['origin']['origin']
    t0 = events['armed']['boot_ms'] / 1000
    hs = events['hover_start']['boot_ms'] / 1000
    he = events['hover_complete']['boot_ms'] / 1000
    disarm = events['auto_disarmed']['boot_ms'] / 1000
    pt, at, mt = [x['timestamp'] / 1e6 for x in [p, a, m]]
    pm = (pt >= hs) & (pt <= he)
    am = (at >= hs) & (at <= he)
    mm = (mt >= hs) & (mt <= he)
    fm = (pt >= t0) & (pt <= disarm)
    controls = np.array([m[f'control[{i}]'][mm] for i in range(4)])
    row['hover_mean_motor_control'] = float(np.mean(controls))
    q = np.array([a[f'q[{i}]'] for i in range(4)])
    roll = np.degrees(np.arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2)))
    pitch = np.degrees(np.arcsin(np.clip(2*(q[0]*q[2]-q[3]*q[1]), -1, 1)))
    color = 'tab:blue' if row['payload_kg'] == 0 else 'tab:orange'
    label = 'Empty' if row['payload_kg'] == 0 else '+50 g'
    label = label if name.endswith('01') else None
    axes[0, 0].plot(pt[fm]-t0, origin['z']-p['z'][fm], color=color, alpha=.65, label=label)
    axes[0, 1].plot(pt[pm]-hs, np.hypot(p['x'][pm]-origin['x'], p['y'][pm]-origin['y']), color=color, alpha=.65, label=label)
    axes[1, 0].plot(at[am]-hs, np.maximum(np.abs(roll[am]), np.abs(pitch[am])), color=color, alpha=.65, label=label)
    axes[1, 1].plot(mt[mm]-hs, controls.mean(axis=0), color=color, alpha=.65, label=label)
    rows.append(row)
for payload, key in [(0, 'empty'), (.05, 'load50g')]:
    subset = [r for r in rows if r['payload_kg'] == payload]
    groups[key] = {'trials': len(subset), 'mass_kg': subset[0]['mass_kg'],
        'mean_hover_height_m': float(np.mean([r['hover_mean_height_m'] for r in subset])),
        'worst_hover_height_error_m': max(r['hover_max_height_error_m'] for r in subset),
        'worst_hover_horizontal_offset_m': max(r['hover_max_horizontal_offset_m'] for r in subset),
        'worst_hover_abs_roll_pitch_deg': max(max(r['hover_max_abs_roll_pitch_deg']) for r in subset),
        'mean_motor_control': float(np.mean([r['hover_mean_motor_control'] for r in subset]))}
for ax, title, ylabel in zip(axes.flat, ['Full flight', 'Hover horizontal displacement', 'Hover attitude', 'Hover motor command'], ['Relative height (m)', 'Distance from takeoff (m)', 'Max |roll|, |pitch| (deg)', 'Mean normalized motor control']):
    ax.set(title=title, xlabel='Simulation time (s)', ylabel=ylabel)
    ax.grid(alpha=.25)
    ax.legend()
axes[0, 0].axhline(2, color='k', lw=.7, ls='--')
fig.savefig(O / 'comparison.png', dpi=170)
plt.close(fig)
report = {'all_passed': all(r['passed'] for r in rows), 'groups': groups, 'trials': rows,
          'startup_failure': {'trial': 'empty_02', 'armed': False, 'ulog': None,
                              'cause': 'Heartbeat check ran before PX4 finished spawning; wait now up to 30 s'}}
(O / 'comparison.json').write_text(json.dumps(report, indent=2))
(O / 'ulog-manifest.json').write_text(json.dumps(manifest, indent=2))
(O / 'SHA256SUMS.txt').write_text(''.join(f"{x['sha256']}  {x['file']}\n" for x in manifest))
print(json.dumps(groups, indent=2))
