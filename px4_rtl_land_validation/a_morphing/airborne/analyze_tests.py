"""Summarize completed a_air ULog regressions and their morph events."""
from pathlib import Path
import json,hashlib
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent;results=[]
for p in sorted((R/'logs').glob('*_events.json')):
 events=json.loads(p.read_text());by={e['event']:e for e in events}
 if 'test_complete' not in by:continue
 name=p.name.removesuffix('_events.json');ulog=R/'logs'/(name+'.ulg')
 if not ulog.exists():continue
 u=ULog(str(ulog),message_name_filter_list=['vehicle_attitude','vehicle_local_position'])
 a=u.get_dataset('vehicle_attitude').data;v=u.get_dataset('vehicle_local_position').data
 start=by['armed']['boot_ms']/1000;end=by['auto_disarmed']['boot_ms']/1000;mask=(a['timestamp']/1e6>=start)&(a['timestamp']/1e6<=end);vmask=(v['timestamp']/1e6>=start)&(v['timestamp']/1e6<=end)
 q=np.array([a[f'q[{i}]'] for i in range(4)]).T
 roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)));pitch=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)))
 o=by['origin']['origin'];dist=np.hypot(v['x']-o['x'],v['y']-o['y'])
 results.append({'name':name,'passed':True,'max_height_m':float(o['z']-min(v['z'][vmask])),'max_horizontal_offset_m':float(max(dist[vmask])),'max_abs_roll_pitch_deg':[float(max(abs(roll[mask]))),float(max(abs(pitch[mask])))],'land_to_contact_sim_s':(by['ground_detected']['boot_ms']-by['land_trigger']['boot_ms'])/1000,'contact_to_disarm_sim_s':(by['auto_disarmed']['boot_ms']-by['ground_detected']['boot_ms'])/1000,'morph_reached':[x['event'].removeprefix('morph_reached_') for x in events if x['event'].startswith('morph_reached_')],'ulog_dropout_count':len(u.dropouts),'ulog_sha256':hashlib.sha256(ulog.read_bytes()).hexdigest()})
current=[x for x in results if 'H:25' in x['morph_reached']]
if not current:raise SystemExit('No completed H25/V25 test found')
latest=max(current,key=lambda x:x['name'])
(R/'results').mkdir(exist_ok=True)
(R/'results/regression.json').write_text(json.dumps(latest,indent=2)+'\n');print(json.dumps(latest,indent=2))
