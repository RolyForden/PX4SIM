from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
R=Path(__file__).resolve().parent;run=json.loads((R/'results/regression.json').read_text());name=run['name']
events=json.loads((R/'logs'/f'{name}_events.json').read_text());e={x['event']:x for x in events};start=e['armed']['boot_ms']/1000;end=e['auto_disarmed']['boot_ms']/1000
u=ULog(str(R/'logs'/f'{name}.ulg'),message_name_filter_list=['vehicle_attitude','vehicle_local_position']);a=u.get_dataset('vehicle_attitude').data;v=u.get_dataset('vehicle_local_position').data
q=np.array([a[f'q[{i}]'] for i in range(4)]).T;roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)));pitch=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)))
records=[]
for f in (R/'sessions').glob('*.jsonl'):
 for line in f.read_text().splitlines():
  try:d=json.loads(line)
  except json.JSONDecodeError:continue
  if abs(d['wall_s']-__import__('datetime').datetime.fromisoformat(e['armed']['utc']).timestamp())<300 and start<=d['sim_s']<=end:records.append(d)
records.sort(key=lambda d:d['sim_s']);t=np.array([x['sim_s'] for x in records])-start;joints=np.array([x['q_deg'] for x in records]);target=np.array([x.get('target_deg',x['q_deg']) for x in records])
fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True)
tv=v['timestamp']/1e6;sel=(tv>=start)&(tv<=end);ax[0].plot(tv[sel]-start,e['origin']['origin']['z']-v['z'][sel]);ax[0].set_ylabel('Height (m)')
ta=a['timestamp']/1e6;sel=(ta>=start)&(ta<=end);ax[1].plot(ta[sel]-start,roll[sel],label='Roll');ax[1].plot(ta[sel]-start,pitch[sel],label='Pitch');ax[1].set_ylabel('Attitude (deg)');ax[1].legend(loc='upper right')
ax[2].plot(t,joints[:,0],label='H joint actual');ax[2].plot(t,joints[:,1],label='V joint actual');ax[2].plot(t,target[:,0],'--',alpha=.5,label='H target');ax[2].plot(t,target[:,1],'--',alpha=.5,label='V target');ax[2].set_ylabel('Arm 0 (deg)');ax[2].legend(loc='upper right',ncol=2);ax[2].set_xlabel('Simulation seconds after arming')
for event in events:
 if event['event']=='morph_trigger':
  x=event['boot_ms']/1000-start
  for axis in ax:axis.axvline(x,color='gray',alpha=.3,lw=.8)
  ax[0].text(x,5.5,event['command'],fontsize=8,rotation=90,va='bottom')
for axis in ax:axis.grid(alpha=.25)
ax[0].set_ylim(-.2,6.5);fig.suptitle('A UAV / PX4 native allocation / H10-V10-H25-V25 flight regression');fig.tight_layout();fig.savefig(R/'results/flight.png',dpi=140)
print('Saved results/flight.png',len(records),'joint feedback records')
