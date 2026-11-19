"""Plot the accepted runs and retain the initial sensor diagnosis as a separate case."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
R=Path(__file__).resolve().parent;O=R/'results'
names=['empty_imu5000','load50g_01'];summaries=[];manifest=[]
fig,ax=plt.subplots(5,2,figsize=(13,13),sharex='col',constrained_layout=True)
for col,name in enumerate(names):
 d=R/'runs'/name;s=json.loads((d/'summary.json').read_text());assert s['passed'];summaries.append(s)
 o=json.loads((d/'outcome.json').read_text());ev={e['event']:e for e in o['events']};ulog=d/o['ulog']['file'];assert hashlib.sha256(ulog.read_bytes()).hexdigest()==o['ulog']['sha256']
 manifest.append({'trial':name,'role':'accepted comparison','file':str(ulog.relative_to(R)),**{k:v for k,v in o['ulog'].items() if k!='file'}})
 u=ULog(str(ulog));p=u.get_dataset('vehicle_local_position').data;a=u.get_dataset('vehicle_attitude').data;arm=[json.loads(x) for x in (d/'arm-feedback.jsonl').read_text().splitlines()];origin=ev['origin']['origin'];t0=ev['motion_start']['boot_ms']/1000;end=(ev['motion_complete']['boot_ms']/1000)-t0
 tp=p['timestamp']/1e6-t0;ta=a['timestamp']/1e6-t0
 q=np.array([a[f'q[{i}]'] for i in range(4)]);roll=np.degrees(np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2)));pitch=np.degrees(np.arcsin(np.clip(2*(q[0]*q[2]-q[3]*q[1]),-1,1)))
 ar=[v for v in arm if ev['baseline_start']['arm_sim_s']<=v['sim_s']<=ev['recovery_complete']['arm_sim_s']];tt=np.array([v['sim_s']-ev['motion_start']['arm_sim_s'] for v in ar]);w=np.array([v['mount_wrench_n_nm'] for v in ar]);pos=np.array([v['position_m'] for v in ar]);home=json.loads((R/'config.json').read_text())['home_m']
 pm=(tp>=-10)&(tp<=end+10);am=(ta>=-10)&(ta<=end+10)
 ax[0,col].plot(tp[pm],(origin['z']-p['z']-3)[pm],label='PX4 height error')
 ground=min(arm,key=lambda v:abs(v['sim_s']-ev['ground_constraints_settled']['arm_sim_s']))['base_world_m'][2]
 ax[0,col].plot(tt,[v['base_world_m'][2]-ground-3 for v in ar],ls='--',label='Gazebo truth height error')
 ax[1,col].plot(tp[pm],np.hypot(p['x']-origin['x'],p['y']-origin['y'])[pm],label='Horizontal displacement')
 ax[2,col].plot(ta[am],roll[am],label='Roll');ax[2,col].plot(ta[am],pitch[am],label='Pitch')
 for j,label in enumerate('XYZ'):ax[3,col].plot(tt,(pos[:,j]-home[j])*1000,label=label)
 for j,label in enumerate(['Mx','My','Mz']):ax[4,col].plot(tt,w[:,3+j],label=label)
 for j in range(5):
  ax[j,col].axvspan(0,end,color='tab:orange',alpha=.10,label='Arm motion' if j==0 else None)
  ax[j,col].set_xlim(-10,end+10);ax[j,col].grid(alpha=.25);ax[j,col].legend(loc='upper right',fontsize=8)
 ax[0,col].set_title('Empty' if s['payload_kg']==0 else '+50 g')
 ax[4,col].set_xlabel('Simulation seconds from motion start')
for j,label in enumerate(['Height error (m)','Horizontal offset (m)','Attitude (deg)','Arm displacement (mm)','Mount moment (N m)']):ax[j,0].set_ylabel(label)
fig.savefig(O/'comparison.png',dpi=160);plt.close(fig)
diag=json.loads((R/'runs/empty_01/summary.json').read_text())
fig,ax=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
for name,color,label in [('empty_01','tab:red','250 Hz IMU'),('empty_imu5000','tab:blue','5000 Hz IMU')]:
 d=R/'runs'/name;o=json.loads((d/'outcome.json').read_text());ev={e['event']:e for e in o['events']};u=ULog(str(d/o['ulog']['file']));p=u.get_dataset('vehicle_local_position').data;a=[json.loads(v) for v in (d/'arm-feedback.jsonl').read_text().splitlines()];t0=ev['land_trigger']['boot_ms']/1000;gt=np.array([[v['sim_s'],v['base_world_m'][2]] for v in a]);ground=np.interp(ev['ground_constraints_settled']['arm_sim_s'],gt[:,0],gt[:,1]);t=p['timestamp']/1e6;mask=(t>=t0)&(t<=ev['auto_disarmed']['boot_ms']/1000);truth=np.interp(t[mask],gt[:,0],gt[:,1])-ground;estimate=ev['origin']['origin']['z']-p['z'][mask]
 ax[0].plot(t[mask]-t0,estimate,color=color,label=label+' estimate');ax[0].plot(t[mask]-t0,truth,color=color,ls='--',label=label+' truth')
 ax[1].plot(t[mask]-t0,estimate-truth,color=color,label=label)
ax[0].set(title='Landing height',ylabel='Height above initial ground (m)',xlabel='Simulation seconds from Land')
ax[1].set(title='Landing estimator deviation',ylabel='Estimate minus truth (m)',xlabel='Simulation seconds from Land')
for a in ax:a.grid(alpha=.25);a.legend(fontsize=8)
fig.savefig(O/'landing-sampling.png',dpi=160);plt.close(fig)
d=R/'runs/empty_01';o=json.loads((d/'outcome.json').read_text());manifest.append({'trial':'empty_01','role':'diagnostic run, rejected landing estimate','file':str((d/o['ulog']['file']).relative_to(R)),**{k:v for k,v in o['ulog'].items() if k!='file'}})
(O/'comparison.json').write_text(json.dumps({'accepted':summaries,'diagnostic':diag},indent=2))
(O/'ulog-manifest.json').write_text(json.dumps(manifest,indent=2))
(O/'SHA256SUMS.txt').write_text(''.join(f"{v['sha256']}  {v['file']}\n" for v in manifest))
print('Compared accepted empty/load runs; initial diagnostic evidence retained.')
