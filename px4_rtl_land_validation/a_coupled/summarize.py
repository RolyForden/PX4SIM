"""Plot complete coupled tests and retain both unsuccessful precursor runs."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
R=Path(__file__).resolve().parent;O=R/'results';accepted=[];manifest=[];matrices={}
fig,axs=plt.subplots(6,2,figsize=(14,15),sharex='col',constrained_layout=True)
for col,name in enumerate(['empty_03','load50g_01']):
 d=R/'runs'/name;s=json.loads((d/'summary.json').read_text());assert s['passed'];accepted.append(s);o=json.loads((d/'outcome.json').read_text());ev={e['event']:e for e in o['events']};u=ULog(str(d/o['ulog']['file']));p=u.get_dataset('vehicle_local_position').data;a=u.get_dataset('vehicle_attitude').data
 arm=[json.loads(v) for v in (d/'arm-feedback.jsonl').read_text().splitlines()];morph=[json.loads(v) for v in (d/'morph-feedback.jsonl').read_text().splitlines()];t0=ev['baseline_start']['boot_ms']/1000;t1=ev['recovery_end']['boot_ms']/1000;pt=p['timestamp']/1e6;at=a['timestamp']/1e6;pm=(pt>=t0)&(pt<=t1);am=(at>=t0)&(at<=t1)
 ar=[v for v in arm if t0<=v['sim_s']<=t1];mr=[v for v in morph if t0<=v['sim_s']<=t1];tt=np.array([v['sim_s']-t0 for v in ar]);tm=np.array([v['sim_s']-t0 for v in mr]);origin=ev['origin']['origin'];ground=min(arm,key=lambda v:abs(v['sim_s']-ev['ground_constraints_settled']['arm_sim_s']))['base_world_m'];bp=np.array([v['base_world_m'] for v in ar])
 axs[0,col].plot(pt[pm]-t0,(origin['z']-p['z'][pm]-5)*100,label='PX4');axs[0,col].plot(tt,(bp[:,2]-ground[2]-5)*100,ls='--',label='Gazebo truth')
 axs[1,col].plot(pt[pm]-t0,np.hypot(p['x'][pm]-origin['x'],p['y'][pm]-origin['y'])*100,label='PX4');axs[1,col].plot(tt,np.linalg.norm(bp[:,:2]-ground[:2],axis=1)*100,ls='--',label='Gazebo truth')
 q=np.array([a[f'q[{i}]'] for i in range(4)]);roll=np.degrees(np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2)));pitch=np.degrees(np.arcsin(np.clip(2*(q[0]*q[2]-q[3]*q[1]),-1,1)))
 axs[2,col].plot(at[am]-t0,roll[am],label='Roll');axs[2,col].plot(at[am]-t0,pitch[am],label='Pitch')
 q=np.array([v['q_deg'] for v in mr]);axs[3,col].plot(tm,np.max(np.abs(q[:,::2]),axis=1),label='H yaw');axs[3,col].plot(tm,np.max(np.abs(q[:,1::2]),axis=1),label='V pitch')
 pos=np.array([v['position_m'] for v in ar]);home=np.array(json.loads((R/'config.json').read_text())['home_m']);w=np.array([v['mount_wrench_n_nm'] for v in ar])
 for i,label in enumerate('XYZ'):axs[4,col].plot(tt,(pos[:,i]-home[i])*1000,label=label)
 axs[5,col].plot(tt,w[:,3],label='Mx');axs[5,col].plot(tt,w[:,4],label='My')
 for label in ['H25_arm','V25_arm','coupled_H','coupled_V']:
  lo=ev[label+'_start']['boot_ms']/1000-t0;hi=ev[label+'_end']['boot_ms']/1000-t0
  for ax in axs[:,col]:ax.axvspan(lo,hi,alpha=.08,color='tab:orange' if label.startswith('coupled') else 'tab:green')
  axs[0,col].text((lo+hi)/2,1.01,label,ha='center',fontsize=7,transform=axs[0,col].get_xaxis_transform())
 for ax in axs[:,col]:ax.grid(alpha=.25);ax.legend(loc='upper right',fontsize=8)
 axs[0,col].set_title('Empty' if s['payload_kg']==0 else '+50 g',pad=25);axs[-1,col].set_xlabel('Simulation time from baseline (s)')
 matrices[name]=[json.loads(f.read_text()) for f in sorted(d.glob('matrix-*.json'))]
for i,label in enumerate(['Height error (cm)','Horizontal offset (cm)','Attitude (deg)','Morph angle (deg)','Arm displacement (mm)','Mount moment (N m)']):axs[i,0].set_ylabel(label)
fig.savefig(O/'comparison.png',dpi=155);plt.close(fig)
for name,role in [('empty_01','allocation failure'),('empty_02','partial-line reader interruption'),('empty_03','accepted empty'),('load50g_01','accepted 50 g')]:
 d=R/'runs'/name;o=json.loads((d/'outcome.json').read_text());u=o['ulog'];assert hashlib.sha256((d/u['file']).read_bytes()).hexdigest()==u['sha256'];manifest.append({'name':name,'role':role,'file':str((d/u['file']).relative_to(R)),'bytes':u['bytes'],'sha256':u['sha256']})
(O/'comparison.json').write_text(json.dumps({'accepted':accepted,'unsuccessful_runs':['empty_01','empty_02']},indent=2));(O/'ulog-manifest.json').write_text(json.dumps(manifest,indent=2));(O/'native-matrices.json').write_text(json.dumps(matrices,indent=2));(O/'SHA256SUMS.txt').write_text(''.join(f"{v['sha256']}  {v['file']}\n" for v in manifest));print('Coupled results and four original ULogs indexed.')
