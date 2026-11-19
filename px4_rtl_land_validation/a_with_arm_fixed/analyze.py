"""Plot actual mounting loads, CoG movement and total tensor in the integrated bench."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for name in ['empty','payload_50g']:
 rows=[json.loads(l) for l in (R/'results'/name/'feedback.jsonl').read_text().splitlines()];rows=[r for r in rows if r['command_id']>0];t=np.array([r['sim_s'] for r in rows]);t-=t[0];ax[0,0].plot(t,[r['mount_wrench_n_nm'][2] for r in rows],label=name)
rows=[json.loads(l) for l in (R/'results/payload_50g/feedback.jsonl').read_text().splitlines()];rows=[r for r in rows if r['command_id']>0];t=np.array([r['sim_s'] for r in rows]);t-=t[0]
cg=np.array([r['com_flu_m'] for r in rows])*1000
for i,label in enumerate('XYZ'):ax[0,1].plot(t,cg[:,i]-cg[0,i],label=label)
for i,label in enumerate(['Mx','My','Mz']):ax[1,0].plot(t,[r['mount_wrench_n_nm'][3+i] for r in rows],label=label)
for i,label in zip([0,4,8],['Ixx','Iyy','Izz']):ax[1,1].plot(t,[r['inertia_com_flu_kg_m2'][i] for r in rows],label=label)
ax[0,0].set(title='Arm-to-aircraft mounting interface',ylabel='Measured mount Fz (N)');ax[0,1].set(title='Actual whole-vehicle CoG movement, 50 g load',ylabel='CoG change from start (mm)');ax[1,0].set(title='Measured arm mounting torque, 50 g load',ylabel='Torque at mount origin (N m)');ax[1,1].set(title='Actual total inertia about current CoG',ylabel='Inertia (kg m²)')
for a in ax.flat:a.set_xlabel('Simulation time from trajectory start (s)');a.grid(alpha=.2);a.legend(fontsize=8)
fig.suptitle('Existing A aircraft + native Delta arm | fixed body, rotors locked');fig.savefig(R/'results/integration.png',dpi=150)
print('Saved results/integration.png')
