"""Plot recorded Gazebo feedback against Cartesian references."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kinematics import HOME
R=Path(__file__).resolve().parent
rows=[json.loads(l) for l in (R/'results/verification.jsonl').read_text().splitlines()]
t=np.array([s['sim_s'] for s in rows]);t-=t[0]
p=(np.array([s['position_m'] for s in rows])-HOME)*1000;ref=(np.array([s['reference_m'] for s in rows])-HOME)*1000
q=np.degrees([s['q_rad'] for s in rows]);fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for k,name in enumerate('XYZ'):
 ax[0,0].plot(t,p[:,k],label=name);ax[0,0].plot(t,ref[:,k],'--',lw=.8,color=f'C{k}',alpha=.6)
ax[0,0].set(xlabel='Simulation time (s)',ylabel='Displacement from CAD home (mm)',title='Actual solid / reference dashed');ax[0,0].legend()
ax[0,1].plot(ref[:,0],ref[:,1],'--',label='reference');ax[0,1].plot(p[:,0],p[:,1],label='actual');ax[0,1].set(xlabel='X displacement (mm)',ylabel='Y displacement (mm)',title='XY trajectory');ax[0,1].axis('equal');ax[0,1].legend()
for i in range(3):ax[1,0].plot(t,q[:,i],label=f'joint {i+1}')
ax[1,0].set(xlabel='Simulation time (s)',ylabel='Actual joint angle (deg)',title='Three Gazebo actuated joints');ax[1,0].legend()
ax[1,1].plot(t,[s['tracking_error_m']*1000 for s in rows],label='Cartesian tracking');ax[1,1].plot(t,[s['closure_m']*1000 for s in rows],label='rod endpoint closure');ax[1,1].set(xlabel='Simulation time (s)',ylabel='Error (mm)',title='Errors measured from Gazebo poses');ax[1,1].legend()
for a in ax.flat:a.grid(alpha=.25)
fig.suptitle('A UAV Delta manipulator | fixed base | CAD kinematic closure')
fig.savefig(R/'results/tracking.png',dpi=150)
print('Saved results/tracking.png')
