"""Compare Gazebo support forces and actuator efforts to independent gravity statics."""
from pathlib import Path
import json,hashlib,argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kinematics import HOME,CONFIG,S,A,rotation,forward
R=Path(__file__).resolve().parent;D=R/'results'
def potential(q,payload,g):
 p=forward(q);u=(.05+payload)*p[2]-payload*.04
 for i,leg in enumerate(CONFIG['legs']):
  es=np.array(leg['elbows_home_m']);ru=rotation(A[i],q[i]);ee=S[i]+(es-S[i])@ru.T;com=S[i]+ru@(.5*(es.mean(axis=0)-S[i]));u+=.03*com[2]
  for k in range(2):u+=.012*.5*(ee[k,2]+p[2]+leg['platform_offsets_m'][k][2])
 return u*g

def expected_torque(payload,g):
 h=1e-5
 return np.array([(potential(np.eye(3)[i]*h,payload,g)-potential(-np.eye(3)[i]*h,payload,g))/(2*h) for i in range(3)])
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--suffix',default='');args=ap.parse_args()
 names=[name+args.suffix for name in ['zero_g','gravity_empty','gravity_payload_50g']];summaries=[json.loads((D/name/'summary.json').read_text()) for name in names];comparison=[]
 fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
 for name,s in zip(names,summaries):
  pred=expected_torque(s['settings']['payload'],s['settings']['gravity']);actual=np.array(s['static_joint_torque_nm']);err=np.max(np.abs(pred-actual));comparison.append({'case':name,'expected_static_torque_nm':pred.tolist(),'measured_static_torque_nm':actual.tolist(),'max_torque_difference_nm':float(err),'statics_check_passed':bool(err<.003)})
  rows=[json.loads(l) for l in (D/name/'feedback.jsonl').read_text().splitlines()];rows=[r for r in rows if r['command_id']>0];t=np.array([r['sim_s'] for r in rows]);t-=t[0];ax[0,0].plot(t,[(r['position_m'][2]-HOME[2])*1000 for r in rows],label=name);ax[0,1].plot(t,[r['tracking_error_m']*1000 for r in rows],label=name);ax[1,0].plot(t,[r['closure_m']*1000 for r in rows],label=name)
 xs=np.arange(3)
 for k,s in enumerate(summaries):ax[1,1].bar(xs+(k-1)*.24,np.abs(s['static_joint_torque_nm']),width=.24,label=names[k])
 ax[0,0].set(title='Actual Z motion',xlabel='Time from trajectory start (s)',ylabel='Z from CAD home (mm)');ax[0,1].set(title='Actual Cartesian tracking',xlabel='Time from trajectory start (s)',ylabel='Tracking error (mm)');ax[1,0].set(title='Compliant closure deformation',xlabel='Time from trajectory start (s)',ylabel='Maximum gap (mm)');ax[1,1].set(title='Measured static holding effort',xlabel='Actuated joint',ylabel='Absolute applied torque (N m)',xticks=xs,xticklabels=['1','2','3'])
 for a in ax.flat:a.legend(fontsize=8);a.grid(alpha=.2)
 fig.suptitle('A Delta fixed-body force closure | gravity and payload comparison');fig.savefig(D/'comparison.png',dpi=150)
 delta=np.array(summaries[2]['static_joint_torque_nm'])-np.array(summaries[1]['static_joint_torque_nm']);force_delta=summaries[2]['static_support_force_n'][2]-summaries[1]['static_support_force_n'][2]
 result={'passed':bool(all(s['passed'] for s in summaries) and all(s['statics_check_passed'] for s in comparison) and abs(force_delta-.05*9.81)<.01),'cases':comparison,'added_payload_kg':.05,'measured_support_force_increment_n':force_delta,'expected_increment_n':.05*9.81,'measured_holding_torque_increment_nm':delta.tolist(),'all_base_displacements_m':[s['max_base_displacement_m'] for s in summaries]}
 (D/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 if not result['passed']:raise SystemExit('Static force/torque comparison failed')
if __name__=='__main__':main()
