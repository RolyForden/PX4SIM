"""Sensitivity and native-constraint comparison using actual Gazebo traces."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;N=R.parent/'native';names=['baseline','stiffness_1000','stiffness_4000','damping_1p5','damping_6']
def read(p):return json.loads(p.read_text())
def lines(p):return [json.loads(s) for s in p.read_text().splitlines()]
def mount_stats(folder,feedback):
 rows=[r for r in lines(folder/'mount_wrench.jsonl') if r['valid']];first=next(r['sim_s'] for r in feedback if r['busy']);last=max(r['sim_s'] for r in feedback if r['busy']);static=[r for r in rows if first-2<=r['sim_s']<first];dynamic=[r for r in rows if first<=r['sim_s']<=last];f=np.array([r['force_n'] for r in static]).mean(axis=0);m=np.array([r['torque_nm'] for r in static]).mean(axis=0)
 return {'joint_frame':'fixed body joint frame; force at body origin','static_force_n':f.tolist(),'static_torque_nm':m.tolist(),'dynamic_force_peak_from_static_n':float(np.max(np.linalg.norm(np.array([r['force_n'] for r in dynamic])-f,axis=1))),'dynamic_torque_peak_from_static_nm':float(np.max(np.linalg.norm(np.array([r['torque_nm'] for r in dynamic])-m,axis=1)))}
result={'elastic_cases':[],'native_cases':[]}
for name in names:
 s=read(R/name/'summary.json');f=lines(R/name/'feedback.jsonl');entry={'case':name,'K_n_m':s['settings']['stiffness'],'D_ns_m':s['settings']['damping'],'passed':s['passed'],'max_tracking_mm':s['max_tracking_error_m']*1000,'max_closure_mm':s['max_closure_m']*1000,'reason':s.get('error')}
 if s['passed']:entry['fixture_wrench']=mount_stats(R/name,f)
 result['elastic_cases'].append(entry)
for name in ['gravity_empty','payload_50g','payload_50g_rotated']:
 folder=N/'results'/name;s=read(folder/'summary.json');f=lines(folder/'feedback.jsonl');e={'case':name,'passed':s['passed'],'max_tracking_mm':s['max_tracking_error_m']*1000,'max_closure_mm':s['max_closure_m']*1000,'final_home_error_mm':s['final_home_error_m']*1000,'fixture_wrench':mount_stats(folder,f),'extra_proxy_mass_kg':.00006};result['native_cases'].append(e)
baseline=result['elastic_cases'][0];native=result['native_cases'][1]
result['conclusion']={'direct_sdf_closure_supported':False,'runtime_fixed_proxy_closure_supported':True,'selected_backend':'native constraints via ball joints and runtime fixed proxy welds','native_payload_closure_improvement_factor':baseline['max_closure_mm']/native['max_closure_mm'],'soft_stiffness_completion_failed':not result['elastic_cases'][1]['passed'],'native_full_tests_passed':all(s['passed'] for s in result['native_cases']),'flight_tested':False}
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained');labels=['K2000 D3','K1000 D3','K4000 D3','K2000 D1.5','K2000 D6','Native']
entries=result['elastic_cases']+[native];colors=['C0' if e['passed'] else 'C3' for e in entries];ax[0,0].bar(range(6),[e['max_closure_mm'] for e in entries],color=colors);ax[0,0].set(ylabel='Maximum closure error (mm)',title='Physical closure error; red = sequence incomplete');ax[0,1].bar(range(6),[e['max_tracking_mm'] for e in entries],color=colors);ax[0,1].set(ylabel='Maximum Cartesian error (mm)',title='Tracking under the same payload and command sequence')
for a in ax[0]:a.set_xticks(range(6),labels,rotation=25,ha='right')
for folder,label in [(R/'baseline','Elastic K2000 D3'),(N/'results/payload_50g','Native constraints')]:
 feedback=lines(folder/'feedback.jsonl');t0=next(r['sim_s'] for r in feedback if r['busy']);tf=max(r['sim_s'] for r in feedback if r['busy']);rows=[r for r in lines(folder/'mount_wrench.jsonl') if r['valid'] and t0<=r['sim_s']<=tf];t=np.array([r['sim_s']-t0 for r in rows]);ax[1,0].plot(t,[r['force_n'][2] for r in rows],label=label);ax[1,1].plot(t,[r['torque_nm'][1] for r in rows],label=label)
ax[1,0].set(xlabel='Simulation time from command (s)',ylabel='Fixed-body joint Fz (N)',title='Measured fixed-body/world reaction force');ax[1,1].set(xlabel='Simulation time from command (s)',ylabel='Fixed-body joint My (N m)',title='Measured fixed-body/world reaction torque')
for a in ax.flat:a.grid(alpha=.2)
for a in ax[1]:a.legend()
fig.suptitle('Fixed-body Delta | sensitivity and native-constraint verification');fig.savefig(R/'comparison.png',dpi=150)
(R/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
