"""Judge native ULog flight phases and independently measured arm/aircraft states."""
from pathlib import Path
import json,sys,hashlib
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent;D=R/'runs'/sys.argv[1]
o=json.loads((D/'outcome.json').read_text());events={e['event']:e for e in o['events']}
if not o['passed_execution'] or not o['ulog']:raise SystemExit('Execution incomplete; inspect preserved evidence')
path=D/o['ulog']['file'];assert hashlib.sha256(path.read_bytes()).hexdigest()==o['ulog']['sha256']
u=ULog(str(path));p=u.get_dataset('vehicle_local_position').data;a=u.get_dataset('vehicle_attitude').data;s=u.get_dataset('vehicle_status').data;l=u.get_dataset('vehicle_land_detected').data;ca=u.get_dataset('control_allocator_status').data;m=u.get_dataset('actuator_motors').data
origin=events['origin']['origin'];height=origin['z']-p['z'];dist=np.hypot(p['x']-origin['x'],p['y']-origin['y'])
q=np.array([a[f'q[{i}]'] for i in range(4)]);roll=np.degrees(np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2)));pitch=np.degrees(np.arcsin(np.clip(2*(q[0]*q[2]-q[3]*q[1]),-1,1)))
arm=[json.loads(line) for line in (D/'arm-feedback.jsonl').read_text().splitlines()]
ground_sample=min(arm,key=lambda v:abs(v['sim_s']-events['ground_constraints_settled']['arm_sim_s']));ground_pose=np.array(ground_sample['base_world_m'])
def select(data,start,end):return (data['timestamp']>=start*1000)&(data['timestamp']<=end*1000)
phases={}
labels=['baseline','H25_transition','H25_arm','H25_return_X','V25_transition','V25_arm','V25_return_X','coupled_H','coupled_V','recovery']
for label in labels:
 start=label+'_start';end=label+'_end'
 lo=events[start]['boot_ms'];hi=events[end]['boot_ms'];pm=select(p,lo,hi);am=select(a,lo,hi);cm=select(ca,lo,hi);mm=select(m,lo,hi)
 ar=[v for v in arm if events[start]['arm_sim_s']<=v['sim_s']<=events[end]['arm_sim_s']]
 assert ar and pm.any() and am.any() and cm.any() and mm.any()
 upper=np.any(np.array([ca[f'actuator_saturation[{i}]'][cm] for i in range(4)])>=2,axis=0)
 w=np.array([v['mount_wrench_n_nm'] for v in ar]);com=np.array([v['com_flu_m'] for v in ar]);bp=np.array([v['base_world_m'] for v in ar]);br=np.degrees([v['base_world_rpy_rad'] for v in ar]);plat=np.array([v['position_m'] for v in ar]);duration=(hi-lo)/1000
 phases[label]={'sim_s':duration,'height_mean_m':float(np.mean(height[pm])),'height_max_error_m':float(np.max(np.abs(height[pm]-5))),'height_rms_error_m':float(np.sqrt(np.mean((height[pm]-5)**2))),'horizontal_max_offset_m':float(np.max(dist[pm])),'horizontal_rms_offset_m':float(np.sqrt(np.mean(dist[pm]**2))),'roll_pitch_peak_deg':[float(np.max(np.abs(roll[am]))),float(np.max(np.abs(pitch[am])))],'roll_pitch_rms_deg':[float(np.sqrt(np.mean(roll[am]**2))),float(np.sqrt(np.mean(pitch[am]**2)))],'motor_mean_control':float(np.mean([m[f'control[{i}]'][mm] for i in range(4)])),'upper_saturation_fraction':float(np.mean(upper)),'torque_achieved_fraction':float(np.mean(ca['torque_setpoint_achieved'][cm])),'thrust_achieved_fraction':float(np.mean(ca['thrust_setpoint_achieved'][cm])),'arm_max_tracking_error_m':max(v['tracking_error_m'] for v in ar),'arm_max_closure_m':max(v['closure_m'] for v in ar),'arm_position_range_m':np.ptp(plat,axis=0).tolist(),'arm_joint_range_rad':np.ptp([v['q_rad'] for v in ar],axis=0).tolist(),'arm_max_abs_joint_torque_nm':np.max(np.abs([v['joint_torque_nm'] for v in ar]),axis=0).tolist(),'com_range_flu_m':np.ptp(com,axis=0).tolist(),'mount_wrench_mean_n_nm':w.mean(axis=0).tolist(),'mount_wrench_peak_to_peak_n_nm':np.ptp(w,axis=0).tolist(),'mount_wrench_min_n_nm':w.min(axis=0).tolist(),'mount_wrench_max_n_nm':w.max(axis=0).tolist(),'truth_height_mean_m':float(np.mean(bp[:,2]-ground_pose[2])),'truth_height_max_error_m':float(np.max(np.abs(bp[:,2]-ground_pose[2]-5))),'truth_horizontal_max_offset_m':float(np.max(np.linalg.norm(bp[:,:2]-ground_pose[:2],axis=1))),'truth_base_height_range_m':float(np.ptp(bp[:,2])),'truth_base_horizontal_range_m':np.ptp(bp[:,:2],axis=0).tolist(),'truth_base_roll_pitch_peak_deg':np.max(np.abs(br[:,:2]),axis=0).tolist(),'nav_states':np.unique(s['nav_state'][select(s,lo,hi)]).tolist()}
armedmask=select(s,events['armed']['boot_ms'],events['auto_disarmed']['boot_ms'])
auto=bool(l['landed'][-1]) and int(s['arming_state'][-1])==1 and 'Disarmed by landing' in (D/'px4.log').read_text()
pre=json.loads((D/'preflight.json').read_text());params0=json.loads((D/'parameters/before.json').read_text());params1=json.loads((D/'parameters/after.json').read_text());changes={k:[params0.get(k),params1.get(k)] for k in params0.keys()|params1.keys() if params0.get(k)!=params1.get(k)}
allowed_changes={'COM_FLIGHT_UUID','LND_FLIGHT_T_LO'}
autocal={}
if 'CAL_BARO0_OFF' in changes and params0.get('SENS_BAR_AUTOCAL')==1 and u.initial_parameters.get('CAL_BARO0_OFF')==params1['CAL_BARO0_OFF'] and not any(v[1]=='CAL_BARO0_OFF' for v in u.changed_parameters):
 allowed_changes.add('CAL_BARO0_OFF');autocal={'parameter':'CAL_BARO0_OFF','before_snapshot':params0['CAL_BARO0_OFF'],'ULog_initial':u.initial_parameters['CAL_BARO0_OFF'],'after_snapshot':params1['CAL_BARO0_OFF'],'flight_updates':0,'source':'VehicleAirData::BaroGNSSAltitudeOffset -> Barometer::ParametersSave; SENS_BAR_AUTOCAL enabled'}
motion=phases['coupled_V'];delta=np.array(motion['mount_wrench_mean_n_nm'])-phases['baseline']['mount_wrench_mean_n_nm']
result={'name':D.name,'payload_kg':pre['payload_kg'],'mass_kg':pre['mass_properties']['mass_kg'],'passed':False,'phases':phases,'motion_minus_before_mean_mount_wrench_n_nm':delta.tolist(),'all_arm_segments':sorted({v['segment'] for v in arm if v.get('busy')}),'arm_completed_id':events['recovery_end']['arm_seq'],'auto_landed_and_disarmed':auto,'land_to_contact_sim_s':(events['ground_detected']['boot_ms']-events['land_trigger']['boot_ms'])/1000,'contact_to_disarm_sim_s':(events['auto_disarmed']['boot_ms']-events['ground_detected']['boot_ms'])/1000,'flight_failsafe_count':int(np.sum(s['failsafe'][armedmask])),'ulog_dropout_count':len(u.dropouts),'ulog':o['ulog'],'px4_revision':u.msg_info_dict.get('ver_sw'),'parameter_changes':changes,'control_parameters_unchanged':events['parameter_snapshot']['control_parameters_unchanged'],'dynamic_CA_updates':True,'automatic_baro_calibration':autocal}
result['arm_completed_id']=max(v.get('completed_id',0) for v in arm)
result['max_mass_deviation_kg']=max(abs(v['total_mass_kg']-result['mass_kg']) for v in arm)
result['all_runtime_inertias_positive']=all(np.linalg.eigvalsh(np.array(v['inertia_com_flu_kg_m2']).reshape(3,3)).min()>0 for v in arm)
morph=[json.loads(line) for line in (D/'morph-feedback.jsonl').read_text().splitlines()]
for label in labels:
 lo=events[label+'_start']['arm_sim_s'];hi=events[label+'_end']['arm_sim_s'];mr=[v for v in morph if lo<=v['geometry_sim_s']<=hi];assert mr
 B=np.array([v['B'] for v in mr]);ranks=[];margins=[];residuals=[]
 for bb in B:
  aa=bb[[0,1,2,5]];ranks.append(int(np.linalg.matrix_rank(aa)));f=np.linalg.solve(aa,[0,0,0,-result['mass_kg']*9.81]);margins.append(float(min(8.54858-f)));residuals.append(float(np.linalg.norm((bb@f)[3:5])))
 q=np.array([v['q_deg'] for v in mr]);overlap=[v for v in mr if v['transition'] and v['arm_busy']]
 phases[label].update(morph_min_rank=min(ranks),minimum_hover_thrust_margin_n=min(margins),max_horizontal_hover_residual_n=max(residuals),max_morph_tracking_error_deg=max(v['tracking_error_deg'] for v in mr),max_allocator_ack_age_ms=max(v['allocator_ack_age_ms'] for v in mr),min_actual_q_deg=q.min(axis=0).tolist(),max_actual_q_deg=q.max(axis=0).tolist(),overlap_sim_s=float(sum(max(0,mr[i+1]['geometry_sim_s']-v['geometry_sim_s']) for i,v in enumerate(mr[:-1]) if v['transition'] and v['arm_busy'])),overlap_samples=len(overlap),max_com_excursion_m=np.ptp([v['com_flu_m'] for v in mr],axis=0).tolist())
matrices=[json.loads(p.read_text()) for p in sorted(D.glob('matrix-*.json'))]
result['native_matrix_readbacks']=[{'label':m['label'],'passed':m['passed'],'max_abs_error':m['max_abs_error']} for m in matrices]
result['matrix_update_count']=len(morph)
result['fixed_H25_verified']=all(abs(abs(x)-25)<1 for x in phases['H25_arm']['min_actual_q_deg'][::2]+phases['H25_arm']['max_actual_q_deg'][::2])
result['fixed_V25_verified']=all(abs(x-25)<1 for x in phases['V25_arm']['min_actual_q_deg'][1::2]+phases['V25_arm']['max_actual_q_deg'][1::2])
result['arm_completed_id']=max(v.get('completed_id',0) for v in arm)
result['passed']=bool(result['fixed_H25_verified'] and result['fixed_V25_verified'] and auto and not u.dropouts and result['flight_failsafe_count']==0 and set(changes)<=allowed_changes and all(v['truth_height_max_error_m']<.5 and v['truth_horizontal_max_offset_m']<.5 and v['height_max_error_m']<.5 and v['horizontal_max_offset_m']<.5 and max(v['roll_pitch_peak_deg'])<5 and v['upper_saturation_fraction']==0 and v['arm_max_tracking_error_m']<.003 and v['arm_max_closure_m']<1e-5 and v['morph_min_rank']==4 and v['minimum_hover_thrust_margin_n']>0 and v['max_morph_tracking_error_deg']<5 for v in phases.values()) and phases['baseline']['sim_s']>=10 and phases['recovery']['sim_s']>=10 and result['arm_completed_id']==4 and all(min(phases[k]['arm_position_range_m'])>.02 for k in ['H25_arm','V25_arm']) and all(phases[k]['overlap_sim_s']>15 and phases[k]['overlap_samples']>50 for k in ['coupled_H','coupled_V']) and len(matrices)==6 and all(m['passed'] for m in matrices))
result['flight_dynamics_passed']=result['passed']
gt_t=np.array([v['sim_s'] for v in arm]);gt_z=np.array([v['base_world_m'][2] for v in arm]);gt_xy=np.array([v['base_world_m'][:2] for v in arm])
ground_t=events['ground_constraints_settled']['arm_sim_s'];ground_z=float(np.interp(ground_t,gt_t,gt_z));land_t=events['land_trigger']['arm_sim_s'];disarm_t=events['auto_disarmed']['arm_sim_s']
lm=select(p,events['land_trigger']['boot_ms'],events['auto_disarmed']['boot_ms'])
# Both clocks are simulator time; event snapshots can differ by telemetry latency.
truth=np.interp(p['timestamp'][lm]/1e6,gt_t,gt_z)-ground_z
result['landing_estimator_max_height_error_vs_truth_m']=float(np.max(np.abs(height[lm]-truth)))
endrows=[v for v in arm if disarm_t-.5<=v['sim_s']<=disarm_t]
result['final_truth_height_above_ground_m']=float(np.mean([v['base_world_m'][2] for v in endrows])-ground_z)
result['landing_estimator_check_passed']=result['landing_estimator_max_height_error_vs_truth_m']<.3
result['passed']=bool(result['passed'] and result['max_mass_deviation_kg']<1e-8 and result['all_runtime_inertias_positive'] and abs(result['final_truth_height_above_ground_m'])<.01 and result['landing_estimator_check_passed'])
(D/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if not result['passed']:raise SystemExit('Acceptance failed; loaded flight not authorized by this result')
