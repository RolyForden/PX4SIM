"""Judge executed tests using original ULog states, motor outputs and allocation."""
from pathlib import Path
import json,sys,hashlib
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent;name=sys.argv[1];D=R/'runs'/name
outcome=json.loads((D/'outcome.json').read_text());events=outcome['events'];by={e['event']:e for e in events}
if not outcome['passed_execution'] or not outcome['ulog']:raise SystemExit('Flight execution incomplete; evidence retained')
u=ULog(str(D/outcome['ulog']['file']));a=u.get_dataset('vehicle_attitude').data;p=u.get_dataset('vehicle_local_position').data;s=u.get_dataset('vehicle_status').data;l=u.get_dataset('vehicle_land_detected').data;mot=u.get_dataset('actuator_motors').data;ca=u.get_dataset('control_allocator_status').data
start=by['hover_start']['boot_ms']*1000;end=by['hover_complete']['boot_ms']*1000;arm=by['armed']['boot_ms']*1000;disarm=by['auto_disarmed']['boot_ms']*1000
def mask(d,lo=start,hi=end):return (d['timestamp']>=lo)&(d['timestamp']<=hi)
q=np.array([a[f'q[{i}]'] for i in range(4)]).T;roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)));pitch=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)));origin=by['origin']['origin'];height=origin['z']-p['z'];horizontal=np.hypot(p['x']-origin['x'],p['y']-origin['y']);pm=mask(p);am=mask(a);sm=mask(s,arm,disarm);cm=mask(ca);mm=mask(mot)
controls=np.array([mot[f'control[{i}]'][mm] for i in range(4)]).T;sat=np.array([ca[f'actuator_saturation[{i}]'][cm] for i in range(4)]).T;upper=np.any(sat>=2,axis=1);maxrun=0;run=None
for t,flag in zip(ca['timestamp'][cm],upper):
 if flag:
  if run is None:run=t
  maxrun=max(maxrun,(t-run)/1e6)
 else:run=None
# The runtime event enumeration is authoritative for disarm reason; msg constants differ here.
def find_enum(v):
 if isinstance(v,dict):
  if 'arm_disarm_reason_t' in v:return v['arm_disarm_reason_t']['entries']
  for x in v.values():
   r=find_enum(x)
   if r:return r
 return None
entries=find_enum(json.loads((R.parent/'PX4-Autopilot/src/lib/events/enums.json').read_text()));land_reason=next(int(k) for k,v in entries.items() if v['name']=='auto_disarm_land');actual_reason=int(s['latest_disarming_reason'][-1]);console=(D/'px4.log').read_text(errors='replace');auto_landed=actual_reason==land_reason and int(s['arming_state'][-1])==1 and bool(l['landed'][-1]) and 'Disarmed by landing' in console
summary={'name':name,'payload_kg':json.loads((D/'preflight.json').read_text())['payload_kg'],'mass_kg':json.loads((D/'preflight.json').read_text())['mass_properties']['mass_kg'],'passed':False,'hover_sim_s':(end-start)/1e6,'hover_mean_height_m':float(np.mean(height[pm])),'hover_max_height_error_m':float(np.max(np.abs(height[pm]-2))),'hover_max_horizontal_offset_m':float(np.max(horizontal[pm])),'hover_max_abs_roll_pitch_deg':[float(np.max(np.abs(roll[am]))),float(np.max(np.abs(pitch[am])))],'hover_max_abs_vertical_speed_m_s':float(np.max(np.abs(p['vz'][pm]))),'flight_max_height_m':float(np.max(height[mask(p,arm,disarm)])),'hover_motor_control_min':np.nanmin(controls,axis=0).tolist(),'hover_motor_control_max':np.nanmax(controls,axis=0).tolist(),'hover_upper_saturation_fraction':float(np.mean(upper)),'hover_longest_upper_saturation_s':float(maxrun),'hover_torque_setpoint_achieved_fraction':float(np.mean(ca['torque_setpoint_achieved'][cm])),'hover_thrust_setpoint_achieved_fraction':float(np.mean(ca['thrust_setpoint_achieved'][cm])),'hover_nav_states':np.unique(s['nav_state'][mask(s)]).tolist(),'flight_failsafe_count':int(np.sum(s['failsafe'][sm])),'auto_landed_and_disarmed':auto_landed,'latest_disarming_reason':actual_reason,'event_enum_auto_disarm_land':land_reason,'land_to_contact_sim_s':(by['ground_detected']['boot_ms']-by['land_trigger']['boot_ms'])/1000,'contact_to_auto_disarm_sim_s':(by['auto_disarmed']['boot_ms']-by['ground_detected']['boot_ms'])/1000,'ulog_dropout_count':len(u.dropouts),'ulog_sha256':outcome['ulog']['sha256'],'ulog_bytes':outcome['ulog']['bytes'],'px4_git_revision':u.msg_info_dict.get('ver_sw'),'control_parameters_unchanged':by['parameter_snapshot']['control_parameters_unchanged']}
summary['passed']=bool(summary['hover_sim_s']>=20 and summary['hover_max_height_error_m']<.5 and summary['hover_max_horizontal_offset_m']<.5 and max(summary['hover_max_abs_roll_pitch_deg'])<5 and summary['hover_longest_upper_saturation_s']<.5 and summary['hover_upper_saturation_fraction']<.01 and summary['flight_failsafe_count']==0 and summary['auto_landed_and_disarmed'] and not u.dropouts)
(D/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if not summary['passed']:raise SystemExit('ULog acceptance failed')
