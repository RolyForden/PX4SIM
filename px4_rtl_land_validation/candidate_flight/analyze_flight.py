"""Read actual candidate ULog and events; produce evidence, never synthetic telemetry."""
import pathlib,json,hashlib,argparse
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parent

def analyze(name):
 path=ROOT/'logs'/f'{name}.ulg';events=json.loads((ROOT/'logs'/f'{name}_events.json').read_text());config=json.loads((ROOT/'logs'/f'{name}_model_config.json').read_text())
 assert events[-1]['event']=='test_complete','Cannot mark incomplete/failed run as pass'
 assert not config.get('diagnostic_variant'),'Stock environment probe is not candidate flight evidence'
 u=ULog(str(path));pos=u.get_dataset('vehicle_local_position').data;att=u.get_dataset('vehicle_attitude').data;land=u.get_dataset('vehicle_land_detected').data;status=u.get_dataset('vehicle_status').data
 origin=next(e['origin'] for e in events if e['event']=='origin');t=pos['timestamp'].astype(float)/1e6;height=origin['z']-pos['z'];dist=np.hypot(pos['x']-origin['x'],pos['y']-origin['y'])
 q=np.array([att[f'q[{i}]'] for i in range(4)]);w,x,y,z=q;roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1));ta=att['timestamp'].astype(float)/1e6
 r=np.interp(t,ta,roll);p=np.interp(t,ta,pitch)
 arm=np.interp(t,status['timestamp'].astype(float)/1e6,status['arming_state'])>1.5
 airborne=land['landed']==0;assert airborne.any(),'No actual airborne detection'
 last_air_start=land['timestamp'][np.flatnonzero(airborne)[0]];contact_indices=np.flatnonzero((land['timestamp']>last_air_start)&(land['landed']==1));assert len(contact_indices),'No contact after airborne'
 contact=land['timestamp'][contact_indices[0]]/1e6;disarm_indices=np.flatnonzero((status['timestamp']/1e6>contact)&(status['arming_state']==1));assert len(disarm_indices),'No disarm after contact';disarm=status['timestamp'][disarm_indices[0]]/1e6
 hover_event=next(e for e in events if e['event']=='hover_confirmed');land_event=next(e for e in events if e['event']=='land_trigger');assert any(e['event']=='auto_disarmed' for e in events)
 stable=(abs(height-5)<.5)&(abs(pos['vz'])<.3)&(dist<1)&(abs(r)<np.radians(10))&(abs(p)<np.radians(10))&arm
 # Continuous duration measured in PX4 simulation time, not host wall time.
 limit=land_event['boot_ms']/1000+.1;stable&=t<limit
 best=0;start=None;prev=None
 for ti,ok in zip(t,stable):
  if ok:
   if start is None or (prev is not None and ti-prev>.2):start=ti
   best=max(best,ti-start);prev=ti
  else:start=None;prev=None
 region=arm&(height>1)
 result={'run':name,'status':'candidate fixed-rigid flight passed under assumed parameters; NOT final-aircraft validation','source_ulog':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'airframe_id':u.initial_parameters['SYS_AUTOSTART'],'source_cad_sha256':config['source_sha256'],'mass_properties_source':config.get('mass_properties_source'),'mass_properties_source_sha256':config.get('mass_properties_source_sha256'),'mass_properties_status':config.get('mass_properties_status'),'temporary_frame':config.get('frame_status'),'max_relative_height_m':float(height[arm].max()),'max_horizontal_displacement_m':float(dist[arm].max()),'max_abs_roll_pitch_deg_above_1m':[float(np.degrees(abs(r[region])).max()),float(np.degrees(abs(p[region])).max())],'longest_stable_hover_sim_s':float(best),'land_command_to_contact_sim_s':float(contact-land_event['boot_ms']/1000),'contact_to_disarm_sim_s':float(disarm-contact),'ulog_dropout_count':len(u.dropouts),'checks':{'airborne_detected':True,'hover_event':True,'stable_hover_over_5_sim_s':bool(best>=5),'contact_after_airborne':True,'disarm_after_contact':True,'complete_event':True},'assumptions':config['limitations']}
 assert result['checks']['stable_hover_over_5_sim_s'],'Insufficient actual stable hover'
 out=ROOT/'analysis';out.mkdir(exist_ok=True);(out/f'{name}_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True);time=t-t[0]
 axes[0].plot(time,height,label='ULog actual relative height');sp=u.get_dataset('vehicle_local_position_setpoint').data;axes[0].plot(sp['timestamp'].astype(float)/1e6-t[0],origin['z']-sp['z'],label='Height setpoint',alpha=.7);axes[0].set_ylabel('Height (m)');axes[0].legend()
 axes[1].plot(time,pos['vz']);axes[1].set_ylabel('Down velocity (m/s)')
 axes[2].plot(time,np.degrees(r),label='roll');axes[2].plot(time,np.degrees(p),label='pitch');axes[2].set_ylabel('Angle (deg)');axes[2].set_xlabel('PX4 simulation time from log start (s)');axes[2].legend()
 for ax in axes:ax.grid(alpha=.25);ax.axvline(contact-t[0],color='green',linestyle='--',alpha=.6)
 fig.suptitle('Candidate CAD | fixed rigid body | assumed dynamics | actual ULog');fig.tight_layout();fig.savefig(out/f'{name}.png',dpi=140);print(json.dumps(result,ensure_ascii=False,indent=2));return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--name');args=a.parse_args();analyze(args.name or json.loads((ROOT/'last_run.json').read_text())['name'])
