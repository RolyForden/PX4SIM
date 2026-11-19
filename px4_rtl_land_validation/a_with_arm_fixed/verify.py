"""Actual trajectory, mount/fixture force balance, and independent mass-tensor checks."""
from pathlib import Path
import argparse,json,time,shutil
import numpy as np
import xml.etree.ElementTree as E
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kinematics import HOME,CONFIG,rotation as axis_rotation
from mass_properties import records,aggregate,tensor,pose,rotation,valid_tensor
R=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--name',required=True);args=ap.parse_args();out=R/'results'/args.name
if out.exists():raise SystemExit('结果目录已存在，请改用新名称')
settings=json.loads((R/'settings.json').read_text());audit=json.loads((R/'mass-audit.json').read_text());root=E.parse(R/'world.sdf').getroot();model=root.find("world/model[@name='a_with_arm_fixed']");platform=root.find("world/model[@name='integrated_platform']");arm_names={f'upper_{i}' for i in range(3)}|{f'rod_{i}_{k}' for i in range(3) for k in range(2)}|{f'tip_{i}_{k}' for i in range(3) for k in range(2)};rigid=[r for r in records(model) if r['name'] not in arm_names]
def inert(m,name):
 a=m.find(f"link[@name='{name}']/inertial");return float(a.findtext('mass')),pose(a.find('pose'))[:3],tensor(a.find('inertia'))
def predicted(s):
 rows=list(rigid);p=np.array(s['position_m']);Qp=rotation(s['platform_rpy_rad'])
 for i,leg in enumerate(CONFIG['legs']):
  S=np.array(leg['shoulder_m']);Q=axis_rotation(np.array(leg['axis']),s['q_rad'][i]);m,c,I=inert(model,f'upper_{i}');rows.append({'name':f'upper_{i}','mass':m,'com':S+Q@c,'I':Q@I@Q.T})
  for k in range(2):
   e=S+Q@(np.array(leg['elbows_home_m'][k])-S);b=p+Qp@np.array(leg['platform_offsets_m'][k]);z=(b-e)/np.linalg.norm(b-e);a=np.array(leg['axis']);x=a-z*np.dot(a,z);x/=np.linalg.norm(x);Qr=np.column_stack([x,np.cross(z,x),z]);m,c,I=inert(model,f'rod_{i}_{k}');rows.append({'name':'rod','mass':m,'com':e+Qr@c,'I':Qr@I@Qr.T});m,c,I=inert(model,f'tip_{i}_{k}');rows.append({'name':'tip','mass':m,'com':b+Qp@c,'I':Qp@I@Qp.T})
 for name in ['platform','payload']:
  if platform.find(f"link[@name='{name}']") is None:continue
  m,c,I=inert(platform,name);offset=pose(platform.find(f"link[@name='{name}']/pose"))[:3];rows.append({'name':name,'mass':m,'com':p+Qp@(offset+c),'I':Qp@I@Qp.T})
 return aggregate(rows)
rclpy.init();node=Node('verify_a_integrated_arm');rows=[];node.create_subscription(String,'/a_uav/integrated_arm/status',lambda m:rows.append(json.loads(m.data)),100);pub=node.create_publisher(String,'/a_uav/integrated_arm/command',10)
def wait(fn,timeout=120):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1)
  if rows and (not np.isfinite(rows[-1]['position_m']).all() or rows[-1]['closure_m']>.01):raise RuntimeError('机构实际约束失稳')
  if fn():return
 raise TimeoutError('等待实际反馈超时')
try:
 wait(lambda:rows and rows[-1]['ready'] and rows[-1]['mount_wrench_n_nm'] is not None and pub.get_subscription_count()>0,20);t0=rows[-1]['sim_s'];wait(lambda:rows[-1]['sim_s']>t0+8);static=[s for s in rows if s['sim_s']>=rows[-1]['sim_s']-2]
 cid=rows[-1]['command_id'];m=String();m.data='demo';pub.publish(m);wait(lambda:rows[-1]['command_id']>cid,10);cid=rows[-1]['command_id'];print('Integrated fixed-body trajectory executing:',args.name,flush=True);wait(lambda:rows[-1]['completed_id']==cid or rows[-1]['error'])
 if rows[-1]['error']:raise RuntimeError(rows[-1]['error'])
 tf=rows[-1]['sim_s'];wait(lambda:rows[-1]['sim_s']>tf+4);ending=[s for s in rows if s['sim_s']>=rows[-1]['sim_s']-2];motion=[s for s in rows if s['command_id']==cid and s['busy']]
 mount=np.array([s['mount_wrench_n_nm'] for s in static]);expected=np.array([s['expected_mount_gravity_n_nm'] for s in static]);mount_error=np.max(np.abs(mount.mean(axis=0)-expected.mean(axis=0)));total=settings['mass_ledger']['expected_total_kg'];airmass=settings['mass_ledger']['aircraft_baseline_kg'];aircom=np.array(audit['corrected_aircraft_aggregate']['com_m']);airweight=np.array([0,0,airmass*9.81]);airtorque=np.cross(aircom,airweight);offset=np.array(settings['mount_origin_flu_m']);force_error=0;torque_error=0
 for s in rows:
  f=np.array(s['fixture_wrench_n_nm']);m=np.array(s['mount_wrench_n_nm']);force_error=max(force_error,np.max(np.abs(f[:3]-m[:3]-airweight)));torque_error=max(torque_error,np.max(np.abs(f[3:]-m[3:]-np.cross(offset,m[:3])-airtorque)))
 comerr=0;Ierr=0
 for s in rows[::20]:
  pred=predicted(s);comerr=max(comerr,np.linalg.norm(np.array(pred['com_m'])-s['com_flu_m']));Ierr=max(Ierr,np.max(np.abs(np.array(pred['inertia_com_kg_m2'])-np.array(s['inertia_com_flu_kg_m2']).reshape(3,3))))
 masserr=max(abs(s['total_mass_kg']-total) for s in rows);tracking=max(s['tracking_error_m'] for s in motion);closure=max(s['closure_m'] for s in rows);base=np.array([s['base_world_m'] for s in rows]);baseerr=float(np.max(np.linalg.norm(base-base[0],axis=1)));homeerr=float(np.linalg.norm(np.mean([s['position_m'] for s in ending],axis=0)-HOME));positive=all(valid_tensor(np.array(s['inertia_com_flu_kg_m2']).reshape(3,3)) for s in rows)
 summary={'name':args.name,'passed':bool(masserr<1e-9 and comerr<2e-6 and Ierr<2e-7 and mount_error<.005 and force_error<.005 and torque_error<.002 and tracking<.003 and closure<1e-5 and baseerr<1e-6 and homeerr<.001 and positive),'payload_kg':settings['payload'],'total_mass_kg':total,'mechanism_mass_kg':rows[-1]['mechanism_mass_kg'],'samples':len(rows),'sim_duration_s':rows[-1]['sim_s']-t0,'max_tracking_error_m':tracking,'max_closure_m':closure,'final_home_error_m':homeerr,'base_displacement_m':baseerr,'static_mount_wrench_n_nm':mount.mean(axis=0).tolist(),'expected_mount_gravity_n_nm':expected.mean(axis=0).tolist(),'max_static_mount_balance_error':float(mount_error),'max_fixture_minus_mount_force_residual_n':float(force_error),'max_fixture_minus_mount_torque_residual_nm':float(torque_error),'max_runtime_mass_error_kg':float(masserr),'max_runtime_com_crosscheck_error_m':float(comerr),'max_runtime_tensor_crosscheck_error_kg_m2':float(Ierr),'all_runtime_inertias_valid':positive,'com_min_flu_m':np.min([s['com_flu_m'] for s in rows],axis=0).tolist(),'com_max_flu_m':np.max([s['com_flu_m'] for s in rows],axis=0).tolist(),'flight':False}
 out.mkdir();(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(out/'feedback.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in rows))
 for name in ['world.sdf','settings.json','mass-audit.json']:shutil.copy2(R/name,out/name)
 for name in ['mount_wrench.jsonl','fixture_wrench.jsonl']:shutil.copy2(R/'results'/name,out/name)
 print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
 if not summary['passed']:raise RuntimeError('集成台架未达到验收判据')
finally:node.destroy_node();rclpy.shutdown()
