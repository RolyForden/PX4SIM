"""Measure native Gazebo positions/forces for gravity and payload validation."""
from pathlib import Path
import json,time,argparse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kinematics import HOME
R=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--name',required=True);ap.add_argument('--static-only',action='store_true');args=ap.parse_args()
if (R/'results'/args.name/'summary.json').exists():raise SystemExit('结果名称已存在，请用 --name 指定新名称')
settings=json.loads((R/'settings.json').read_text());rclpy.init();node=Node('verify_delta_dynamics');rows=[]
node.create_subscription(String,'/a_uav/arm_native/status',lambda m:rows.append(json.loads(m.data)),100);pub=node.create_publisher(String,'/a_uav/arm_native/command',10)
def wait(pred,timeout=120):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1)
  if rows and (not np.isfinite(rows[-1]['position_m']).all() or rows[-1]['closure_m']>.03):raise RuntimeError('实际机构约束失稳')
  if pred():return
 raise TimeoutError('等待仿真实际状态超时')
try:
 wait(lambda:rows and rows[-1]['ready'] and pub.get_subscription_count()>0,20);t0=rows[-1]['sim_s'];wait(lambda:rows[-1]['sim_s']>t0+8)
 static=[s for s in rows if s['sim_s']>=rows[-1]['sim_s']-2];motion=[]
 if not args.static_only:
  before=rows[-1]['command_id'];m=String();m.data='demo';pub.publish(m);wait(lambda:rows[-1]['command_id']>before,10);cid=rows[-1]['command_id'];print('Executing actual dynamics sequence',args.name,flush=True);wait(lambda:rows[-1]['completed_id']==cid or rows[-1]['error'])
  if rows[-1]['error']:raise RuntimeError(rows[-1]['error'])
  motion=[s for s in rows if s['command_id']==cid and s['busy']]
 tf=rows[-1]['sim_s'];wait(lambda:rows[-1]['sim_s']>tf+4);end=[s for s in rows if s['sim_s']>=rows[-1]['sim_s']-2]
 F=np.array([s['platform_constraint_force_n'] for s in static]);tau=np.array([s['joint_torque_nm'] for s in static]);weight=(.05+settings['payload']+settings['proxy_total_mass_kg'])*settings['gravity'];balance=abs(F[:,2].mean()-weight);P=np.array([s['position_m'] for s in rows]);bp=np.array([s['base_world_m'] for s in rows]);closure=max(s['closure_m'] for s in rows);enderr=float(np.linalg.norm(np.mean([s['position_m'] for s in end],axis=0)-HOME));tracking=max([s['tracking_error_m'] for s in motion],default=0);tilt=max(np.linalg.norm(s['platform_rpy_rad'][:2]) for s in rows);speed=float(np.max(np.linalg.norm([s['platform_velocity_m_s'] for s in end],axis=1)))
 summary={'name':args.name,'settings':settings,'passed':bool(balance<max(.01,.03*weight) and closure<.002 and enderr<.001 and tracking<.003 and np.max(np.linalg.norm(bp-bp[0],axis=1))<1e-6 and speed<.005),'sim_duration_s':rows[-1]['sim_s']-t0,'samples':len(rows),'static_support_force_n':F.mean(axis=0).tolist(),'expected_platform_payload_proxy_weight_n':weight,'static_vertical_force_balance_error_n':float(balance),'static_joint_torque_nm':tau.mean(axis=0).tolist(),'static_joint_torque_std_nm':tau.std(axis=0).tolist(),'max_closure_m':closure,'max_tracking_error_m':tracking,'final_home_error_m':enderr,'max_base_displacement_m':float(np.max(np.linalg.norm(bp-bp[0],axis=1))),'max_platform_tilt_rad':float(tilt),'final_max_speed_m_s':speed,'max_applied_joint_torque_nm':np.max(np.abs([s['joint_torque_nm'] for s in rows]),axis=0).tolist(),'trajectory_tested':not args.static_only,'passive_resets_used':False}
 dest=R/'results'/args.name;dest.mkdir(exist_ok=True);(dest/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(dest/'feedback.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in rows));(dest/'settings.json').write_text(json.dumps(settings,indent=2));(dest/'world.sdf').write_bytes((R/'world.sdf').read_bytes());print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
 if not summary['passed']:raise RuntimeError('重力/负载实测未通过判据')
finally:node.destroy_node();rclpy.shutdown()
