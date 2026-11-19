"""Execute the fixed-base demo and assess Gazebo feedback, not command acceptance."""
from pathlib import Path
import json,time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kinematics import HOME
R=Path(__file__).resolve().parent
rclpy.init();n=Node('verify_a_delta_fixed');states=[]
n.create_subscription(String,'/a_uav/arm/status',lambda m:states.append(json.loads(m.data)),100)
pub=n.create_publisher(String,'/a_uav/arm/command',10)
def wait(pred,timeout):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if pred():return
 raise TimeoutError('等待实际状态超时')
try:
 wait(lambda:states and states[-1]['ready'] and pub.get_subscription_count()>0,20)
 before=states[-1]['command_id'];m=String();m.data='demo';pub.publish(m)
 wait(lambda:states[-1]['command_id']>before,5);cid=states[-1]['command_id'];print('Demo accepted:',cid,flush=True)
 wait(lambda:states[-1]['completed_id']==cid or states[-1]['error'],120)
 if states[-1]['error']:raise RuntimeError(states[-1]['error'])
 stop=states[-1]['sim_s']+1;wait(lambda:states[-1]['sim_s']>=stop,10)
 data=[s for s in states if s['command_id']==cid];errors=[s['tracking_error_m'] for s in data];poses=np.array([s['position_m'] for s in data]);base=np.array([s['base_world_m'] for s in data]);q=np.array([s['q_rad'] for s in data]);end_error=float(np.linalg.norm(poses[-1]-HOME));closure=max(s['closure_m'] for s in data);base_error=float(np.max(np.linalg.norm(base-base[0],axis=1)));fk=max(s['platform_fk_error_m'] for s in data)
 summary={'passed':bool(max(errors)<.002 and closure<.002 and end_error<.0005 and base_error<1e-6),'sim_duration_s':data[-1]['sim_s']-data[0]['sim_s'],'feedback_samples':len(data),'max_tracking_error_m':max(errors),'rms_tracking_error_m':float(np.sqrt(np.mean(np.square(errors)))),'max_rod_endpoint_closure_m':closure,'max_platform_fk_error_m':fk,'final_home_error_m':end_error,'max_base_displacement_m':base_error,'position_min_m':poses.min(axis=0).tolist(),'position_max_m':poses.max(axis=0).tolist(),'joint_min_rad':q.min(axis=0).tolist(),'joint_max_rad':q.max(axis=0).tolist(),'segments':sorted(set(s['segment'] for s in data)),'sequence':'home -> down 25 mm -> home -> x +15 mm -> home -> circle radius 15 mm (z -15 mm) -> home','method':'three force-controlled Gazebo joints; passive links reset from actual joint FK; no passive force closure','flight':False}
 (R/'results/verification.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(R/'results/verification.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in data));print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
 if not summary['passed']:raise RuntimeError('实际轨迹未达到判据')
finally:n.destroy_node();rclpy.shutdown()
