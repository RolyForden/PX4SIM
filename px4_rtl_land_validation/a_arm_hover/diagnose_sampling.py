"""Compare measured touchdown delta-v with logged IMU integration."""
from pathlib import Path
import json
import xml.etree.ElementTree as E
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent;out=[]
for name in ['empty_01','empty_imu5000']:
 d=R/'runs'/name;o=json.loads((d/'outcome.json').read_text());ev={e['event']:e for e in o['events']};u=ULog(str(d/o['ulog']['file']));imu=u.get_dataset('sensor_combined').data;att=u.get_dataset('vehicle_attitude').data;p=u.get_dataset('vehicle_local_position').data
 arm=[json.loads(s) for s in (d/'arm-feedback.jsonl').read_text().splitlines()];gt=np.array([[s['sim_s'],s['base_world_m'][2]] for s in arm]);ts=imu['timestamp']/1e6;t0=ev['land_trigger']['boot_ms']/1000;lo=t0+3;hi=min(t0+8,ts[-1]-.01);mask=(ts>=lo)&(ts<=hi);t=ts[mask];q=np.array([np.interp(t,att['timestamp']/1e6,att[f'q[{i}]']) for i in range(4)]);q/=np.linalg.norm(q,axis=0)
 acc=np.array([imu[f'accelerometer_m_s2[{i}]'][mask] for i in range(3)])
 world_down_acc=(2*(q[1]*q[3]-q[0]*q[2])*acc[0]+2*(q[2]*q[3]+q[0]*q[1])*acc[1]+(1-2*(q[1]**2+q[2]**2))*acc[2])+9.81
 def down_v(t):return -(np.interp(t+.025,gt[:,0],gt[:,1])-np.interp(t-.025,gt[:,0],gt[:,1]))/.05
 truth=down_v(hi)-down_v(lo);integral=float(np.trapezoid(world_down_acc,t));s=json.loads((d/'summary.json').read_text())
 out.append({'trial':name,'imu_rate_hz':int(E.parse(d/'world.sdf').find("world/model[@name='a_arm']/link[@name='base_link']/sensor[@name='imu_sensor']/update_rate").text),'window_sim_s':[lo,hi],'truth_down_delta_velocity_m_s':float(truth),'logged_imu_world_down_delta_velocity_m_s':integral,'absolute_delta_velocity_mismatch_m_s':abs(integral-truth),'landing_height_estimate_max_error_vs_truth_m':s['landing_estimator_max_height_error_vs_truth_m'],'all_control_parameters_unchanged':s['control_parameters_unchanged'],'interpretation':'sampling/impulse consistency comparison; not a general sensor calibration'})
(R/'results/sampling-diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
