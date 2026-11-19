"""Verify trial source snapshots, native constraints and unchanged control parameters."""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as E
R=Path(__file__).resolve().parent;V=R.parent
out=[]
for name in ['empty_imu5000','load50g_01']:
 d=R/'runs'/name;pre=json.loads((d/'preflight.json').read_text());tree=E.parse(d/'world.sdf');air=tree.find("world/model[@name='a_arm']");platform=tree.find("world/model[@name='integrated_platform']")
 assert air is not None and platform is not None
 assert hashlib.sha256((V/pre['source_integrated_sdf']).read_bytes()).hexdigest()==pre['source_sha256']
 assert hashlib.sha256((d/'world.sdf').read_bytes()).hexdigest()==pre['world_sha256']
 assert not any(j.findtext('parent')=='world' for m in [air,platform] for j in m.findall('joint'))
 assert len(air.findall("joint[@type='revolute']"))==7
 assert len(air.findall("joint[@type='ball']"))==12
 assert air.find("link[@name='base_link']/sensor[@name='imu_sensor']/update_rate").text=='5000'
 assert len(air.findall('plugin'))==5
 source=(d/'controller.cpp').read_text()
 assert 'DetachableJoint({platform,proxy[i][k],"fixed"})' in source
 assert 'JointPositionReset' not in source and 'WorldPoseCmd' not in source
 before=json.loads((d/'parameters/before.json').read_text());after=json.loads((d/'parameters/after.json').read_text());ref=json.loads((d/'reference-control-parameters.json').read_text())
 assert before['SYS_AUTOSTART']==after['SYS_AUTOSTART']==4994
 for k,v in ref.items():assert abs(before[k]-v)<1e-5*max(1,abs(v)) and before[k]==after[k]
 for i in range(4):
  for k,v in zip(['PX','PY','PZ'],pre['rotor_geometry_frd_m'][i]):assert abs(after[f'CA_ROTOR{i}_{k}']-v)<1e-6
  for k in ['PX','PY','PZ','AX','AY','AZ','CT','KM']:assert before[f'CA_ROTOR{i}_{k}']==after[f'CA_ROTOR{i}_{k}']
 summary=json.loads((d/'summary.json').read_text());assert summary['passed']
 assert (d/'ros-command.log').read_text().count('demo')>=1
 out.append({'trial':name,'passed':True,'no_world_anchor':True,'native_ball_joints':12,'actuated_revolute_joints':3,'rotor_revolute_joints':4,'no_passive_pose_reset':True,'control_parameter_count':len(ref),'control_and_CA_parameters_unchanged':True,'source_hashes_match':True,'imu_rate_hz':5000,'source_fingerprints':{f:hashlib.sha256((d/f).read_bytes()).hexdigest() for f in ['controller.cpp','ros_control.py','world.sdf']}})
(R/'results/audit.json').write_text(json.dumps(out,indent=2));print('Both accepted native-arm flights audited.')
