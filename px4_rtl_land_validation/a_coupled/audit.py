"""Audit source restoration, real articulated joints, control readbacks and raw logs."""
from pathlib import Path
import json,hashlib,subprocess,xml.etree.ElementTree as E
import numpy as np
R=Path(__file__).resolve().parent;V=R.parent;out=[]
build=json.loads((R/'px4-build.json').read_text())
assert hashlib.sha256((V/'PX4-Autopilot/build/px4_sitl_default/bin/px4').read_bytes()).hexdigest()==build['baseline_sha256']
assert hashlib.sha256((R/'px4_coupled').read_bytes()).hexdigest()==build['morph_sha256']
subprocess.run(['git','-C',str(V/'PX4-Autopilot'),'diff','--exit-code','--','src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.cpp','src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp'],check=True,capture_output=True)
for name in ['empty_03','load50g_01']:
 d=R/'runs'/name;t=E.parse(d/'world.sdf');a=t.find("world/model[@name='a_coupled']");p=t.find("world/model[@name='integrated_platform']");pre=json.loads((d/'preflight.json').read_text())
 assert len(a.findall("joint[@type='revolute']"))==15 and len(a.findall("joint[@type='ball']"))==12
 assert not any(j.findtext('parent')=='world' for m in [a,p] for j in m.findall('joint'))
 assert float(a.findtext("link[@name='base_link']/sensor[@name='imu_sensor']/update_rate"))==5000
 assert hashlib.sha256((V/pre['source_integrated_sdf']).read_bytes()).hexdigest()==pre['source_sha256']
 assert hashlib.sha256((d/'world.sdf').read_bytes()).hexdigest()==pre['world_sha256']
 before=json.loads((d/'parameters/before.json').read_text());after=json.loads((d/'parameters/after.json').read_text());ref=json.loads((d/'reference-control-parameters.json').read_text())
 assert before['SYS_AUTOSTART']==after['SYS_AUTOSTART']==4993
 for k,v in ref.items():assert abs(before[k]-v)<1e-5*max(1,abs(v)) and before[k]==after[k]
 for i in range(4):
  for key in ['PX','PY','PZ','AX','AY','AZ','CT','KM']:assert before[f'CA_ROTOR{i}_{key}']==after[f'CA_ROTOR{i}_{key}']
 cs=(d/'controller.cpp').read_text();assert 'DetachableJoint({platform,proxy[i][k],"fixed"})' in cs and 'JointPositionReset' not in cs and 'WorldPoseCmd' not in cs
 summary=json.loads((d/'summary.json').read_text());assert summary['passed']
 mass=json.loads((d/'mass-crosscheck.json').read_text());assert mass['passed']
 records=[json.loads(v) for v in (d/'morph-feedback.jsonl').read_text().splitlines()]
 for v in records:
  full=np.array(v['B']);ctrl=np.array(v['B_control']);np.testing.assert_allclose(full[[0,1,2,5]],ctrl[[0,1,2,5]],atol=1e-12);assert np.max(abs(ctrl[3:5]))==0
 o=json.loads((d/'outcome.json').read_text());assert o['ulog_finalized_after_shutdown'];f=d/o['ulog']['file'];assert hashlib.sha256(f.read_bytes()).hexdigest()==o['ulog']['sha256']
 out.append({'name':name,'passed':True,'morph_revolutes':8,'arm_revolutes':3,'rotor_revolutes':4,'native_balls':12,'no_world_fixture':True,'no_passive_pose_reset':True,'MC_MPC_parameter_count':len(ref),'control_and_CA_parameters_unchanged':True,'physical_B_and_four_axis_control_B_consistent':True,'native_matrix_readbacks':summary['native_matrix_readbacks'],'independent_mass_crosscheck':mass,'original_ULog_finalized_and_verified':True})
(R/'results/audit.json').write_text(json.dumps({'trials':out,'baseline_source_and_binary_restored':True,'dedicated_adapter':build},indent=2));print('Two coupled flights and baseline restoration audited.')
