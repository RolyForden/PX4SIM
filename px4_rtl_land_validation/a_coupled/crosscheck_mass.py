"""Independent articulated mass-tensor reconstruction from measured joint angles."""
from pathlib import Path
import sys,json,copy,xml.etree.ElementTree as E
import numpy as np
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R.parent/'a_with_arm_fixed'))
from mass_properties import records,aggregate,tensor,pose,rotation,valid_tensor
from kinematics import CONFIG,HOME,rotation as axis_rotation
D=R/'runs'/sys.argv[1];root=E.parse(D/'world.sdf');model=root.find("world/model[@name='a_coupled']");platform=root.find("world/model[@name='integrated_platform']")
arm_names={f'upper_{i}' for i in range(3)}|{f'rod_{i}_{k}' for i in range(3) for k in range(2)}|{f'tip_{i}_{k}' for i in range(3) for k in range(2)}
morph_names={f'{k}_{i}' for i in range(4) for k in ['yaw_frame','arm','rotor']}
initial=records(model);rigid=[v for v in initial if v['name'] not in arm_names|morph_names]
mcfg=json.loads((R.parent/'a_morphing/config/morph-config.json').read_text())
def inert(m,name):
 a=m.find(f"link[@name='{name}']/inertial");return float(a.findtext('mass')),pose(a.find('pose'))[:3],tensor(a.find('inertia'))
def predicted(s,qm):
 rows=list(rigid);p=np.array(s['position_m']);Qp=rotation(s['platform_rpy_rad'])
 for i,leg in enumerate(CONFIG['legs']):
  S=np.array(leg['shoulder_m']);Q=axis_rotation(np.array(leg['axis']),s['q_rad'][i]);m,c,I=inert(model,f'upper_{i}');rows.append({'name':f'upper_{i}','mass':m,'com':S+Q@c,'I':Q@I@Q.T})
  for k in range(2):
   e=S+Q@(np.array(leg['elbows_home_m'][k])-S);b=p+Qp@np.array(leg['platform_offsets_m'][k]);z=(b-e)/np.linalg.norm(b-e);a=np.array(leg['axis']);x=a-z*np.dot(a,z);x/=np.linalg.norm(x);Qr=np.column_stack([x,np.cross(z,x),z]);m,c,I=inert(model,f'rod_{i}_{k}');rows.append({'name':'rod','mass':m,'com':e+Qr@c,'I':Qr@I@Qr.T});m,c,I=inert(model,f'tip_{i}_{k}');rows.append({'name':'tip','mass':m,'com':b+Qp@c,'I':Qp@I@Qp.T})
 for name in ['platform','payload']:
  if platform.find(f"link[@name='{name}']") is None:continue
  m,c,I=inert(platform,name);offset=pose(platform.find(f"link[@name='{name}']/pose"))[:3];rows.append({'name':name,'mass':m,'com':p+Qp@(offset+c),'I':Qp@I@Qp.T})
 for i,leg in enumerate(mcfg['arms']):
  h=np.array(leg['hinge_flu_m']);Y=axis_rotation(np.array([0,0,1]),qm[2*i]);T=Y@axis_rotation(np.array(leg['pitch_axis']),qm[2*i+1])
  for name in [f'yaw_frame_{i}',f'arm_{i}',f'rotor_{i}']:
   l=copy.deepcopy(next(v for v in initial if v['name']==name));Q=Y if name.startswith('yaw_frame') else T;l['com']=h+Q@(l['com']-h);l['I']=Q@l['I']@Q.T;rows.append(l)
 return aggregate(rows)

a=[json.loads(v) for v in (D/'arm-feedback.jsonl').read_text().splitlines()];m=[json.loads(v) for v in (D/'morph-feedback.jsonl').read_text().splitlines()];ts=np.array([v['sim_s'] for v in m]);qs=np.radians([v['q_deg'] for v in m]);errors=[]
for s in a[::20]:
 if s['sim_s']<ts[0] or s['sim_s']>ts[-1]:continue
 qm=np.array([np.interp(s['sim_s'],ts,qs[:,i]) for i in range(8)]);p=predicted(s,qm);ce=np.linalg.norm(np.array(p['com_m'])-s['com_flu_m']);ie=np.max(np.abs(np.array(p['inertia_com_kg_m2'])-np.array(s['inertia_com_flu_kg_m2']).reshape(3,3)));errors.append([s['sim_s'],float(ce),float(ie)])
assert errors
out={'samples':len(errors),'max_com_error_m':max(v[1] for v in errors),'max_tensor_error_kg_m2':max(v[2] for v in errors),'com_tolerance_m':.0001,'tensor_tolerance_kg_m2':.00001,'method':'independent CAD FK plus measured platform pose and interpolated morph joints, signed tensor and parallel-axis aggregation'}
out['passed']=out['max_com_error_m']<out['com_tolerance_m'] and out['max_tensor_error_kg_m2']<out['tensor_tolerance_kg_m2'];(D/'mass-crosscheck.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if not out['passed']:raise SystemExit('Independent mass reconstruction mismatch')
