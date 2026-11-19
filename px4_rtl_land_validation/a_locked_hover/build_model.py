"""Rigidly lock the integrated arm, preserve its mass tensor, restore rotor physics."""
from pathlib import Path
import sys,json,argparse,copy,hashlib,xml.etree.ElementTree as E
import numpy as np
R=Path(__file__).resolve().parent;V=R.parent;sys.path.insert(0,str(V/'a_with_arm_fixed'))
from mass_properties import records,aggregate,pose,rotation
ap=argparse.ArgumentParser();ap.add_argument('--payload',type=float,choices=[0,.05],default=0);args=ap.parse_args();case='empty' if args.payload==0 else 'payload_50g'
src=V/'a_with_arm_fixed/results'/case/'world.sdf';tree=E.parse(src);air=copy.deepcopy(tree.find("world/model[@name='a_with_arm_fixed']"));platform=tree.find("world/model[@name='integrated_platform']");before=aggregate(records(air,True)+records(platform,True));before['com_m'][2]-=.65
for e in list(air.findall('plugin')):air.remove(e)
for e in list(air.findall('pose')):air.remove(e)
fixture=air.find("joint[@name='fixture_fixed']");air.remove(fixture)
for j in air.findall('joint'):
 j.set('type','fixed')
 for a in list(j):
  if a.tag in ['axis','axis2']:j.remove(a)
# Flatten the platform model into the same body-coordinate system before locking it.
a_pose=pose(tree.find("world/model[@name='a_with_arm_fixed']/pose"));p_pose=pose(platform.find('pose'));assert np.max(np.abs(a_pose[3:]))<1e-12 and np.max(np.abs(p_pose[3:]))<1e-12
for l in platform.findall('link'):
 l=copy.deepcopy(l);p=pose(l.find('pose'));p[:3]+=p_pose[:3]-a_pose[:3];node=l.find('pose')
 if node is None:node=E.SubElement(l,'pose')
 node.text=' '.join(map(str,p));air.append(l)
for j in platform.findall('joint'):air.append(copy.deepcopy(j))
j=E.SubElement(air,'joint',name='locked_platform',type='fixed');E.SubElement(j,'parent').text='arm_mount';E.SubElement(j,'child').text='platform'
original=E.parse(V/'a_morphing/airborne/models/a_air/model.sdf').getroot().find('model')
for i in range(4):
 old=air.find(f"joint[@name='rotor_{i}_joint']");air.remove(old);air.append(copy.deepcopy(original.find(f"joint[@name='rotor_{i}_joint']")))
for p in original.findall('plugin'):
 if 'MulticopterMotorModel' in p.get('name',''):air.append(copy.deepcopy(p))
air.set('name','a_locked');after=aggregate(records(air));np.testing.assert_allclose(after['mass_kg'],before['mass_kg'],atol=1e-12);np.testing.assert_allclose(after['com_m'],before['com_m'],atol=1e-12);np.testing.assert_allclose(after['inertia_com_kg_m2'],before['inertia_com_kg_m2'],atol=1e-12)
assert all(j.findtext('parent')!='world' for j in air.findall('joint'));assert sum(j.get('type')=='revolute' for j in air.findall('joint'))==4
model=E.Element('sdf',version='1.9');model.append(air);E.indent(model);E.ElementTree(model).write(R/'models/a_locked/model.sdf',encoding='utf-8',xml_declaration=True)
(R/'models/a_locked/model.config').write_text('<model><name>A locked arm hover</name><version>1</version><sdf version="1.9">model.sdf</sdf><description>A integrated rigid-lock hover baseline</description></model>')
world=E.parse(V/'a_morphing/airborne/world.sdf');world.find('world').set('name','a_locked_world');world.find('world/gravity').text='0 0 -9.81';E.indent(world);world.write(R/'world.sdf',encoding='utf-8',xml_declaration=True)
c=np.array(after['com_m']);r=[];axes=[];km=[];motor=[]
for i in range(4):
 l=air.find(f"link[@name='rotor_{i}']");p=pose(l.find('pose'))[:3];r.append((p-c)*[1,-1,-1]);axes.append([0,0,-1]);km.append(.016 if i<2 else -.016);mp=next(p for p in air.findall('plugin') if p.findtext('motorNumber')==str(i));motor.append({'turning':mp.findtext('turningDirection'),'kT':float(mp.findtext('motorConstant')),'omega_max':float(mp.findtext('maxRotVelocity'))})
r=np.array(r);axes=np.array(axes);B=np.vstack([(np.cross(r,axes)-np.array(km)[:,None]*axes).T,axes.T]);A=B[[0,1,2,5]];weight=after['mass_kg']*9.81;thrust=np.linalg.solve(A,[0,0,0,-weight]);maxT=np.array([x['kT']*x['omega_max']**2 for x in motor]);omega=np.sqrt(thrust/np.array([x['kT'] for x in motor]));assert np.linalg.matrix_rank(A)==4 and np.all(thrust>0) and np.all(thrust<maxT)
# Geometry is updated; the MC/MPC control parameters are copied without tuning.
ref=R/'reference-control-parameters.json';params=json.loads(ref.read_text());control={k:v for k,v in params.items() if k.startswith(('MC_','MPC_'))};lines=['#!/bin/sh','. ${R}etc/init.d/rc.mc_defaults','PX4_SIMULATOR=gz','PX4_GZ_WORLD=a_locked_world','PX4_SIM_MODEL=a_locked','param set SIM_GZ_EN 1','param set CA_AIRFRAME 0','param set CA_ROTOR_COUNT 4']
for i in range(4):
 for key,value in zip(['PX','PY','PZ'],r[i]):lines.append(f'param set CA_ROTOR{i}_{key} {value:.12g}')
 for key,value in zip(['AX','AY','AZ'],axes[i]):lines.append(f'param set CA_ROTOR{i}_{key} {value}')
 lines +=[f'param set CA_ROTOR{i}_CT 6.5',f'param set CA_ROTOR{i}_KM {km[i]}',f'param set SIM_GZ_EC_FUNC{i+1} {101+i}',f'param set SIM_GZ_EC_MIN{i+1} 150',f'param set SIM_GZ_EC_MAX{i+1} 1000']
for key,value in sorted(control.items()):lines.append(f'param set {key} {value:.12g}')
lines+=['param set COM_RC_IN_MODE 1','param set COM_DISARM_LAND 2','param set NAV_DLL_ACT 2','param set SDLOG_MODE 0']
(R/'4995_a_locked').write_text('\n'.join(lines)+'\n');(R/'reference-control-parameters.json').write_text(json.dumps(control,indent=2))
meta={'payload_kg':args.payload,'mass_properties':after,'source_integrated_sdf':str(src.relative_to(V)),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256((R/'models/a_locked/model.sdf').read_bytes()).hexdigest(),'rigid_lock_method':'all arm/morph joints fixed, platform flattened into same model, no pose controller','world_fixed_joint_count':0,'rotor_count':4,'rotor_geometry_frd_m':r.tolist(),'rotor_axes_frd':axes.tolist(),'rotor_KM_m':km,'effectiveness_B_physical':B.tolist(),'rank':int(np.linalg.matrix_rank(A)),'required_hover_thrust_n':thrust.tolist(),'max_thrust_n':maxT.tolist(),'thrust_margin_n':(maxT-thrust).tolist(),'expected_hover_omega_rad_s':omega.tolist(),'control_parameter_reference':str(ref.relative_to(V)),'PID_modified':False,'gravity_m_s2':9.81,'propulsion_source':'inherited simulation coefficients, not measured propulsion'}
(R/'preflight.json').write_text(json.dumps(meta,indent=2));print(json.dumps({k:meta[k] for k in ['payload_kg','mass_properties','required_hover_thrust_n','thrust_margin_n','expected_hover_omega_rad_s','rank']},indent=2))
