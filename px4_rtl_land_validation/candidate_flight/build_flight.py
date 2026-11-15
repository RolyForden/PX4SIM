"""Independent rigid candidate SITL model; no PX4 source or stock model edits."""
import json,pathlib,struct,copy,shutil,xml.etree.ElementTree as E
import numpy as np
from integrate_handoff import load_baseline,matched_motor,physical_carrier_decomposition
ROOT=pathlib.Path(__file__).resolve().parent;PX=ROOT.parent/'PX4-Autopilot';modeldir=ROOT/'models'/'candidate_rigid';modeldir.mkdir(parents=True,exist_ok=True)
meshdir=modeldir/'meshes';meshdir.mkdir(exist_ok=True)
cad=json.loads((ROOT/'cad_inventory.json').read_text());parts=cad['parts'];settings=json.loads((ROOT/'settings.json').read_text())
DT=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
# Provisional mapping identical to reference convention: CAD mm -> FLU m.
R=np.array(settings.get('R_flu_from_cad',[[0,0,-1],[-1,0,0],[0,1,0]]),dtype=float)
np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12);assert np.linalg.det(R)>0
handoff=load_baseline(ROOT,settings,R)
props=[r for r in parts if r['name']=='螺旋桨'];motors=[r for r in parts if r['name']=='无刷电机'];assert len(props)==len(motors)==4
# PX4 rotor order: front-right, rear-left, front-left, rear-right.
signs=[(1,-1),(-1,1),(1,1),(-1,-1)]
ordered=[]
for sx,sy in signs:
 m=next(r for r in motors if (-r['uniform_volume_centroid_cad_mm'][2]>0)==(sx>0) and (-r['uniform_volume_centroid_cad_mm'][0]>0)==(sy>0))
 mc=np.array(m['uniform_volume_centroid_cad_mm']);p=min(props,key=lambda r:np.linalg.norm(np.array(r['uniform_volume_centroid_cad_mm'])[[0,2]]-mc[[0,2]]))
 # Approximate shaft horizontal centre from motor volume centroid; not surveyed axis.
 origin=R@np.array([mc[0],8.5,mc[2]])/1000
 if handoff:_,origin,_=matched_motor(handoff,sx,sy)
 ordered.append((m,p,origin))
propids={p['index'] for m,p,o in ordered}
def triangles(record):
 raw=(ROOT/record['mesh']).read_bytes();a=np.frombuffer(raw,dtype=DT,count=struct.unpack_from('<I',raw,80)[0],offset=84).copy();a['vertices']=(a['vertices']@R.T)/1000;a['normal']=a['normal']@R.T;return a
body=np.concatenate([triangles(r) for r in parts if r['index'] not in propids])
def write(f,a):
 with f.open('wb') as fp:fp.write(b'Candidate CAD; provisional FLU metres; NOT calibrated'.ljust(80,b' '));fp.write(struct.pack('<I',len(a)));fp.write(a.tobytes())
write(meshdir/'body.stl',body)
for i,(m,p,o) in enumerate(ordered):a=triangles(p);a['vertices']-=o;write(meshdir/f'prop_{i}.stl',a)
low=body['vertices'].reshape(-1,3).min(0);high=body['vertices'].reshape(-1,3).max(0)
config={'status':'version-unconfirmed CAD; fixed rigid-body flight demonstrator, not final-aircraft validation','source_sha256':cad['sha256'],
 'airframe_id':4999,'model':'candidate_rigid','world':'candidate_world','total_mass_kg':settings['total_mass_kg'],'mass_source':settings['mass_source'],
 'base_inertia_kg_m2':settings['base_inertia_kg_m2'],'inertia_source':settings['inertia_source'],'reference_point_flu_m':[0,0,0],'reference_point_source':'CAD origin assumed CoG',
 'rotor_mass_kg':settings['rotor_mass_kg'],'motor_constant_N_per_rad_s2':settings['motor_constant_N_per_rad_s2'],'motor_constant_source':'borrowed X500 coefficient, NOT actual motor characterization','moment_ratio_m':settings['moment_ratio_m'],'max_rad_s':settings['max_rad_s'],
 'geometry_bounds_flu_m':[low.tolist(),high.tolist()],'rotors':[],
 'limitations':['all elastic/servo/gripper joints locked','all effective thrust axes assumed model +Z; CAD installation orientations not calibrated','rotation directions assigned for simulation only','simplified collision boxes','mass/CoG/inertia and actuator dynamics uncalibrated']}
if handoff:
 baseM,basec,baseI,carrierM,carrierI=physical_carrier_decomposition(handoff,[o for m,p,o in ordered],settings['rotor_mass_kg'])
 config.update({'mass_properties_source':handoff['source'],'mass_properties_source_sha256':handoff['source_sha256'],'mass_properties_status':'BOM_WEIGHTED_PROVISIONAL_NOT_MEASURED',
  'total_mass_kg':handoff['mass_kg'],'center_of_mass_flu_m':handoff['com_flu_m'].tolist(),'inertia_com_flu_kg_m2':handoff['inertia_com_flu_kg_m2'].tolist(),
  'frame_status':'PROVISIONAL debug frame only; aircraft front/up NOT confirmed','R_flu_from_cad':R.tolist(),'rotor_mass_kg':carrierM,'rotor_carrier_inertia_kg_m2':carrierI.tolist(),
  'base_mass_kg':baseM,'base_com_flu_m':basec.tolist(),'base_inertia_com_flu_kg_m2':baseI.tolist(),'base_inertia_kg_m2':np.diag(baseI).tolist(),
  'reference_point_flu_m':handoff['com_flu_m'].tolist(),'reference_point_source':'provisional BOM-weighted whole-assembly CoG',
  'unlocated_mass_kg':handoff['raw']['unmatched_mass_at_cad_origin_kg'],'cad_components_without_assigned_mass':handoff['raw']['cad_components_without_bom_mass'],
  'source_assumptions':handoff['raw']['assumptions'],'user_reported_propulsion':handoff['raw']['propulsion'],
  'mass_bookkeeping':'whole BOM mass/inertia includes original props; prop actuator carriers subtracted from base; no added HQ meshes or second prop masses',
  'limitations':['all elastic/servo/gripper joints locked','front/up frame is provisional, NOT confirmed aircraft convention','motor installation origins used as assumed force points, NOT confirmed thrust centres','effective thrust axes assumed model +Z; rotor spin assignment is illustrative','0.393kg unlocated mass at CAD origin; five CAD components have no BOM mass','whole signed inertia is approximate BOM-weighted, not measured','9g prop carriers use simplified spin inertias/CoG placement; whole rigid tensor preserved','simplified collision boxes; propulsion borrowed and uncalibrated']})
base_sdf=E.parse(PX/'Tools/simulation/gz/models/x500_base/model.sdf');base=copy.deepcopy(base_sdf.find('model/link'));base.set('name','base_link')
for tag in ['visual','collision']:
 for child in list(base.findall(tag)):base.remove(child)
base.find('inertial/mass').text=str(config['total_mass_kg']-4*config['rotor_mass_kg'])
for k,v in zip(['ixx','iyy','izz'],config['base_inertia_kg_m2']):base.find('inertial/inertia/'+k).text=str(v)
if handoff:
 E.SubElement(base.find('inertial'),'pose').text=' '.join(str(float(v)) for v in [*basec,0,0,0])
 for key,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]:
  field=base.find('inertial/inertia/'+key)
  if field is None:field=E.SubElement(base.find('inertial/inertia'),key)
  field.text=str(float(baseI[i,j]))

def add(p,t,text=None,**kw):
 x=E.SubElement(p,t,kw)
 if text is not None:x.text=str(text)
 return x
def vec(a):return ' '.join(str(float(x)) for x in a)
def visual(link,name,uri):
 v=add(link,'visual',name=name);g=add(v,'geometry');me=add(g,'mesh');add(me,'uri',uri);mat=add(v,'material');add(mat,'ambient','.35 .48 .65 1');add(mat,'diffuse','.35 .48 .65 1')
visual(base,'candidate_geometry','model://candidate_rigid/meshes/body.stl')
for name,pos,size in [('core',[0,0,0],[.1,.1,.08]),('landing_envelope',[0,0,float(low[2]+.01)],[.16,.16,.02])]:
 c=add(base,'collision',name=name);add(c,'pose',vec([*pos,0,0,0]));g=add(c,'geometry');b=add(g,'box');add(b,'size',vec(size))
sdf=E.Element('sdf',version='1.9');model=add(sdf,'model',name='candidate_rigid');add(model,'pose',f'0 0 {-float(low[2])+.04} 0 0 0');add(model,'self_collide','false');model.append(base)
stock=E.parse(PX/'Tools/simulation/gz/models/x500/model.sdf')
for i,(motor,prop,origin) in enumerate(ordered):
 l=add(model,'link',name=f'rotor_{i}');add(l,'pose',vec([*origin,0,0,0]));ine=add(l,'inertial');add(ine,'mass',config['rotor_mass_kg']);inertia=add(ine,'inertia')
 for k,v in [('ixx',float(carrierI[0,0]) if handoff else 1e-5),('iyy',float(carrierI[1,1]) if handoff else 1e-5),('izz',float(carrierI[2,2]) if handoff else 2e-5)]:add(inertia,k,v)
 visual(l,'prop',f'model://candidate_rigid/meshes/prop_{i}.stl')
 j=add(model,'joint',name=f'rotor_{i}_joint',type='revolute');add(j,'parent','base_link');add(j,'child',f'rotor_{i}');ax=add(j,'axis');add(ax,'xyz','0 0 1');lim=add(ax,'limit');add(lim,'lower',-1e16);add(lim,'upper',1e16)
 plugin=copy.deepcopy(stock.findall('model/plugin')[i]);model.append(plugin)
 plugin.find('motorConstant').text=str(config['motor_constant_N_per_rad_s2']);plugin.find('momentConstant').text=str(config['moment_ratio_m']);plugin.find('maxRotVelocity').text=str(config['max_rad_s'])
 spin=plugin.findtext('turningDirection');km=config['moment_ratio_m'] if spin=='ccw' else -config['moment_ratio_m']
 frd=(origin-np.array(config['reference_point_flu_m']))*np.array([1,-1,-1]);config['rotors'].append({'index':i,'motor_part':motor['index'],'prop_part':prop['index'],'position_frd_m':frd.tolist(),'axis_frd':[0,0,-1],'km_m':km,'spin':spin})
if handoff:
 for rotor,(sx,sy) in zip(config['rotors'],signs):
  motor,position,Q=matched_motor(handoff,sx,sy)
  rotor.update({'source_component':motor['cad_component'],'placement_origin_cad_m':motor['placement_origin_m'],'position_flu_m':position.tolist(),'orientation_flu_from_motor':Q.tolist(),'orientation_interpretation':'source columns transposed; retained as metadata, not used to infer thrust axis','force_point_status':'motor placement origin assumed for software debug, not confirmed rotor centre'})
E.indent(sdf);E.ElementTree(sdf).write(modeldir/'model.sdf',encoding='utf-8',xml_declaration=True)
(modeldir/'model.config').write_text('<model><name>candidate_rigid</name><version>0.1</version><sdf version="1.9">model.sdf</sdf><description>Unconfirmed CAD, rigid assumed dynamics, not final aircraft</description></model>')
world=E.parse(PX/'Tools/simulation/gz/worlds/default.sdf');world.find('world').set('name','candidate_world');world.find('world/physics/max_step_size').text=str(settings['max_step_size_s']);world.find('world/physics/real_time_factor').text=str(settings['real_time_factor']);world.write(ROOT/'candidate_world.sdf',encoding='utf-8',xml_declaration=True)
runtime=ROOT/'runtime';runtime.mkdir(exist_ok=True)
if not (runtime/'etc').exists():shutil.copytree(PX/'build/px4_sitl_default/etc',runtime/'etc')
air=['#!/bin/sh','# Independent candidate rigid demo, assumptions in flight_config.json','. ${R}etc/init.d/rc.mc_defaults',
 'PX4_SIMULATOR=gz','PX4_GZ_WORLD=candidate_world','PX4_SIM_MODEL=candidate_rigid','param set-default SIM_GZ_EN 1','param set-default CA_AIRFRAME 0','param set-default CA_ROTOR_COUNT 4']
for rotor in config['rotors']:
 i=rotor['index']
 for key,value in zip(['PX','PY','PZ'],rotor['position_frd_m']):air.append(f'param set-default CA_ROTOR{i}_{key} {value:.9f}')
 air.append(f'param set-default CA_ROTOR{i}_KM {rotor["km_m"]}')
 for key,value in [('FUNC',101+i),('MIN',150),('MAX',1000)]:air.append(f'param set-default SIM_GZ_EC_{key}{i+1} {value}')
air+=[f"param set-default MPC_THR_HOVER {settings['hover_normalized']}",'param set-default NAV_DLL_ACT 2']
air=[line.replace('param set-default','param set') for line in air]
(ROOT/'4999_candidate_rigid').write_text('\n'.join(air)+'\n');shutil.copyfile(ROOT/'4999_candidate_rigid',runtime/'etc/init.d-posix/airframes/4999_candidate_rigid')
(ROOT/'startup.sh').write_text('#!/bin/sh\n. ${R}etc/init.d-posix/rcS\nmavlink start -x -u 14651 -o 14650 -t 127.0.0.1 -m onboard -r 1000000\n')
(ROOT/'flight_config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2));print(json.dumps(config,ensure_ascii=False,indent=2))
