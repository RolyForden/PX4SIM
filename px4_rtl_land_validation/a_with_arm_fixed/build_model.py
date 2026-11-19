"""Integrate the existing A aircraft and native Delta through a measured mount."""
from pathlib import Path
import argparse,json,copy,hashlib,xml.etree.ElementTree as E
import numpy as np
from mass_properties import *
R=Path(__file__).resolve().parent;V=R.parent;N=V/'delta_dynamics/native';F=V/'a_morphing/airborne/models/a_air/model.sdf'
def add(p,t,s=None,**attrs):
 x=E.SubElement(p,t,attrs)
 if s is not None:x.text=str(s)
 return x

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--payload',type=float,default=.05);args=ap.parse_args();assert 0<=args.payload<=.2
 assets=json.loads((R/'assets.json').read_text());cfg=json.loads((N/'config.json').read_text());source=json.loads((V/'A-UAV-20261115/parameters/uav_parameters.json').read_text(encoding='utf-8-sig'));rot=np.array(cfg['R_flu_from_cad']);targetM=source['mass_kg'];targetc=rot@source['center_of_mass_m'];targetI=rot@np.array(source['inertia_com_kgm2'])@rot.T
 original=E.parse(F).getroot().find('model');air=copy.deepcopy(original);before=aggregate(records(air));base=air.find("link[@name='base_link']");other=[a for a in records(air) if a['name']!='base_link'];baseM=float(base.findtext('inertial/mass'));basec=(targetM*targetc-sum(x['mass']*x['com'] for x in other))/baseM;baseI=targetI+targetM*parallel(targetc)-sum(x['I']+x['mass']*parallel(x['com']) for x in other)-baseM*parallel(basec)
 assert valid_tensor(baseI);set_inertia(base,baseM,basec,baseI);after=aggregate(records(air));np.testing.assert_allclose(after['com_m'],targetc,atol=1e-12);np.testing.assert_allclose(after['inertia_com_kg_m2'],targetI,atol=1e-12);assert abs(after['mass_kg']-targetM)<1e-12
 # Retain the aircraft geometry and masses. Lock its morphing/rotor joints for this bench.
 for j in air.findall('joint'):
  j.set('type','fixed')
  for a in list(j):
   if a.tag in ['axis','axis2']:j.remove(a)
 for p in list(air.findall('plugin')):air.remove(p)
 air.set('name','a_with_arm_fixed');add(air,'pose','0 0 0.65 0 0 0');air.find('self_collide').text='false'
 base.find('visual/geometry/mesh/uri').text='model://a_with_arm_fixed/meshes/body.stl'
 j=add(air,'joint',name='fixture_fixed',type='fixed');add(j,'parent','world');add(j,'child','base_link')
 mount=np.array(assets['mount_origin_flu_m']);l=add(air,'link',name='arm_mount');add(l,'pose',' '.join(map(str,[*mount,0,0,0])));add(l,'gravity','true');set_inertia(l,assets['fixed_mechanism_mass_kg'],np.array(assets['mount_com_flu_m'])-mount,np.array(assets['mount_inertia_com_flu_kg_m2']));v=add(l,'visual',name='mount');g=add(v,'geometry');me=add(g,'mesh');add(me,'uri','model://a_with_arm_fixed/meshes/mount.stl');mat=add(v,'material');add(mat,'ambient','0.45 0.50 0.60 1');add(mat,'diffuse','0.45 0.50 0.60 1')
 j=add(air,'joint',name='arm_mount_joint',type='fixed');add(j,'parent','base_link');add(j,'child','arm_mount')
 # Reuse the validated native linkage structure, excluding its dummy 1 kg aircraft.
 native=E.parse(N/'world.sdf');nm=native.find("world/model[@name='native_structure']");platform=copy.deepcopy(native.find("world/model[@name='native_platform']"));platform.set('name','integrated_platform');platform.find('pose').text=' '.join(map(str,[*list(np.array(cfg['home_m'])+[0,0,.65]),0,0,0]))
 for link in nm.findall('link'):
  if link.get('name')!='base_link':air.append(copy.deepcopy(link))
 for joint in nm.findall('joint'):
  if joint.get('name')=='fixed_body':continue
  joint=copy.deepcopy(joint)
  if joint.findtext('parent')=='base_link':joint.find('parent').text='arm_mount'
  air.append(joint)
 # Payload is generated explicitly instead of relying on the last native session's load.
 for joint in list(platform.findall('joint')):platform.remove(joint)
 for link in list(platform.findall('link')):
  if link.get('name')!='platform':platform.remove(link)
 if args.payload>0:
  l=add(platform,'link',name='payload');add(l,'pose','0 0 -.04 0 0 0');add(l,'gravity','true');set_inertia(l,args.payload,[0,0,0],np.eye(3)*max(args.payload*.00015,1e-7));v=add(l,'visual',name='load');g=add(v,'geometry');box=add(g,'box');add(box,'size','.03 .03 .03');mat=add(v,'material');add(mat,'ambient','.8 .2 .2 1');add(mat,'diffuse','.8 .2 .2 1');j=add(platform,'joint',name='payload_fixed',type='fixed');add(j,'parent','platform');add(j,'child','payload')
 pl=add(air,'plugin',filename='liba_with_arm_fixed.so',name='a_with_arm_fixed::Controller');add(pl,'payload',args.payload);add(pl,'gravity',9.81)
 for name,joint in [('mount','arm_mount_joint'),('fixture','fixture_fixed')]:
  pl=add(air,'plugin',filename='libwrench_logger.so',name='study::WrenchLogger');add(pl,'joint',joint);add(pl,'output',f'/work/a_with_arm_fixed/results/{name}_wrench.jsonl')
 tree=E.parse(V/'delta_fixed/world.sdf');w=tree.getroot().find('world');w.set('name','a_with_arm_fixed_world');w.find('physics/max_step_size').text='.0002';w.find('gravity').text='0 0 -9.81'
 for tag in ['include','model']:
  for x in list(w.findall(tag)):
   if x.get('name')!='ground_plane':w.remove(x)
 w.append(air);w.append(platform);E.indent(tree);tree.write(R/'world.sdf',encoding='utf-8',xml_declaration=True)
 full=records(air,True)+records(platform,True);combined=aggregate(full);combined['com_m'][2]-=.65
 mech=[x for x in full if x['name'] not in {l.get('name') for l in original.findall('link')}];mp=aggregate(mech);mp['com_m'][2]-=.65
 assert all(valid_tensor(x['I']) for x in full)
 ledger={'aircraft_baseline_kg':targetM,'removed_mechanism_bom_mass_kg':0,'fixed_mechanism_test_mass_kg':assets['fixed_mechanism_mass_kg'],'upper_arm_mass_kg':3*.03,'lower_rod_mass_kg':6*.012,'platform_mass_kg':.05,'proxy_mass_kg':.00006,'payload_mass_kg':args.payload,'expected_total_kg':targetM+assets['fixed_mechanism_mass_kg']+.09+.072+.05+.00006+args.payload,'baseline_mechanism_mass_assignment':'B机械夹爪 and connector have no BOM mass; no subtraction','unlocated_baseline_mass_kg':source['unmatched_mass_at_cad_origin_kg']}
 assert abs(combined['mass_kg']-ledger['expected_total_kg'])<1e-12
 audit={'mass_ledger':ledger,'original_aircraft_sdf_aggregate':before,'target_aircraft_mass_properties':{'mass_kg':targetM,'com_m':targetc.tolist(),'inertia_com_kg_m2':targetI.tolist()},'corrected_aircraft_aggregate':after,'new_base_inertia_kg_m2':baseI.tolist(),'combined_at_home_flu':combined,'mechanism_at_home_flu':mp,'all_link_inertias_valid':True,'body_visual_removed_parts':[121,122],'mount_origin_flu_m':mount.tolist(),'original_sources_sha256':{str(p.relative_to(V)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [F,V/'a_morphing/config/morph-config.json',N/'controller.cpp',V/'A-UAV-20261115/cad/airframe.step']},'flight_enabled':False,'frame':'FLU, tensors about each stated COM; signed off-diagonal entries retained'}
 (R/'mass-audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2));(R/'settings.json').write_text(json.dumps({'payload':args.payload,'gravity':9.81,'proxy_total_mass_kg':.00006,'base_rpy_rad':[0,0,0],'mass_ledger':ledger,'mount_origin_flu_m':mount.tolist()},indent=2));print(json.dumps(ledger,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
