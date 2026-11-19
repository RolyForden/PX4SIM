"""Fixed body, dynamic rods/platform; force-coupled compliant spherical closures."""
from pathlib import Path
import json,argparse,math
import numpy as np
import xml.etree.ElementTree as E
R=Path(__file__).resolve().parent;F=R.parent/'delta_fixed';cfg=json.loads((R/'config.json').read_text())
def add(p,t,s=None,**a):
 x=E.SubElement(p,t,a)
 if s is not None:x.text=str(s)
 return x
def vec(a):return ' '.join(f'{float(v):.14g}' for v in a)
def rpy(m):
 pitch=math.asin(np.clip(-m[2,0],-1,1));return [math.atan2(m[2,1],m[2,2]),pitch,math.atan2(m[1,0],m[0,0])]
def link(m,n,p,mass,mesh=None,angles=[0,0,0],com=[0,0,0],inertia=[.0002]*3):
 l=add(m,'link',name=n);add(l,'pose',vec([*p,*angles]));add(l,'gravity','true');i=add(l,'inertial');add(i,'mass',mass);add(i,'pose',vec([*com,0,0,0]));ii=add(i,'inertia')
 for name,value in zip(['ixx','iyy','izz'],inertia):add(ii,name,value)
 if mesh:
  v=add(l,'visual',name=n);g=add(v,'geometry');me=add(g,'mesh');add(me,'uri',f'model://a_delta_fixed/meshes/{mesh}.stl');ma=add(v,'material');col='0.38 0.48 0.6 1' if n=='base_link' else ('0.95 0.45 0.12 1' if n.startswith('upper') else '0.25 0.65 0.4 1');add(ma,'ambient',col);add(ma,'diffuse',col)
 return l
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',default='world.sdf');ap.add_argument('--payload',type=float,default=0);ap.add_argument('--gravity',type=float,default=9.81);ap.add_argument('--dt',type=float,default=.0002);ap.add_argument('--stiffness',type=float,default=2000);ap.add_argument('--damping',type=float,default=3);args=ap.parse_args()
 if not 0<=args.payload<=.2:raise SystemExit('payload must be 0..0.2 kg')
 t=E.parse(F/'world.sdf');w=t.getroot().find('world');w.set('name','delta_dynamics_world');w.find('gravity').text=f'0 0 {-args.gravity}'
 w.find('physics/max_step_size').text=str(args.dt)
 for inc in w.findall('include'):w.remove(inc)
 m=add(w,'model',name='delta_structure');add(m,'pose','0 0 0.65 0 0 0');add(m,'self_collide','false');link(m,'base_link',[0,0,0],1,'aircraft')
 j=add(m,'joint',name='fixed_body',type='fixed');add(j,'parent','world');add(j,'child','base_link')
 for i,g in enumerate(cfg['legs']):
  s=np.array(g['shoulder_m']);ee=np.array(g['elbows_home_m']);com=(ee.mean(axis=0)-s)*.5
  link(m,f'upper_{i}',s,.03,f'upper_{i}',com=com.tolist(),inertia=[.0002]*3)
  j=add(m,'joint',name=f'act_{i}',type='revolute');add(j,'parent','base_link');add(j,'child',f'upper_{i}');a=add(j,'axis');add(a,'xyz',vec(g['axis']));lim=add(a,'limit');add(lim,'lower',-.6);add(lim,'upper',.6);add(lim,'effort',2);add(lim,'velocity',1)
  for k in range(2):
   f=np.array(g['rod_frames_home'][k]);p=ee[k];name=f'rod_{i}_{k}'
   link(m,name,p,.012,name,rpy(f),com=[0,0,.1],inertia=[.00004005,.00004005,.000000096])
   j=add(m,'joint',name=f'elbow_{i}_{k}',type='ball');add(j,'parent',f'upper_{i}');add(j,'child',name)
 plugin=add(m,'plugin',filename='libdelta_dynamics.so',name='delta_dynamics::Controller')
 for k,val in [('stiffness',args.stiffness),('damping',args.damping),('payload',args.payload),('gravity',args.gravity)]:add(plugin,k,val)
 p=add(w,'model',name='delta_platform');home=np.array(cfg['home_m']);add(p,'pose',vec([*list(home+[0,0,.65]),0,0,0]));add(p,'self_collide','false')
 link(p,'platform',[0,0,0],.05,'platform',inertia=[.00008,.00008,.00012])
 if args.payload>0:
  l=link(p,'payload',[0,0,-.04],args.payload,com=[0,0,0],inertia=[max(args.payload*.00015,1e-7)]*3);v=add(l,'visual',name='payload');g=add(v,'geometry');box=add(g,'box');add(box,'size','.03 .03 .03');ma=add(v,'material');add(ma,'ambient','0.8 0.2 0.2 1');add(ma,'diffuse','0.8 0.2 0.2 1');j=add(p,'joint',name='payload_fixed',type='fixed');add(j,'parent','platform');add(j,'child','payload')
 E.indent(t);t.write(R/args.output,encoding='utf-8',xml_declaration=True)
 settings={**vars(args),'upper_mass_kg':.03,'rod_mass_kg':.012,'platform_mass_kg':.05,'constraint':'6 equal/opposite 3D spring-damper ball closures','passive_position_reset':False,'base_fixed':True,'mass_source':'explicit simulation test values; not BOM calibration'}
 (R/('settings.json' if args.output=='world.sdf' else str(Path(args.output).with_suffix('.settings.json')))).write_text(json.dumps(settings,indent=2));print(json.dumps(settings))
if __name__=='__main__':main()
