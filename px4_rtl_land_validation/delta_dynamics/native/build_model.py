"""Native upper/lower ball joints with tiny fixed-weld proxy bodies."""
from pathlib import Path
import subprocess,sys,argparse,json
import xml.etree.ElementTree as E
import numpy as np
import math
R=Path(__file__).resolve().parent;B=R.parent
ap=argparse.ArgumentParser();ap.add_argument('--payload',type=float,default=.05);ap.add_argument('--gravity',type=float,default=9.81);ap.add_argument('--base-rpy',nargs=3,type=float,default=[0,0,0]);args=ap.parse_args()
subprocess.run([sys.executable,str(B/'build_model.py'),'--output','native/world.sdf','--payload',str(args.payload),'--gravity',str(args.gravity),'--stiffness','0','--damping','0'],check=True,stdout=subprocess.DEVNULL)
t=E.parse(R/'world.sdf');root=t.getroot();m=root.find("world/model[@name='delta_structure']");m.set('name','native_structure');root.find("world/model[@name='delta_platform']").set('name','native_platform');pl=m.find("plugin[@name='delta_dynamics::Controller']");pl.set('filename','libdelta_native.so');pl.set('name','delta_native::Controller');pl.remove(pl.find('stiffness'));pl.remove(pl.find('damping'))
cfg=json.loads((R/'config.json').read_text())
for i,leg in enumerate(cfg['legs']):
 for k in range(2):
  p=[cfg['home_m'][n]+leg['platform_offsets_m'][k][n] for n in range(3)];name=f'tip_{i}_{k}';l=E.SubElement(m,'link',name=name);E.SubElement(l,'pose').text=' '.join(map(str,[*p,0,0,0]));E.SubElement(l,'gravity').text='true';inert=E.SubElement(l,'inertial');E.SubElement(inert,'mass').text='0.00001';ii=E.SubElement(inert,'inertia')
  for a in ['ixx','iyy','izz']:E.SubElement(ii,a).text='1e-8'
  j=E.SubElement(m,'joint',name=f'end_{i}_{k}',type='ball');E.SubElement(j,'parent').text=f'rod_{i}_{k}';E.SubElement(j,'child').text=name
logger=E.SubElement(m,'plugin',filename='libwrench_logger.so',name='study::WrenchLogger');E.SubElement(logger,'joint').text='fixed_body';E.SubElement(logger,'output').text='/work/delta_dynamics/native/results/mount_wrench.jsonl'
roll,pitch,yaw=args.base_rpy;cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
rot=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
m.find('pose').text=' '.join(map(str,[0,0,.65,*args.base_rpy]));platform=root.find("world/model[@name='native_platform']");platform.find('pose').text=' '.join(map(str,[*(rot@np.array(cfg['home_m'])+np.array([0,0,.65])),*args.base_rpy]))
E.indent(t);t.write(R/'world.sdf',encoding='utf-8',xml_declaration=True)
s=json.loads((R/'world.settings.json').read_text());s.update({'constraint':'native 12 ball joints + 6 runtime fixed loop welds','proxy_mass_each_kg':.00001,'base_rpy_rad':args.base_rpy,'proxy_total_mass_kg':.00006,'stiffness':None,'damping':None});(R/'settings.json').write_text(json.dumps(s,indent=2))
