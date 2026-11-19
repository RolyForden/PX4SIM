"""Check the home arm configuration over X/H/V 0..25 degrees before flight."""
from pathlib import Path
import sys,json,copy,math,xml.etree.ElementTree as E
import numpy as np
R=Path(__file__).resolve().parent;V=R.parent;sys.path.insert(0,str(V/'a_with_arm_fixed'))
from mass_properties import records,aggregate
from kinematics import rotation
t=E.parse(R/'world.sdf');air=t.find("world/model[@name='a_coupled']");platform=t.find("world/model[@name='integrated_platform']");rows=records(air,True)+records(platform,True)
cfg=json.loads((V/'a_morphing/config/morph-config.json').read_text());out=[]
for branch in ['H','V']:
 for deg in [0,5,10,15,20,25]:
  links=copy.deepcopy(rows);r=[];axes=[]
  for i,a in enumerate(cfg['arms']):
   h=np.array(a['hinge_flu_m'])+[0,0,.35]
   angle=math.radians(deg)
   T=rotation(np.array([0,0,1]),math.copysign(angle,a['H']['yaw_rad'])) if branch=='H' else rotation(np.array(a['pitch_axis']),angle)
   for l in links:
    if l['name'] in [f'arm_{i}',f'rotor_{i}']:
     l['com']=h+T@(l['com']-h);l['I']=T@l['I']@T.T
   rp=h+T@(np.array(cfg['rotors'][i]['position_flu_m'])+[0,0,.35]-h);r.append(rp);axes.append(T@np.array([0,0,1]))
  total=aggregate(links);frd=np.array([1,-1,-1]);r=(np.array(r)-total['com_m'])*frd;axis=np.array(axes)*frd;B=np.vstack([(np.cross(r,axis)-np.array([.016,.016,-.016,-.016])[:,None]*axis).T,axis.T]);A=B[[0,1,2,5]];f=np.linalg.solve(A,[0,0,0,-total['mass_kg']*9.81]);rank=int(np.linalg.matrix_rank(A));assert rank==4 and min(f)>0 and max(f)<8.54858
  out.append({'branch':branch,'degrees':deg,'rank':rank,'hover_thrust_n':f.tolist(),'minimum_margin_n':float(min(8.54858-f)),'horizontal_residual_force_n':(B@f)[3:5].tolist(),'com_world_m':total['com_m']})
(R/'hover-envelope.json').write_text(json.dumps(out,indent=2));print('12 geometry checks passed; minimum per-rotor thrust margin:',min(v['minimum_margin_n'] for v in out))
