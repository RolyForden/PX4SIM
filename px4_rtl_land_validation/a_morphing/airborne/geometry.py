"""A articulated model geometry and physical thrust-effectiveness matrix."""
from pathlib import Path
import json,math
import numpy as np
R=Path(__file__).resolve().parent
CONFIG=json.loads((R.parent/'config/morph-config.json').read_text())
def rot(axis,angle):
 a=np.array(axis,dtype=float);a/=np.linalg.norm(a);x,y,z=a;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
 return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)
def geometry(q):
 masses=[CONFIG['body']['mass_kg']-.004];positions=[np.array(CONFIG['body']['com_flu_m'])]
 rp=[];axes=[]
 for i,a in enumerate(CONFIG['arms']):
  h=np.array(a['hinge_flu_m']);T=rot([0,0,1],q[2*i])@rot(a['pitch_axis'],q[2*i+1])
  masses.extend([.001,a['mass_kg'],CONFIG['rotor_mass_kg']]);p=h+T@(np.array(CONFIG['rotors'][i]['position_flu_m'])-h)
  positions.extend([h,h+T@(np.array(a['com_flu_m'])-h),p]);rp.append(p);axes.append(T@np.array([0,0,1]))
 c=np.average(positions,axis=0,weights=masses);sgn=np.array([1,-1,-1]);r=(np.array(rp)-c)*sgn;d=np.array(axes)*sgn
 B=np.vstack([(np.cross(r,d)-np.array([x['km_m'] for x in CONFIG['rotors']])[:,None]*d).T,d.T])
 return c,r,d,B
