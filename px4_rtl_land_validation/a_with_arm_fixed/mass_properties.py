"""Signed tensors, frame rotations and parallel-axis assembly in SI units."""
import math,xml.etree.ElementTree as E
import numpy as np

def parallel(p):
 p=np.asarray(p,float);return np.dot(p,p)*np.eye(3)-np.outer(p,p)
def rotation(rpy):
 r,p,y=rpy;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
 return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def pose(node):return np.array(list(map(float,(node.text if node is not None else '0 0 0 0 0 0').split())))
def tensor(node):
 val=lambda k:float(node.findtext(k,'0'))
 return np.array([[val('ixx'),val('ixy'),val('ixz')],[val('ixy'),val('iyy'),val('iyz')],[val('ixz'),val('iyz'),val('izz')]])
def set_inertia(link,m,c,I):
 a=link.find('inertial')
 if a is not None:link.remove(a)
 a=E.SubElement(link,'inertial');E.SubElement(a,'mass').text=format(float(m),'.17g');E.SubElement(a,'pose').text=' '.join(format(float(v),'.17g') for v in [*c,0,0,0]);node=E.SubElement(a,'inertia')
 for k,v in [('ixx',I[0,0]),('iyy',I[1,1]),('izz',I[2,2]),('ixy',I[0,1]),('ixz',I[0,2]),('iyz',I[1,2])]:E.SubElement(node,k).text=format(float(v),'.17g')
def records(model,include_model_pose=False):
 base=pose(model.find('pose')) if include_model_pose else np.zeros(6);R=rotation(base[3:]);result=[]
 for l in model.findall('link'):
  p=pose(l.find('pose'));a=l.find('inertial');ip=pose(a.find('pose'));Q=R@rotation(p[3:]);QI=Q@rotation(ip[3:]);c=base[:3]+R@p[:3]+Q@ip[:3];I=QI@tensor(a.find('inertia'))@QI.T
  result.append({'name':l.get('name'),'mass':float(a.findtext('mass')),'com':c,'I':I})
 return result
def aggregate(rows):
 M=sum(x['mass'] for x in rows);c=sum(x['mass']*x['com'] for x in rows)/M;I=sum(x['I']+x['mass']*parallel(x['com']-c) for x in rows)
 return {'mass_kg':M,'com_m':c.tolist(),'inertia_com_kg_m2':I.tolist()}
def valid_tensor(I):
 ev=np.linalg.eigvalsh(I);return bool(ev.min()>0 and ev.max()<ev.sum()/2+1e-12)
