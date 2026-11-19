"""Split the existing A CAD into a fixed aircraft and articulated Delta mechanism."""
from pathlib import Path
import json,hashlib,struct,math
import numpy as np
import xml.etree.ElementTree as ET
from inspect_cad import load,props
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Sphere,GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
R=Path(__file__).resolve().parent
M=R/'models/a_delta_fixed';D=M/'meshes';D.mkdir(parents=True,exist_ok=True)
DT=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
ROT=np.array([[0,0,-1],[-1,0,0],[0,1,0]],float)
def flu(p):return np.asarray(p)@ROT.T/1000

def frame(v,axis):
 z=v/np.linalg.norm(v);x=axis-z*np.dot(z,axis);x/=np.linalg.norm(x);return np.column_stack([x,np.cross(z,x),z])
def export(shapes,name,origin=np.zeros(3),rotation=np.eye(3)):
 arrays=[]
 for k,shape in enumerate(shapes):
  tmp=D/'temporary.stl';BRepMesh_IncrementalMesh(shape,.35,False,.35,True).Perform();w=StlAPI_Writer();w.ASCIIMode=False;assert w.Write(shape,str(tmp))
  raw=tmp.read_bytes();a=np.frombuffer(raw,dtype=DT,offset=84,count=struct.unpack_from('<I',raw,80)[0]).copy();a['v']=(flu(a['v'])-origin)@rotation;a['n']=a['n']@ROT.T@rotation;arrays.append(a)
 a=np.concatenate(arrays)
 with (D/(name+'.stl')).open('wb') as f:f.write(b'A CAD split mesh; units m'.ljust(80,b' '));f.write(struct.pack('<I',len(a)));f.write(a.tobytes())
 return len(a)
def add(p,t,value=None,**attrs):
 x=ET.SubElement(p,t,attrs)
 if value is not None:x.text=str(value)
 return x
def vec(v):return ' '.join(f'{x:.12g}' for x in v)
def inert(link,mass):
 a=add(link,'inertial');add(a,'mass',mass);i=add(a,'inertia')
 for n in ['ixx','iyy','izz']:add(i,n,max(mass*.001,1e-8))
def link(model,name,pos,mass=.001,mesh=None,color='0.3 0.6 0.8 1'):
 l=add(model,'link',name=name);add(l,'pose',vec([*pos,0,0,0]));inert(l,mass);add(l,'gravity','false')
 if mesh:
  v=add(l,'visual',name=name);g=add(v,'geometry');m=add(g,'mesh');add(m,'uri',f'model://a_delta_fixed/meshes/{mesh}.stl');mat=add(v,'material');add(mat,'ambient',color);add(mat,'diffuse',color)
 return l
def joint(model,name,parent,child,axis,type='revolute'):
 j=add(model,'joint',name=name,type=type);add(j,'parent',parent);add(j,'child',child);a=add(j,'axis');add(a,'xyz',vec(axis));lim=add(a,'limit');add(lim,'lower',-3.14 if type=='revolute' else -.3);add(lim,'upper',3.14 if type=='revolute' else .3);add(lim,'effort',2);return j

def main():
 parts=load();solids=[];ex=TopExp_Explorer(parts[121][1],TopAbs_SOLID)
 while ex.More():solids.append(ex.Current());ex.Next()
 assert len(solids)==101
 spheres={};shoulders={};axes={}
 for i,s in enumerate(solids):
  face=TopExp_Explorer(s,TopAbs_FACE);sp=[];sc=[]
  while face.More():
   f=TopoDS.Face_s(face.Current());a=BRepAdaptor_Surface(f)
   if a.GetType()==GeomAbs_Sphere and abs(a.Sphere().Radius()-4)<1e-5:
    p=a.Sphere().Location();v=np.array([p.X(),p.Y(),p.Z()]);
    if not any(np.linalg.norm(v-b)<1e-5 for b in sp):sp.append(v)
   if i<3 and a.GetType()==GeomAbs_Cylinder and abs(a.Cylinder().Radius()-4)<1e-5:
    g=GProp_GProps();BRepGProp.SurfaceProperties_s(f,g);p=g.CentreOfMass();sc.append([p.X(),p.Y(),p.Z()]);d=a.Cylinder().Axis().Direction();axes[i]=ROT@np.array([d.X(),d.Y(),d.Z()])
   face.Next()
  spheres[i]=sp
  if sc:shoulders[i]=flu(np.mean(sc,axis=0))
 bottom_ids=[[75,76],[73,74],[77,78]]
 upper_ids=[[0,61,62,63,64],[1,69,70,71,72],[2,65,66,67,68]]
 rod_ids=[[list(range(12,21)),list(range(33,42))],[list(range(3,12)),list(range(42,51))],[list(range(51,60)),list(range(21,30))]]
 platform_ids=[60,*range(73,81),100];fixed_ids=[30,31,32,*range(81,100)]
 all_ids=[*platform_ids,*fixed_ids,*[n for ids in upper_ids for n in ids],*[n for pair in rod_ids for ids in pair for n in ids]];assert sorted(all_ids)==list(range(101))
 bottoms=np.array([[flu(spheres[b][0]) for b in ids] for ids in bottom_ids]);P=np.mean(bottoms.reshape(-1,3),axis=0)
 cfg={'source':'A-UAV-20261115/cad/airframe.step','cad_sha256':hashlib.sha256((R.parent/'A-UAV-20261115/cad/airframe.step').read_bytes()).hexdigest(),'R_flu_from_cad':ROT.tolist(),'units':'m, rad, s','mode':'actual actuated joints; reduced-coordinate kinematic closure of passive links; fixed base','home_m':P.tolist(),'platform_solids':platform_ids,'fixed_solids':fixed_ids,'legs':[]}
 for i in range(3):
  E=np.array([flu(x) for x in spheres[i]]);B=bottoms[i];S=shoulders[i];axis=axes[i]
  cfg['legs'].append({'shoulder_m':S.tolist(),'axis':axis.tolist(),'elbows_home_m':E.tolist(),'platform_offsets_m':(B-P).tolist(),'lower_length_m':float(np.mean(np.linalg.norm(E-B,axis=1))),'upper_solids':upper_ids[i],'rod_solids':rod_ids[i]})
 counts={};counts['aircraft']=export([s for i,(_,s) in enumerate(parts) if i!=121]+[solids[i] for i in fixed_ids],'aircraft')
 counts['platform']=export([solids[k] for k in platform_ids],'platform',P)
 for i,g in enumerate(cfg['legs']):
  S=np.array(g['shoulder_m']);counts[f'upper_{i}']=export([solids[k] for k in upper_ids[i]],f'upper_{i}',S)
  for k in range(2):
   E=np.array(g['elbows_home_m'][k]);B=P+np.array(g['platform_offsets_m'][k]);F=frame(B-E,np.array(g['axis']));g.setdefault('rod_frames_home',[]).append(F.tolist());counts[f'rod_{i}_{k}']=export([solids[j] for j in rod_ids[i][k]],f'rod_{i}_{k}',E,F)
 cfg['mesh_triangles']=counts;(R/'config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
 sdf=ET.Element('sdf',version='1.9');model=add(sdf,'model',name='a_delta_fixed');add(model,'pose','0 0 0.65 0 0 0');add(model,'self_collide','false')
 link(model,'base_link',[0,0,0],1,mesh='aircraft',color='0.38 0.48 0.60 1');j=add(model,'joint',name='base_fixed',type='fixed');add(j,'parent','world');add(j,'child','base_link')
 for i,g in enumerate(cfg['legs']):
  S=np.array(g['shoulder_m']);axis=np.array(g['axis']);l=link(model,f'upper_{i}',S,.03,f'upper_{i}','0.95 0.45 0.12 1');
  for nn in ['ixx','iyy','izz']:l.find('inertial/inertia/'+nn).text='.0005'
  j=joint(model,f'act_{i}','base_link',f'upper_{i}',axis)
  j.find('axis/limit/lower').text='-.6';j.find('axis/limit/upper').text='.6'
  for k in range(2):
   E=np.array(g['elbows_home_m'][k]);F=np.array(g['rod_frames_home'][k]);
   # Mesh uses the home rod frame; visual pose restores CAD orientation at q=0.
   parent=f'upper_{i}'
   for n,ax in enumerate(np.eye(3)):
    name=f'rod_{i}_{k}_{n}';child=f'rod_{i}_{k}' if n==2 else name+'_frame';l=link(model,child,E,1e-5 if n<2 else .001, f'rod_{i}_{k}' if n==2 else None,'0.24 0.68 0.48 1');joint(model,name,parent,child,ax);parent=child
   # joint reset supplies full world rod orientation, so no initial visual rotation here.
 parent='base_link'
 for n,ax in enumerate(np.eye(3)):
  name=f'platform_{n}';child='platform' if n==2 else name+'_frame';link(model,child,P,1e-5 if n<2 else .02,'platform' if n==2 else None,'0.80 0.20 0.25 1');joint(model,name,parent,child,ax,'prismatic');parent=child
 plugin=add(model,'plugin',filename='libdelta_fixed.so',name='delta_fixed::Controller')
 ET.indent(sdf);ET.ElementTree(sdf).write(M/'model.sdf',encoding='utf-8',xml_declaration=True)
 (M/'model.config').write_text('<model><name>A fixed Delta</name><version>1</version><sdf version="1.9">model.sdf</sdf><description>A CAD articulated fixed-base Delta bench</description></model>')
 # Header shared by the native Gazebo follower and host ROS controller.
 def arr(v):return '{'+','.join(format(float(x),'.17g') for x in np.asarray(v).reshape(-1))+'}'
 h='#pragma once\nnamespace cad {\n'
 for name,vals in [('S',[g['shoulder_m'] for g in cfg['legs']]),('A',[g['axis'] for g in cfg['legs']]),('E',[[*np.mean(g['elbows_home_m'],axis=0)] for g in cfg['legs']]),('O',[[*np.mean(g['platform_offsets_m'],axis=0)] for g in cfg['legs']])]:h+=f'const double {name}[3][3]={{'+','.join(arr(v) for v in vals)+'};\n'
 h+='const double P[3]='+arr(P)+';\n'
 h+='const double EP[3][2][3]={'+','.join('{'+','.join(arr(v) for v in g['elbows_home_m'])+'}' for g in cfg['legs'])+'};\n'
 h+='const double OP[3][2][3]={'+','.join('{'+','.join(arr(v) for v in g['platform_offsets_m'])+'}' for g in cfg['legs'])+'};\n}\n';(R/'cad_geometry.hh').write_text(h)
 print(json.dumps({'solids':len(solids),'partition_complete':True,'home_m':P.tolist(),'triangles':counts},indent=2))
if __name__=='__main__':main()
