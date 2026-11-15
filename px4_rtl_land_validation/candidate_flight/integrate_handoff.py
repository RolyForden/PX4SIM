"""Adapt BOM-weighted source tensor consistently; never infer propulsion from KV."""
import json,pathlib,hashlib
import numpy as np

def parallel(v):
 v=np.asarray(v,float);return (v@v)*np.eye(3)-np.outer(v,v)

def load_baseline(root,settings,R):
 file=settings.get('mass_properties_json')
 if not file:return None
 source=(root/file).resolve();raw=json.loads(source.read_text(encoding='utf-8-sig'))
 assert raw['status']=='PROVISIONAL_NOT_MEASURED'
 assert raw['units']=={'length':'m','mass':'kg','inertia':'kg*m^2'}
 M=float(raw['mass_kg']);com=R@np.array(raw['center_of_mass_m']);I=R@np.array(raw['inertia_com_kgm2'])@R.T
 np.testing.assert_allclose(I,I.T,atol=1e-12);ev=np.linalg.eigvalsh(I);assert ev.min()>0 and ev.max()<=ev.sum()/2+1e-12
 return {'source':str(source.relative_to(root.parent)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'raw':raw,'mass_kg':M,'com_flu_m':com,'inertia_com_flu_kg_m2':I,'R':R}

def matched_motor(baseline,sx,sy):
 R=baseline['R'];matches=[]
 for motor in baseline['raw']['motor_placements']:
  pos=R@np.array(motor['placement_origin_m'])
  if np.sign(pos[0])==sx and np.sign(pos[1])==sy:matches.append((motor,pos))
 assert len(matches)==1,'Motor/frame association is not unique'
 motor,pos=matches[0];Q=np.array(motor['orientation_columns']).T
 np.testing.assert_allclose(Q.T@Q,np.eye(3),atol=1e-10);assert np.linalg.det(Q)>0
 return motor,pos,R@Q

def physical_carrier_decomposition(baseline,positions,carrier_mass=.009):
 """Whole tensor includes props. Split off four prop actuator carriers exactly.

 The carrier mass comes from BOM prop mass; carrier spin inertia/force-point
 placement remain simplified, but whole rigid mass/CoG/tensor are preserved.
 """
 M=baseline['mass_kg'];com=baseline['com_flu_m'];I=baseline['inertia_com_flu_kg_m2'];positions=np.array(positions)
 carrierI=np.diag([1e-5,1e-5,2e-5]);baseM=M-len(positions)*carrier_mass
 basec=(M*com-carrier_mass*positions.sum(0))/baseM
 baseI=I+M*parallel(com)-sum((carrierI+carrier_mass*parallel(p) for p in positions),start=np.zeros((3,3)))-baseM*parallel(basec)
 eig=np.linalg.eigvalsh(baseI);assert eig.min()>0 and eig.max()<=eig.sum()/2+1e-12
 # Independent reconstruction at the WHOLE-assembly CoG.
 recovered=baseI+baseM*parallel(basec-com)+sum((carrierI+carrier_mass*parallel(p-com) for p in positions),start=np.zeros((3,3)))
 np.testing.assert_allclose(recovered,I,atol=1e-12,rtol=0)
 np.testing.assert_allclose((baseM*basec+carrier_mass*positions.sum(0))/M,com,atol=1e-12,rtol=0)
 return baseM,basec,baseI,carrier_mass,carrierI
