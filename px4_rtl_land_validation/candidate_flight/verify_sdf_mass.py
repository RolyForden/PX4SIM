"""Independently reconstruct aggregate mass properties from generated fixed SDF."""
import pathlib,json,xml.etree.ElementTree as E
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parent

def verify():
 config=json.loads((ROOT/'flight_config.json').read_text());sdf=E.parse(ROOT/'models/candidate_rigid/model.sdf');parts=[]
 for link in sdf.findall('model/link'):
  pose=np.array([float(x) for x in link.findtext('pose','0 0 0 0 0 0').split()]);i=link.find('inertial');ip=np.array([float(x) for x in i.findtext('pose','0 0 0 0 0 0').split()])
  assert np.allclose(pose[3:],0) and np.allclose(ip[3:],0),'This audit requires the current unrotated fixed-link frames'
  m=float(i.findtext('mass'));tensor=i.find('inertia');g=lambda k:float(tensor.findtext(k,'0'));I=np.array([[g('ixx'),g('ixy'),g('ixz')],[g('ixy'),g('iyy'),g('iyz')],[g('ixz'),g('iyz'),g('izz')]])
  eig=np.linalg.eigvalsh(I);assert m>0 and eig.min()>0 and eig.max()<=eig.sum()/2+1e-12
  parts.append((m,pose[:3]+ip[:3],I))
 M=sum(m for m,c,I in parts);c=sum(m*c for m,c,I in parts)/M;net=np.zeros((3,3))
 for m,pos,I in parts:
  d=pos-c;net+=I+m*((d@d)*np.eye(3)-np.outer(d,d))
 np.testing.assert_allclose(M,config['total_mass_kg'],atol=1e-12,rtol=0);np.testing.assert_allclose(c,config['center_of_mass_flu_m'],atol=1e-12,rtol=0);np.testing.assert_allclose(net,config['inertia_com_flu_kg_m2'],atol=1e-12,rtol=0)
 for rotor in config['rotors']:np.testing.assert_allclose((np.array(rotor['position_flu_m'])-c)*[1,-1,-1],rotor['position_frd_m'],atol=1e-12,rtol=0)
 result={'source':'BOM-weighted provisional handoff','model_mass_kg':M,'model_com_flu_m':c.tolist(),'model_inertia_com_flu_kg_m2':net.tolist(),'checks':{'aggregate_sdf_mass_matches_source':True,'aggregate_sdf_com_matches_rotated_source':True,'aggregate_sdf_full_signed_tensor_matches_rotated_source':True,'all_link_inertias_positive_physical':True,'px4_lever_arms_relative_to_whole_com':True},'prop_partition':'four 9g simplified actuator carriers SUBTRACTED from base; not added on top of source whole mass'}
 out=ROOT/'handoff_audit';out.mkdir(exist_ok=True);(out/'sdf_mass_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':verify()
