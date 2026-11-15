"""Audit supplied baseline without modifying the package or inferring propulsion."""
import json,csv,hashlib,pathlib,tempfile,shutil,subprocess,sys
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parent;P=ROOT.parent/'A-UAV-20261115';params=P/'parameters';out=ROOT/'handoff_audit';out.mkdir(exist_ok=True)
j=json.loads((params/'uav_parameters.json').read_text(encoding='utf-8-sig'))
with (params/'component_mass_properties.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
m=np.array([float(r['mass_kg']) for r in rows]);x=np.array([[float(r[k]) for k in ['x_m','y_m','z_m']] for r in rows]);com=(m[:,None]*x).sum(0)/m.sum();I=np.zeros((3,3))
for mass,pos,r in zip(m,x,rows):
 a,b,c,d,e,f=map(lambda k:float(r[k]),['ixx','ixy','ixz','iyy','iyz','izz']);own=np.array([[a,b,c],[b,d,e],[c,e,f]]);delta=pos-com;I+=own+mass*((delta@delta)*np.eye(3)-np.outer(delta,delta))
np.testing.assert_allclose(m.sum(),j['mass_kg'],atol=1e-12,rtol=0);np.testing.assert_allclose(com,j['center_of_mass_m'],atol=1e-12,rtol=0);np.testing.assert_allclose(I,j['inertia_com_kgm2'],atol=1e-12,rtol=0)
R=np.array([[0,0,-1],[-1,0,0],[0,1,0.]]);assert np.allclose(R@R.T,np.eye(3)) and np.linalg.det(R)>0
rotated=R@I@R.T;co=R@com;ev=np.linalg.eigvalsh(rotated);assert ev.min()>0 and ev.max()<=ev.sum()/2+1e-10
for motor in j['motor_placements']:
 Q=np.array(motor['orientation_columns']).T;np.testing.assert_allclose(Q.T@Q,np.eye(3),atol=1e-10);assert np.linalg.det(Q)>0
original_hash={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in P.rglob('*') if f.is_file()}
# The supplied script writes mesh normals; execute solely on a disposable package copy.
with tempfile.TemporaryDirectory(prefix='handoff_recompute_') as tmp:
 target=pathlib.Path(tmp)/'A-UAV-20261115';shutil.copytree(P,target)
 py=sys.executable
 result=subprocess.run([py,str(target/'scripts/build_parameters.py')],capture_output=True,text=True,timeout=90);(out/'isolated_recompute.log').write_text(result.stdout+result.stderr);result.check_returncode()
 new=json.loads((target/'parameters/uav_parameters.json').read_text())
 for field in ['mass_kg','center_of_mass_m','inertia_com_kgm2']:np.testing.assert_allclose(new[field],j[field],atol=1e-12,rtol=0)
for file,h in original_hash.items():assert hashlib.sha256((P/file).read_bytes()).hexdigest()==h,'Input package changed'
audit={'source_directory':P.name,'source_files_sha256':original_hash,'checks':{'csv_reproduces_json_mass_com_tensor':True,'supplied_builder_reproduces_in_isolated_copy':True,'original_package_unchanged':True,'inertia_symmetric_positive_physical':True,'motor_column_orientation_matrices_are_proper_rotations':True,'source_step_matches_existing_candidate':original_hash['cad/airframe.step']==hashlib.sha256(((ROOT.parent/'现实材料'/'A无人机.STEP') if (ROOT.parent/'现实材料'/'A无人机.STEP').exists() else P/'cad/airframe.step').read_bytes()).hexdigest()},'temporary_frame':'software debug only: FLU x=-CAD z, y=-CAD x, z=CAD y; not confirmed aircraft front/up','R_flu_from_cad':R.tolist(),'mass_kg':j['mass_kg'],'com_flu_m':co.tolist(),'inertia_com_flu_kg_m2':rotated.tolist(),'unlocated_mass_kg':j['unmatched_mass_at_cad_origin_kg'],'unlocated_mass_fraction':j['unmatched_mass_at_cad_origin_kg']/j['mass_kg'],'unassigned_cad_components':j['cad_components_without_bom_mass'],'propulsion_fields_remain_unidentified':j['propulsion']}
(out/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2));print(json.dumps({k:v for k,v in audit.items() if k!='source_files_sha256'},ensure_ascii=False,indent=2))
