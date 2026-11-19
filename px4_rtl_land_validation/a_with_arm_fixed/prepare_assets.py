"""Remove the embedded mechanism mesh and export its fixed mounting assembly."""
from pathlib import Path
import sys,json,struct,hashlib,importlib.util
import numpy as np
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
R=Path(__file__).resolve().parent;V=R.parent;F=V/'delta_fixed';D=R/'models/a_with_arm_fixed/meshes';D.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(F));spec=importlib.util.spec_from_file_location('cad_export',F/'build_model.py');cad=importlib.util.module_from_spec(spec);spec.loader.exec_module(cad);cad.D=D
cfg=json.loads((F/'config.json').read_text());morph=json.loads((V/'a_morphing/config/morph-config.json').read_text());inventory=json.loads((V/'candidate_flight/cad_inventory.json').read_text())
assert inventory['sha256']==cfg['cad_sha256']==morph['source']['cad_sha256']
ids=[i for i in morph['body']['parts'] if i not in [121,122]];assert 121 in morph['body']['parts'] and 122 in morph['body']['parts'];arrays=[]
for i in ids:
 p=inventory['parts'][i];raw=(V/'candidate_flight'/p['mesh']).read_bytes();a=np.frombuffer(raw,dtype=cad.DT,count=struct.unpack_from('<I',raw,80)[0],offset=84).copy();a['v']=cad.flu(a['v']);a['n']=a['n']@cad.ROT.T;arrays.append(a)
a=np.concatenate(arrays)
with (D/'body.stl').open('wb') as f:f.write(b'A aircraft body without CAD parts 121/122'.ljust(80,b' '));f.write(struct.pack('<I',len(a)));f.write(a.tobytes())
parts=cad.load();assert '夹爪' in parts[121][0];assert '连接件' in parts[122][0];e=TopExp_Explorer(parts[121][1],TopAbs_SOLID);solids=[]
while e.More():solids.append(e.Current());e.Next()
assert len(solids)==101
fixed=[solids[i] for i in cfg['fixed_solids']]+[parts[122][1]]
compound=TopoDS_Compound();builder=BRep_Builder();builder.MakeCompound(compound)
for shape in fixed:builder.Add(compound,shape)
g=GProp_GProps();BRepGProp.VolumeProperties_s(compound,g);c=g.CentreOfMass();co=cad.flu([c.X(),c.Y(),c.Z()]);T=g.MatrixOfInertia();I=np.array([[T.Value(i+1,j+1) for j in range(3)] for i in range(3)]);I=cad.ROT@I@cad.ROT.T/g.Mass()*1e-6
# The interface is the top plane of the existing CAD connector.
bounds=inventory['parts'][122]['bbox_cad_mm'];mount=np.array([0,0,bounds[4]/1000]);mass=.1;count=cad.export(fixed,'mount',mount)
meta={'cad_sha256':cfg['cad_sha256'],'removed_from_embedded_body_parts':[121,122],'aircraft_body_parts':ids,'fixed_mechanism_solids':cfg['fixed_solids'],'connector_part':122,'mount_origin_flu_m':mount.tolist(),'mount_com_flu_m':co.tolist(),'fixed_mechanism_mass_kg':mass,'fixed_mechanism_mass_source':'explicit fixed-hardware test assumption; not measured','mount_inertia_com_flu_kg_m2':(I*mass).tolist(),'body_triangles':len(a),'mount_triangles':count,'mechanism_subdivision_total_solids':101,'original_airframe_sdf_sha256':hashlib.sha256((V/'a_morphing/airborne/models/a_air/model.sdf').read_bytes()).hexdigest()}
(R/'assets.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2));print(json.dumps(meta,ensure_ascii=False,indent=2))
