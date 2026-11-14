"""Read REFERENCE-ONLY STEP; this is NOT the final aircraft or approved geometry."""
import hashlib,json,pathlib,struct
import numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt

ROOT=pathlib.Path(__file__).resolve().parent
source=ROOT.parent/'UAV.STEP';meshes=ROOT/'meshes';meshes.mkdir(exist_ok=True)
reader=STEPControl_Reader()
assert reader.ReadFile(str(source))==IFSelect_RetDone
reader.TransferRoots();shape=reader.OneShape();explorer=TopExp_Explorer(shape,TopAbs_SOLID)
records=[];triangles=[];solids=[]
def export_shape(s,filename):
    BRepMesh_IncrementalMesh(s,0.35,False,0.35,True).Perform()
    writer=StlAPI_Writer();writer.ASCIIMode=False
    assert writer.Write(s,str(filename))
    raw=filename.read_bytes();count=struct.unpack_from('<I',raw,80)[0]
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    arr=np.frombuffer(raw,dtype=dtype,count=count,offset=84)['vertices'].astype(float)
    flu=np.stack([-arr[:,:,2],-arr[:,:,0],arr[:,:,1]],axis=2)/1000
    with filename.open('wb') as f:
        f.write(b'Local STEP visual; provisional FLU axes; metres'.ljust(80,b' '));f.write(struct.pack('<I',count))
        for tri in flu:
            normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(normal)
            if norm:normal/=norm
            f.write(struct.pack('<12fH',*normal,*tri.ravel(),0))
    return flu
while explorer.More():
    s=TopoDS.Solid_s(explorer.Current());i=len(records)
    box=Bnd_Box();BRepBndLib.AddOptimal_s(s,box)
    properties=GProp_GProps();BRepGProp.VolumeProperties_s(s,properties)
    center=properties.CentreOfMass()
    filename=meshes/f'part_{i:02d}.stl';flu=export_shape(s,filename);count=len(flu)
    # CAD millimetres -> provisional model FLU metres. Front inferred from LF/RF labels.
    # x_forward=-CAD_Z, y_left=-CAD_X, z_up=CAD_Y; CAD origin is NOT a measured CoG.
    records.append({'part_index':i,'cad_bbox_mm':list(box.Get()),'cad_uniform_volume_centroid_mm':[center.X(),center.Y(),center.Z()],
                    'volume_mm3':properties.Mass(),'brep_valid':bool(BRepCheck_Analyzer(s).IsValid()),'triangles':count,
                    'mesh':str(filename.relative_to(ROOT)),'classification':'unassigned'})
    triangles.append(flu);solids.append(s);explorer.Next()
assert len(records)==9, len(records)
span=[np.array(r['cad_bbox_mm'][3:])-np.array(r['cad_bbox_mm'][:3]) for r in records]
body=int(np.argmax([max(s) for s in span]));records[body]['classification']='simplified_body'
for i,r in enumerate(records):
    if i!=body:r['classification']='propeller' if span[i][1]<12 else 'motor'
remainder=solids[body];partitions=[]
for name,sx,sy in [('LF',1,-1),('LR',-1,-1),('RF',1,1),('RR',-1,1)]:
    # Explicit visualization segmentation: virtual root at FRD (+/-45,+/-45,0) mm.
    # These planes are NOT recovered mechanical joint locations.
    xmin,xmax=(45,1000) if sy>0 else (-1000,-45)
    zmin,zmax=(45,1000) if sx<0 else (-1000,-45)
    cutter=BRepPrimAPI_MakeBox(gp_Pnt(xmin,-10,zmin),xmax-xmin,20,zmax-zmin).Shape()
    arm=BRepAlgoAPI_Common(solids[body],cutter).Shape();remainder=BRepAlgoAPI_Cut(remainder,cutter).Shape()
    prop=GProp_GProps();BRepGProp.VolumeProperties_s(arm,prop)
    assert prop.Mass()>0 and BRepCheck_Analyzer(arm).IsValid()
    export_shape(arm,meshes/f'arm_{name}.stl')
    partitions.append({'name':name,'volume_mm3':prop.Mass(),'virtual_hinge_frd_m':[sx*.045,sy*.045,0]})
prop=GProp_GProps();BRepGProp.VolumeProperties_s(remainder,prop)
export_shape(remainder,meshes/'body_central.stl')
volume_error=abs(sum(x['volume_mm3'] for x in partitions)+prop.Mass()-records[body]['volume_mm3'])
assert volume_error<1e-4,volume_error
result={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'reader':'OCP 7.9.3.1.1',
        'source_role':'reference_only_not_final_aircraft; user confirmed differences unknown',
        'cad_length_unit':'mm','mesh_length_unit':'m','cad_to_model_flu':[[0,0,-1],[-1,0,0],[0,1,0]],
        'axis_status':'provisional: inferred from part names, not approved frame/CoG',
        'body_index':body,'solids':records,'all_declared_solids_valid':all(r['brep_valid'] for r in records),
        'visual_partition':{'status':'assumed root planes, not actual joint extraction','arms':partitions,'volume_conservation_error_mm3':volume_error}}
(ROOT/'cad_inspection.json').write_text(json.dumps(result,indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(9,8));ax=fig.add_subplot(projection='3d')
colors={'simplified_body':'#64849b','motor':'#ef9547','propeller':'#343d45'}
for r,tri in zip(records,triangles):
    if len(tri)>18000:tri=tri[::max(1,len(tri)//18000)]
    ax.add_collection3d(Poly3DCollection(tri,facecolor=colors[r['classification']],edgecolor='none',alpha=.96))
points=np.concatenate(triangles).reshape(-1,3);low=points.min(axis=0);high=points.max(axis=0)
mid=(low+high)/2;radius=max(high-low)*.55
ax.set_xlim(mid[0]-radius,mid[0]+radius);ax.set_ylim(mid[1]-radius,mid[1]+radius);ax.set_zlim(mid[2]-radius,mid[2]+radius)
ax.set_xlabel('Provisional forward (m)');ax.set_ylabel('Provisional left (m)');ax.set_zlabel('Up (m)');ax.set_box_aspect((1,1,1));ax.view_init(24,-45)
fig.suptitle('REFERENCE STEP ONLY | NOT final aircraft | static geometry');fig.tight_layout();fig.savefig(ROOT/'cad_preview.png',dpi=150)
print(json.dumps(result,indent=2))
