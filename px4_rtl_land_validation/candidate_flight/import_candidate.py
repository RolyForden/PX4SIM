"""Import version-unconfirmed candidate CAD. Geometric centres are NOT real CoG."""
import json,pathlib,hashlib,sys
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
ROOT=pathlib.Path(__file__).resolve().parent
source=ROOT.parent/'现实材料'/'A无人机.STEP'
if not source.exists():source=ROOT.parent/'A-UAV-20261115'/'cad'/'airframe.step'
if not source.exists():raise FileNotFoundError('请先下载并解压 A-UAV-20261115.zip 至 px4_rtl_land_validation/')
doc=TDocStd_Document(TCollection_ExtendedString('candidate'));reader=STEPCAFControl_Reader();reader.SetNameMode(True)
assert int(reader.ReadFile(str(source)))==1
assert reader.Transfer(doc)
tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());roots=TDF_LabelSequence();tool.GetFreeShapes(roots)
records=[]
def name(l):
 a=TDataStd_Name()
 return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(),a) else ''
def walk(label,placement,path):
 local=tool.GetLocation_s(label);placed=placement*local
 target=TDF_Label()
 if tool.IsReference_s(label):
  assert tool.GetReferredShape_s(label,target)
 else:target=label
 title=name(target);path=path+[name(label) or title]
 children=TDF_LabelSequence()
 if tool.GetComponents_s(target,children):
  for i in range(1,children.Length()+1):walk(children.Value(i),placed,path)
  return
 shape=tool.GetShape_s(target).Moved(placed)
 b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b);prop=GProp_GProps();BRepGProp.VolumeProperties_s(shape,prop);c=prop.CentreOfMass()
 records.append({'index':len(records),'name':title,'path':path,'bbox_cad_mm':list(b.Get()),'uniform_volume_centroid_cad_mm':[c.X(),c.Y(),c.Z()],'volume_mm3':prop.Mass()})
 shapes.append(shape)
shapes=[]
for i in range(1,roots.Length()+1):walk(roots.Value(i),TopLoc_Location(),[])
out={'source':source.name,'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'version':'unconfirmed candidate; not certified final','units':'CAD mm','parts':records}
(ROOT/'cad_inventory.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
for r in records:print(r['index'],r['name'],np.round(r['uniform_volume_centroid_cad_mm'],2),np.round(np.array(r['bbox_cad_mm'][3:])-r['bbox_cad_mm'][:3],2),flush=True)
if '--export' in sys.argv:
 import struct
 meshes=ROOT/'meshes';meshes.mkdir(exist_ok=True)
 for shape,r in zip(shapes,records):
  BRepMesh_IncrementalMesh(shape,.6,False,.4,True).Perform();w=StlAPI_Writer();w.ASCIIMode=False;f=meshes/f"part_{r['index']:03d}.stl";assert w.Write(shape,str(f));r['mesh']=str(f.relative_to(ROOT))
 (ROOT/'cad_inventory.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
