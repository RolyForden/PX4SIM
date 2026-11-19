from pathlib import Path
import json
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
R=Path(__file__).resolve().parent

def load():
 doc=TDocStd_Document(TCollection_ExtendedString('delta_fixed'))
 reader=STEPCAFControl_Reader();reader.SetNameMode(True)
 assert int(reader.ReadFile(str(R.parent/'A-UAV-20261115/cad/airframe.step')))==1
 assert reader.Transfer(doc)
 tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());roots=TDF_LabelSequence();tool.GetFreeShapes(roots);parts=[]
 def walk(l,loc):
  placed=loc*tool.GetLocation_s(l);target=TDF_Label()
  if tool.IsReference_s(l):tool.GetReferredShape_s(l,target)
  else:target=l
  ns=TDataStd_Name();name=ns.Get().ToExtString() if target.FindAttribute(TDataStd_Name.GetID_s(),ns) else ''
  ch=TDF_LabelSequence()
  if tool.GetComponents_s(target,ch):
   for k in range(1,ch.Length()+1):walk(ch.Value(k),placed)
  else:parts.append((name,tool.GetShape_s(target).Moved(placed)))
 for k in range(1,roots.Length()+1):walk(roots.Value(k),TopLoc_Location())
 return parts

def props(s):
 b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b);g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);c=g.CentreOfMass()
 return {'bbox_mm':list(b.Get()),'center_mm':[c.X(),c.Y(),c.Z()],'volume_mm3':g.Mass()}
if __name__=='__main__':
 parts=load();records=[]
 for index,(name,shape) in enumerate(parts):
  if '夹爪' not in name:continue
  ex=TopExp_Explorer(shape,TopAbs_SOLID);k=0
  while ex.More():
   rec={'part_index':index,'part_name':name,'solid':k,**props(ex.Current())};records.append(rec);print(json.dumps(rec,ensure_ascii=False),flush=True);k+=1;ex.Next()
 (R/'cad-solids.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
