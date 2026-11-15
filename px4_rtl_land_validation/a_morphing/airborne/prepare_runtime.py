"""Prepare the current model and dedicated PX4 runtime from released assets."""
from pathlib import Path
import shutil,xml.etree.ElementTree as E
R=Path(__file__).resolve().parent;PX=R.parents[1]/'PX4-Autopilot'
model=R/'models/a_air/model.sdf'
for uri in E.parse(model).findall('.//mesh/uri'):
 path=R/'models'/uri.text.removeprefix('model://')
 if not path.exists():raise SystemExit('Missing model mesh: '+str(path)+'; extract a-air-runtime-assets.zip here first')
etc=R/'runtime/etc'
if not etc.exists():shutil.copytree(PX/'build/px4_sitl_default/etc',etc)
shutil.copy2(R/'4996_a_air',etc/'init.d-posix/airframes/4996_a_air')
print('A articulated model and airframe4996 runtime ready')
