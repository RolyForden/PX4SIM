"""Reproduce the three fixed-base gravity/load cases; no flight software."""
from pathlib import Path
import subprocess,sys,datetime
R=Path(__file__).resolve().parent
suffix=datetime.datetime.now().strftime('_%Y%m%d_%H%M%S')
cases=[('zero_g',0,0),('gravity_empty',0,9.81),('gravity_payload_50g',.05,9.81)]
for name,payload,gravity in cases:
 name+=suffix
 subprocess.run([sys.executable,str(R/'stop.py')],check=True)
 try:
  subprocess.run([sys.executable,str(R/'run.py'),'--payload',str(payload),'--gravity',str(gravity)],check=True)
  subprocess.run(['bash','-c','source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "$1" --name "$2"','bash',str(R/'verify.py'),name],check=True)
 finally:subprocess.run([sys.executable,str(R/'stop.py')],check=False)
subprocess.run([str(R.parent/'.venv/bin/python'),str(R/'analyze.py'),'--suffix',suffix],check=True)
