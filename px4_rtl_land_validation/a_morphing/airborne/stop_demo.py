"""Stop only articulated A test and simulation processes."""
from pathlib import Path
import subprocess,signal,os,time
R=Path(__file__).resolve().parent;C='px4-rtl-land-harmonic'
p=R/'test_pid.txt'
if p.exists():
 try:
  pid=int(p.read_text());cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
  if b'airborne/test_hover.py' in cmd:os.kill(pid,signal.SIGTERM);time.sleep(2)
 except (ProcessLookupError,FileNotFoundError):pass
for row in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines():
 a=row.strip().split(None,2)
 if len(a)==3 and a[1].startswith('python') and a[2].endswith('/airborne/ros_control.py'):
  try:os.kill(int(a[0]),signal.SIGINT)
  except ProcessLookupError:pass
for row in subprocess.check_output(['docker','exec',C,'ps','-eo','pid,args'],text=True).splitlines():
 a=row.strip().split(None,1)
 if len(a)==2 and (a[1].startswith('/work/a_morphing/airborne/px4_morph ') or a[1]=='/work/a_morphing/airborne/gz_link' or a[1].startswith('gz sim -s -r /work/a_morphing/airborne/') or a[1]=='gz sim -g --gui-config /work/a_morphing/airborne/gui.config'):
  subprocess.run(['docker','exec',C,'kill','-INT',a[0]],capture_output=True)
print('A articulated session stopped; QGC remains open')
