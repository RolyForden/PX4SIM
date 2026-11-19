"""Finish the active empty retest, gate the 50 g test on native-log acceptance."""
from pathlib import Path
import subprocess,time,json,sys
R=Path(__file__).resolve().parent;PY=R.parent/'.venv/bin/python'
def finish(name):
 d=R/'runs'/name
 subprocess.run([sys.executable,str(R/'stop.py')],check=True)
 subprocess.run([sys.executable,str(R/'finalize_log.py'),name],check=True)
 with (d/'analysis.log').open('w') as f:subprocess.run([str(PY),str(R/'analyze.py'),name],stdout=f,stderr=subprocess.STDOUT,check=True)
 print('PASSED',name,flush=True)
empty=sys.argv[1] if len(sys.argv)>1 else 'empty_03'
deadline=time.monotonic()+1200
while not (R/'runs'/empty/'outcome.json').exists():
 if time.monotonic()>deadline:raise TimeoutError('Active empty flight not finished; inspect it before stopping')
 time.sleep(.5)
finish(empty)
name='load50g_01';subprocess.run([sys.executable,str(R/'launch.py'),'--name',name,'--payload','.05'],check=True)
d=R/'runs'/name
while not (d/'morph-feedback.jsonl').exists():time.sleep(.1)
with (d/'matrix-live.log').open('w') as log:
 capture=subprocess.Popen([str(PY),str(R/'capture_live.py'),name],stdout=log,stderr=subprocess.STDOUT)
 try:
  with (d/'test.log').open('w') as f:subprocess.run([str(PY),str(R/'test_flight.py'),name],stdout=f,stderr=subprocess.STDOUT,timeout=1200,check=True)
  capture.wait(timeout=10)
  if capture.returncode!=0:raise RuntimeError('Native matrix capture worker failed; inspect its log')
 finally:
  if capture.poll() is None:capture.terminate()
  if (R/'active.json').exists():subprocess.run([sys.executable,str(R/'stop.py')],check=True)
  if (d/'outcome.json').exists():subprocess.run([sys.executable,str(R/'finalize_log.py'),name],check=True)
with (d/'analysis.log').open('w') as f:subprocess.run([str(PY),str(R/'analyze.py'),name],stdout=f,stderr=subprocess.STDOUT,check=True)
print('PASSED',name,flush=True)
