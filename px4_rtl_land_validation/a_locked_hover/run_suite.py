"""Three empty flights first; loaded comparison only after all empty flights pass."""
from pathlib import Path
import subprocess,sys,json
R=Path(__file__).resolve().parent;PY=R.parent/'.venv/bin/python'
for payload,prefix in [(0,'empty'),(.05,'load50g')]:
 for trial in range(1,4):
  name=f'{prefix}_{trial:02d}';D=R/'runs'/name
  if (D/'summary.json').exists():
   if not json.loads((D/'summary.json').read_text())['passed']:raise SystemExit('Previous failed trial requires diagnosis: '+name)
   print('Retained passed trial',name,flush=True);continue
  if D.exists():
   prior=json.loads((D/'outcome.json').read_text()) if (D/'outcome.json').exists() else {}
   if prior.get('error')=='PX4 heartbeat absent' and not prior.get('events'):
    name+='_retry1';D=R/'runs'/name
    if D.exists():raise SystemExit('Existing retry requires inspection: '+name)
   else:raise SystemExit('Existing incomplete trial requires diagnosis: '+name)
  if (R/'active.json').exists():subprocess.run([sys.executable,str(R/'stop.py')],check=True)
  subprocess.run([sys.executable,str(R/'launch.py'),'--name',name,'--payload',str(payload)],check=True)
  try:
   with (D/'test.log').open('w') as log:subprocess.run([str(PY),str(R/'test_flight.py'),name],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=260)
   with (D/'analysis.log').open('w') as log:subprocess.run([str(PY),str(R/'analyze.py'),name],stdout=log,stderr=subprocess.STDOUT,check=True)
   print('PASSED',name,flush=True)
  except Exception:
   print('STOPPED: inspect actual logs for',name,flush=True);raise
  finally:subprocess.run([sys.executable,str(R/'stop.py')],check=False)
