"""Read native PX4 matrices while the arm and morph joints are both moving."""
from pathlib import Path
import sys,time,json,re,subprocess
import numpy as np
R=Path(__file__).resolve().parent;D=R/'runs'/sys.argv[1]
def latest():
 with (D/'morph-feedback.jsonl').open('rb') as f:
  f.seek(max(0,(D/'morph-feedback.jsonl').stat().st_size-30000));lines=f.read().decode().splitlines(keepends=True)
 complete=next((v for v in reversed(lines) if v.endswith('\n')),None)
 return json.loads(complete) if complete is not None else None
for mode,index in [('H',0),('V',1)]:
 deadline=time.monotonic()+900
 while time.monotonic()<deadline:
  d=latest()
  if d is None:time.sleep(.1);continue
  angle=max(abs(x) for x in d['q_deg'][index::2])
  if d['arm_busy'] and d['transition'] and 8<angle<18:break
  if (D/'outcome.json').exists():raise SystemExit('Flight ended before live matrix capture')
  time.sleep(.1)
 else:raise TimeoutError('Simultaneous '+mode+' not observed')
 offset=(D/'px4.log').stat().st_size
 subprocess.run(['docker','exec','px4-rtl-land-harmonic','/work/PX4-Autopilot/build/px4_sitl_default/bin/px4-control_allocator','status'],check=True,capture_output=True,timeout=8)
 time.sleep(.08);raw=(D/'px4.log').read_text(errors='replace')[offset:];rows=[]
 for line in raw.splitlines():
  m=re.match(r'^\s*([0-3])\|(.+)$',line)
  if m:
   v=[float(x) for x in m.group(2).split()]
   if len(v)==6:rows.append(v)
 if len(rows)!=4:raise RuntimeError('Matrix readback missing')
 after=latest();actual=np.array(rows).T;expected=np.array(after['B_control'])*6.5;expected[np.all(np.abs(expected)<=.05,axis=1)]=0;error=float(np.max(abs(actual-expected)))
 out={'label':'coupled-'+mode,'passed':error<.02,'max_abs_error':error,'native_matrix':actual.tolist(),'expected_CT_B':expected.tolist(),'geometry_record':after,'geometry_before':d,'native_text':raw,'comparison_tolerance':.02}
 (D/f'matrix-coupled-{mode}.json').write_text(json.dumps(out,indent=2));print(mode,'live native matrix error',error,flush=True)
 if not out['passed']:raise SystemExit('Live matrix mismatch; inspect before continuing')
