"""Read native PX4 allocator matrix at V25 and compare with measured geometry."""
import pathlib,json,time,subprocess,re
import numpy as np
R=pathlib.Path(__file__).resolve().parent
end=time.monotonic()+70
while time.monotonic()<end:
 p=max((R/'sessions').glob('*.jsonl'),key=lambda p:p.stat().st_mtime)
 with p.open('rb') as f:f.seek(max(0,p.stat().st_size-15000));lines=f.read().decode().splitlines()
 for line in reversed(lines):
  try:d=json.loads(line);break
  except json.JSONDecodeError:pass
 if min(d['q_deg'][1::2])>24.5 and max(abs(x) for x in d['q_deg'][::2])<1:
  offset=(R/'px4.log').stat().st_size
  output=subprocess.check_output(['docker','exec','px4-rtl-land-harmonic','/work/PX4-Autopilot/build/px4_sitl_default/bin/px4-control_allocator','status'],text=True)
  time.sleep(.1)
  with (R/'px4.log').open() as f:f.seek(offset);raw=f.read()
  rows=[]
  for row in raw.splitlines():
   m=re.match(r'^\s*([0-3])\|(.+)$',row)
   if m:
    vals=[float(x) for x in m.group(2).split()]
    if len(vals)==6:rows.append(vals)
  if len(rows)!=4:raise RuntimeError('Native matrix output not captured')
  native=np.array(rows).T;expected=np.array(d['B'])*6.5
  expected[np.all(abs(expected)<.05,axis=1)]=0
  err=float(np.max(abs(native-expected)))
  result={'method':'PX4 control_allocator status readback, compared with actual-joint geometry','geometry_record':d,'px4_effectiveness_6x4':native.tolist(),'expected_CT_times_B':expected.tolist(),'CT':6.5,'max_absolute_error':err,'tolerance':.02,'passed':err<.02,'native_output':output+'\n'+raw}
  (R/'results/native-matrix.json').write_text(json.dumps(result,indent=2));print('Native PX4 matrix readback:',err,err<.02);break
 time.sleep(.15)
else:raise TimeoutError('V25 not observed')
