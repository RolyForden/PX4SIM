from pathlib import Path
import json,os,signal,subprocess
R=Path(__file__).resolve().parent
p=R/'pids.json'
if p.exists():
 for pid in json.loads(p.read_text()).values():
  try:os.killpg(pid,signal.SIGTERM)
  except ProcessLookupError:pass
 # docker exec signals do not always propagate to the gz server.
 subprocess.run(['docker','exec','px4-rtl-land-harmonic','pkill','-f','^gz sim.*(/work/delta_fixed/world.sdf|/work/delta_fixed/gui.config)'],check=False)
 p.unlink();print('固定台架已停止')
