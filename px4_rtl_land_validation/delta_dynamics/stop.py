from pathlib import Path
import os,signal,json,subprocess
R=Path(__file__).resolve().parent;p=R/'pids.json'
if p.exists():
 for pid in json.loads(p.read_text()).values():
  try:
   if b'delta_dynamics' in Path(f'/proc/{pid}/cmdline').read_bytes():os.killpg(pid,signal.SIGTERM)
  except ProcessLookupError:pass
  except FileNotFoundError:pass
 p.unlink()
subprocess.run(['docker','exec','px4-rtl-land-harmonic','pkill','-f','^gz sim.*(/work/delta_dynamics/world.sdf|/work/delta_dynamics/gui.config)'],check=False)
print('受力台架已停止')
