from pathlib import Path
import json,os,signal,subprocess
R=Path(__file__).resolve().parent;p=R/'pids.json'
if p.exists():
 for pid in json.loads(p.read_text()).values():
  try:
   if b'a_with_arm_fixed' in Path(f'/proc/{pid}/cmdline').read_bytes():os.killpg(pid,signal.SIGTERM)
  except (ProcessLookupError,FileNotFoundError):pass
 p.unlink()
subprocess.run(['docker','exec','px4-rtl-land-harmonic','pkill','-f','^gz sim.*(/work/a_with_arm_fixed/world.sdf|/work/a_with_arm_fixed/gui.config)'],check=False)
