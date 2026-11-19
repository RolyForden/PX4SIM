from pathlib import Path
import subprocess,json,os,signal,time
R=Path(__file__).resolve().parent;C='px4-rtl-land-harmonic';P='/work/PX4-Autopilot/build/px4_sitl_default/bin'
p=R/'active.json'
if not p.exists():raise SystemExit('没有本目录的活动试验')
a=json.loads(p.read_text());marker=f"/work/a_arm_hover/runs/{a['name']}/startup.sh";top=subprocess.check_output(['docker','top',C,'-eo','pid,args'],text=True)
if marker in top:
 subprocess.run(['docker','exec','-e','GZ_PARTITION=a_arm_hover',C,P+'/px4-shutdown'],capture_output=True,timeout=10)
for pid in a['pids'].values():
 try:
  if b'a_arm_hover' in Path(f'/proc/{pid}/cmdline').read_bytes():os.killpg(pid,signal.SIGTERM)
 except (ProcessLookupError,FileNotFoundError):pass
# gz rewrites its process title; match the exact server command, not argv regex.
processes=subprocess.check_output(['docker','exec',C,'ps','-eo','pid,args'],text=True)
for line in processes.splitlines()[1:]:
 fields=line.strip().split(None,1)
 if len(fields)==2 and fields[1]=='gz sim -s -r /work/a_arm_hover/world.sdf':
  subprocess.run(['docker','exec',C,'kill','-TERM',fields[0]],check=False)
time.sleep(2)
processes=subprocess.check_output(['docker','exec',C,'ps','-eo','pid,args'],text=True)
for line in processes.splitlines()[1:]:
 fields=line.strip().split(None,1)
 if len(fields)==2 and fields[1]=='gz sim -s -r /work/a_arm_hover/world.sdf':
  subprocess.run(['docker','exec',C,'kill','-KILL',fields[0]],check=False)
p.unlink();print('Stopped owned SITL session',a['name'])
