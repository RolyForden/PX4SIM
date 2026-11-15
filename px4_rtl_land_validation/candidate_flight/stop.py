"""Stop only this project's candidate SITL/diagnostic processes; no hardware."""
import subprocess,signal,os,pathlib,json,datetime,time
ROOT=pathlib.Path(__file__).resolve().parent;C='px4-rtl-land-harmonic'
for row in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines():
 a=row.strip().split(None,2)
 if len(a)==3 and a[1].startswith('python') and a[2].endswith('candidate_flight/test_flight.py'):os.kill(int(a[0]),signal.SIGINT)
r=subprocess.run(['docker','exec',C,'ps','-eo','pid,args'],capture_output=True,text=True);stopped=[]
for row in r.stdout.splitlines():
 a=row.strip().split(None,1)
 if len(a)!=2:continue
 pid,args=a
 if ('/work/candidate_flight/' in args and (args.startswith('gz sim -s ') or args.startswith('/work/PX4-Autopilot/build/px4_sitl_default/bin/px4 '))):
  subprocess.run(['docker','exec',C,'kill','-INT',pid],capture_output=True);stopped.append({'pid':int(pid),'command':args})
(ROOT/'stop_record.json').write_text(json.dumps({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'stopped':stopped},indent=2));print(json.dumps(stopped,indent=2))
