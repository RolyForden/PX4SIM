from pathlib import Path
import subprocess,socket,json,time,struct,argparse
R=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--duration',type=float,default=10);ap.add_argument('--name',default='probe');args=ap.parse_args()
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',14831));s.settimeout(15)
cmd=['docker','exec','-e','GZ_PARTITION=a_delta_dynamics','-e','GZ_IP=127.0.0.1','-e','GZ_SIM_RESOURCE_PATH=/work/delta_fixed/models','-e','GZ_SIM_SYSTEM_PLUGIN_PATH=/work/delta_dynamics','px4-rtl-land-harmonic','gz','sim','-s','-r','/work/delta_dynamics/world.sdf']
log=(R/'results'/(args.name+'-server.log')).open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT);rows=[];last=-1
try:
 while True:
  data,_=s.recvfrom(8192);row=json.loads(data);rows.append(row)
  if row['sim_s']>last+1:print(json.dumps(row),flush=True);last=row['sim_s']
  if row['sim_s']>=args.duration:break
finally:
 subprocess.run(['docker','exec','px4-rtl-land-harmonic','pkill','-f','^gz sim -s -r /work/delta_dynamics/world.sdf'],check=False);proc.wait(timeout=15);s.close();log.close();(R/'results'/(args.name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
