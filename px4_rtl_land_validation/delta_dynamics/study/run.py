"""One-factor sensitivity campaign; preserve the baseline controller source."""
from pathlib import Path
import subprocess,json,xml.etree.ElementTree as E,sys,shutil,os
R=Path(__file__).resolve().parent;B=R.parent;P='/work/delta_dynamics';C='px4-rtl-land-harmonic'
subprocess.run(['docker','exec',C,'bash','-lc',f'cd {P}/study && g++ -O2 -std=c++17 -shared -fPIC wrench_logger.cpp -o libwrench_logger.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
cases=[('baseline',2000,3),('stiffness_1000',1000,3),('stiffness_4000',4000,3),('damping_1p5',2000,1.5),('damping_6',2000,6)]
for name,k,d in cases:
 out=R/name
 if (out/'summary.json').exists():print('Existing result retained:',name,flush=True);continue
 out.mkdir(exist_ok=True);tag='sensitivity_'+name
 if (B/'results'/tag/'summary.json').exists():raise SystemExit('已有同名实验结果：'+tag)
 subprocess.run([sys.executable,str(B/'stop.py')],check=True)
 subprocess.run([sys.executable,str(B/'build_model.py'),'--payload','.05','--stiffness',str(k),'--damping',str(d)],check=True,stdout=subprocess.DEVNULL)
 t=E.parse(B/'world.sdf');m=t.find("world/model[@name='delta_structure']");p=E.SubElement(m,'plugin',filename='libwrench_logger.so',name='study::WrenchLogger');E.SubElement(p,'joint').text='fixed_body';E.SubElement(p,'output').text=f'{P}/study/{name}/mount_wrench.jsonl';t.write(B/'world.sdf',encoding='utf-8',xml_declaration=True)
 cmds={'server':['docker','exec','-e','GZ_PARTITION=a_delta_dynamics','-e','GZ_IP=127.0.0.1','-e','GZ_SIM_RESOURCE_PATH=/work/delta_fixed/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}:{P}/study',C,'gz','sim','-s','-r',f'{P}/world.sdf'],'ros':['bash','-c',f'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "{B}/ros_control.py"']}
 ids={}
 for key,cmd in cmds.items():
  with (out/(key+'.log')).open('w') as f:ids[key]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
 (B/'pids.json').write_text(json.dumps(ids))
 try:
  proc=subprocess.run(['bash','-c','source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "$1" --name "$2"','bash',str(B/'verify.py'),tag],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120);(out/'verification.log').write_text(proc.stdout)
  if proc.returncode:
   shutil.copy2(B/'results/session.jsonl',out/'feedback.jsonl');shutil.copy2(B/'settings.json',out/'settings.json');shutil.copy2(B/'world.sdf',out/'world.sdf')
   rows=[json.loads(l) for l in (out/'feedback.jsonl').read_text().splitlines()]
   reason=rows[-1]['error'] or proc.stdout.strip().splitlines()[-1]
   (out/'summary.json').write_text(json.dumps({'passed':False,'settings':json.loads((B/'settings.json').read_text()),'error':reason,'max_closure_m':max(r['closure_m'] for r in rows),'max_tracking_error_m':max(r['tracking_error_m'] for r in rows),'final_tracking_error_m':rows[-1]['tracking_error_m'],'segment':rows[-1]['segment']},ensure_ascii=False,indent=2));print('FAILED',name,reason,flush=True);continue
  src=B/'results'/tag
  for n in ['summary.json','feedback.jsonl','settings.json','world.sdf']:
   if (src/n).exists():shutil.copy2(src/n,out/n)
  print('COMPLETED',name,k,d,flush=True)
 finally:subprocess.run([sys.executable,str(B/'stop.py')],check=False,stdout=subprocess.DEVNULL)
