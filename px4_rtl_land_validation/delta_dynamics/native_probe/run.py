from pathlib import Path
import subprocess,json,xml.etree.ElementTree as E,hashlib
R=Path(__file__).resolve().parent;P='/work/delta_dynamics/native_probe';C='px4-rtl-land-harmonic'
def add(p,t,s=None,**attrs):
 e=E.SubElement(p,t,attrs)
 if s is not None:e.text=str(s)
 return e
def world(mode,yaw):
 root=E.Element('sdf',version='1.9');w=add(root,'world',name='native_probe');p=add(w,'physics',type='ignored');add(p,'max_step_size',.0002);add(p,'real_time_factor',0);add(w,'gravity','0 0 -9.81');add(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
 m=add(w,'model',name='test');add(m,'pose',f'0 0 1 0 0 {yaw}')
 def link(name,xyz,mass,com):
  l=add(m,'link',name=name);add(l,'pose',xyz+' 0 0 0');ii=add(l,'inertial');add(ii,'mass',mass);add(ii,'pose',com+' 0 0 0');i=add(ii,'inertia')
  for k in ['ixx','iyy','izz']:add(i,k,.0002 if name!='tip' else 1e-6)
 def joint(name,parent,child,pose='',kind='ball'):
  j=add(m,'joint',name=name,type=kind);add(j,'parent',parent);add(j,'child',child)
  if pose:add(j,'pose',pose+' 0 0 0')
 link('base','0 0 0',1,'0 0 0');link('bar','0 0 0',.1,'0 0 -.04');joint('fixed','world','base',kind='fixed');joint('left','base','bar','-.1 0 0')
 if mode=='direct':joint('right','base','bar','.1 0 0')
 if mode=='proxy':
  link('tip','.1 0 0',.0001,'0 0 0');joint('right','bar','tip')
 name=f'{mode}_{yaw}'.replace('.','p');plugin=add(m,'plugin',filename='libnative_probe.so',name='native_probe::Probe');add(plugin,'proxy',str(mode=='proxy').lower());add(plugin,'output',P+'/'+name+'.jsonl');E.indent(root);E.ElementTree(root).write(R/(name+'.sdf'),encoding='utf-8',xml_declaration=True);return name
subprocess.run(['docker','exec',C,'bash','-lc',f'cd {P} && g++ -O2 -std=c++17 -shared -fPIC probe.cpp -o libnative_probe.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
results=[]
for mode in ['open','direct','proxy']:
 for yaw in [0,.37]:
  name=world(mode,yaw);cmd=['docker','exec','-e','GZ_PARTITION=a_native_probe','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}',C,'gz','sim','-s','-r','--iterations','5000',P+'/'+name+'.sdf']
  with (R/(name+'.log')).open('w') as f:
   try:ret=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=30);code=ret.returncode
   except subprocess.TimeoutExpired:
    subprocess.run(['docker','exec',C,'pkill','-f','^gz sim.*'+P+'/'+name+'.sdf'],check=False);code='timeout'
  rows=[json.loads(s) for s in (R/(name+'.jsonl')).read_text().splitlines()] if (R/(name+'.jsonl')).exists() else []
  result={'mode':mode,'yaw_rad':yaw,'exit_code':code,'samples':len(rows),'final_sim_s':rows[-1]['sim_s'] if rows else None,'max_gap_m':max([x['endpoint_gap_m'] for x in rows],default=None),'final_gap_m':rows[-1]['endpoint_gap_m'] if rows else None};results.append(result);print(json.dumps(result),flush=True)
(R/'results.json').write_text(json.dumps(results,indent=2))
