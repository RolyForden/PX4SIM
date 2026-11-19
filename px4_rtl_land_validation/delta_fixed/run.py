"""Start only the fixed Delta bench; never starts PX4 or QGC."""
from pathlib import Path
import subprocess,os,json,sys
R=Path(__file__).resolve().parent;C='px4-rtl-land-harmonic';P='/work/delta_fixed';result=R/'results';result.mkdir(exist_ok=True)
if (R/'pids.json').exists():
 old=json.loads((R/'pids.json').read_text())
 for pid in old.values():
  try:os.kill(pid,0)
  except ProcessLookupError:continue
  raise SystemExit('台架仍在运行，先运行 stop.py')
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
if not (R/'libdelta_fixed.so').exists() or max((R/'controller.cpp').stat().st_mtime,(R/'cad_geometry.hh').stat().st_mtime)>(R/'libdelta_fixed.so').stat().st_mtime:subprocess.run(['docker','exec',C,'bash','-lc',f'cd {P} && g++ -std=c++17 -O2 -shared -fPIC controller.cpp -o libdelta_fixed.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
env=['docker','exec','-e','GZ_PARTITION=a_delta_fixed','-e','GZ_IP=127.0.0.1','-e',f'GZ_SIM_RESOURCE_PATH={P}/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia',C]
commands={'server':env+['gz','sim','-s','-r',f'{P}/world.sdf'],'ros':['bash','-c',f'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "{R}/ros_control.py"']}
if '--gui' in sys.argv:commands['gui']=env+['gz','sim','-g','--gui-config',f'{P}/gui.config']
pids={}
for name,cmd in commands.items():
 with (result/(name+'.log')).open('w') as f:pids[name]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
(R/'pids.json').write_text(json.dumps(pids,indent=2));print('固定台架已启动：',pids)
