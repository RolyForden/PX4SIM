"""Start fixed-body force-closure Gazebo/ROS bench, with explicit test load."""
from pathlib import Path
import subprocess,os,json,sys,argparse
R=Path(__file__).resolve().parent;C='px4-rtl-land-harmonic';P='/work/delta_dynamics'
ap=argparse.ArgumentParser();ap.add_argument('--gui',action='store_true');ap.add_argument('--payload',type=float,default=0);ap.add_argument('--gravity',type=float,default=9.81);ap.add_argument('--dt',type=float,default=.0002);args=ap.parse_args()
for pid in json.loads((R/'pids.json').read_text()).values() if (R/'pids.json').exists() else []:
 try:cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
 except FileNotFoundError:continue
 if b'delta_dynamics' in cmd:raise SystemExit('台架已运行，先执行 stop.py')
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
subprocess.run([sys.executable,str(R/'build_model.py'),'--payload',str(args.payload),'--gravity',str(args.gravity),'--dt',str(args.dt)],check=True)
so=R/'libdelta_dynamics.so'
if not so.exists() or max((R/'controller.cpp').stat().st_mtime,(R/'cad_geometry.hh').stat().st_mtime)>so.stat().st_mtime:
 subprocess.run(['docker','exec',C,'bash','-lc',f'cd {P} && g++ -std=c++17 -O2 -shared -fPIC controller.cpp -o libdelta_dynamics.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
env=['docker','exec','-e','GZ_PARTITION=a_delta_dynamics','-e','GZ_IP=127.0.0.1','-e','GZ_SIM_RESOURCE_PATH=/work/delta_fixed/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia',C]
commands={'server':env+['gz','sim','-s','-r',f'{P}/world.sdf'],'ros':['bash','-c',f'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "{R}/ros_control.py"']}
if args.gui:commands['gui']=env+['gz','sim','-g','--gui-config',f'{P}/gui.config']
(R/'results').mkdir(exist_ok=True);pids={}
for name,cmd in commands.items():
 with (R/'results'/f'{name}.log').open('w') as f:pids[name]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
(R/'pids.json').write_text(json.dumps(pids,indent=2));print('固定机身受力台架已启动',pids)
