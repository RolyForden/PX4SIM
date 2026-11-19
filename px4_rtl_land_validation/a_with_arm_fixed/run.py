"""Fixed-body integration only; no PX4, motor model or flight commands."""
from pathlib import Path
import argparse,subprocess,sys,json,os
R=Path(__file__).resolve().parent;P='/work/a_with_arm_fixed';C='px4-rtl-land-harmonic';V=R.parent
ap=argparse.ArgumentParser();ap.add_argument('--payload',type=float,default=.05);ap.add_argument('--gui',action='store_true');args=ap.parse_args()
if (R/'pids.json').exists():raise SystemExit('请先运行本目录 stop.py')
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
if not (R/'assets.json').exists():subprocess.run([str(V/'.venv/bin/python'),str(R/'prepare_assets.py')],check=True)
subprocess.run([sys.executable,str(R/'build_model.py'),'--payload',str(args.payload)],check=True)
so=R/'liba_with_arm_fixed.so'
if not so.exists() or max((R/'controller.cpp').stat().st_mtime,(R/'cad_geometry.hh').stat().st_mtime)>so.stat().st_mtime:subprocess.run(['docker','exec',C,'bash','-lc',f'cd {P} && g++ -std=c++17 -O2 -shared -fPIC controller.cpp -o liba_with_arm_fixed.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
logger=V/'delta_dynamics/study/libwrench_logger.so'
if not logger.exists():subprocess.run(['docker','exec',C,'bash','-lc','cd /work/delta_dynamics/study && g++ -std=c++17 -O2 -shared -fPIC wrench_logger.cpp -o libwrench_logger.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
env=['docker','exec','-e','GZ_PARTITION=a_with_arm_fixed','-e','GZ_IP=127.0.0.1','-e',f'GZ_SIM_RESOURCE_PATH={P}/models:/work/a_morphing/airborne/models:/work/delta_fixed/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}:/work/delta_dynamics/study','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia',C]
cmds={'server':env+['gz','sim','-s','-r',f'{P}/world.sdf'],'ros':['bash','-c',f'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "{R}/ros_control.py"']}
if args.gui:cmds['gui']=env+['gz','sim','-g','--gui-config',f'{P}/gui.config']
pids={}
for key,cmd in cmds.items():
 with (R/'results'/f'{key}.log').open('w') as f:pids[key]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
(R/'pids.json').write_text(json.dumps(pids));print('Integrated fixed-body bench started',pids)
