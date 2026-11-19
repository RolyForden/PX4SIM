from pathlib import Path
import subprocess,json,os,argparse,sys
R=Path(__file__).resolve().parent;P='/work/delta_dynamics/native';C='px4-rtl-land-harmonic'
ap=argparse.ArgumentParser();ap.add_argument('--payload',type=float,default=.05);ap.add_argument('--gravity',type=float,default=9.81);ap.add_argument('--gui',action='store_true');ap.add_argument('--base-rpy',nargs=3,type=float,default=[0,0,0]);args=ap.parse_args()
if (R/'pids.json').exists():raise SystemExit('先运行 native/stop.py')
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
logger=R.parent/'study/libwrench_logger.so';logger_src=R.parent/'study/wrench_logger.cpp'
if not logger.exists() or logger_src.stat().st_mtime>logger.stat().st_mtime:subprocess.run(['docker','exec',C,'bash','-lc','cd /work/delta_dynamics/study && g++ -O2 -std=c++17 -shared -fPIC wrench_logger.cpp -o libwrench_logger.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
subprocess.run([sys.executable,str(R/'build_model.py'),'--payload',str(args.payload),'--gravity',str(args.gravity),'--base-rpy',*map(str,args.base_rpy)],check=True)
so=R/'libdelta_native.so'
if not so.exists() or (R/'controller.cpp').stat().st_mtime>so.stat().st_mtime:subprocess.run(['docker','exec',C,'bash','-lc',f'cd {P} && g++ -std=c++17 -O2 -shared -fPIC controller.cpp -o libdelta_native.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
env=['docker','exec','-e','GZ_PARTITION=a_delta_native','-e','GZ_IP=127.0.0.1','-e','GZ_SIM_RESOURCE_PATH=/work/delta_fixed/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}:/work/delta_dynamics/study','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia',C]
cmds={'server':env+['gz','sim','-s','-r',f'{P}/world.sdf'],'ros':['bash','-c',f'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "{R}/ros_control.py"']}
if args.gui:cmds['gui']=env+['gz','sim','-g','--gui-config',f'{P}/gui.config']
ids={}
for name,cmd in cmds.items():
 with (R/'results'/f'{name}.log').open('w') as f:ids[name]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
(R/'pids.json').write_text(json.dumps(ids));print('Started native-constraint bench',ids)
