"""Start independent free-flight world, standard PX4 and ROS trajectory node."""
from pathlib import Path
import argparse,subprocess,json,sys,shutil,os
R=Path(__file__).resolve().parent;V=R.parent;P='/work/PX4-Autopilot';C='px4-rtl-land-harmonic';A='/work/a_arm_hover'
ap=argparse.ArgumentParser();ap.add_argument('--name',required=True);ap.add_argument('--payload',type=float,choices=[0,.05],default=0);args=ap.parse_args();D=R/'runs'/args.name
if D.exists() or (R/'active.json').exists():raise SystemExit('Existing run/active session; inspect before starting')
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
top=subprocess.check_output(['docker','top',C,'-eo','pid,args'],text=True)
if '/bin/px4 ' in top or '/px4_morph ' in top:raise SystemExit('Another PX4 running')
subprocess.run([str(V/'.venv/bin/python'),str(R/'build_model.py'),'--payload',str(args.payload)],check=True)
subprocess.run(['docker','exec',C,'bash','-lc',f'cd {A} && g++ -std=c++17 -O2 -shared -fPIC controller.cpp -o liba_arm_hover.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)'],check=True)
D.mkdir(parents=True);(D/'logs').mkdir();(D/'parameters').mkdir();rt=D/'runtime'
shutil.copytree(V/'PX4-Autopilot/build/px4_sitl_default/etc',rt/'etc');shutil.copy2(R/'4994_a_arm',rt/'etc/init.d-posix/airframes/4994_a_arm')
(D/'startup.sh').write_text('#!/bin/sh\n. ${R}etc/init.d-posix/rcS\nmavlink start -x -u 14651 -o 14650 -t 127.0.0.1 -m onboard -r 1000000\n')
for f in ['world.sdf','preflight.json','reference-control-parameters.json','4994_a_arm','controller.cpp','ros_control.py']:shutil.copy2(R/f,D/f)
env=['docker','exec','-e','GZ_PARTITION=a_arm_hover','-e','GZ_IP=127.0.0.1','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia','-e',f'GZ_SIM_RESOURCE_PATH=/work/a_with_arm_fixed/models:/work/a_morphing/airborne/models:/work/delta_fixed/models:{P}/Tools/simulation/gz/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={A}:{P}/build/px4_sitl_default/src/modules/simulation/gz_plugins','-e',f'GZ_SIM_SERVER_CONFIG_PATH={P}/src/modules/simulation/gz_bridge/server.config']
cmds={'server':env+[C,'gz','sim','-s','-r',A+'/world.sdf'],'px4':env+['-e',f'PATH={P}/build/px4_sitl_default/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin','-e','PX4_SYS_AUTOSTART=4994','-e','PX4_GZ_STANDALONE=1','-e','PX4_GZ_MODEL_NAME=a_arm','-e','PX4_GZ_WORLD=a_arm_world','-e','PX4_SIM_MODEL=a_arm',C,'stdbuf','-oL','-eL',P+'/build/px4_sitl_default/bin/px4','-d','-s',f'{A}/runs/{args.name}/startup.sh','-w',f'{A}/runs/{args.name}/runtime',f'{A}/runs/{args.name}/runtime/etc'],'ros':['bash','-c',f'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "{R}/ros_control.py"']}
pids={}
for key,cmd in cmds.items():
 with (D/f'{key}.log').open('w') as f:pids[key]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env={**os.environ,'A_ARM_RUN_DIR':str(D)}).pid
(D/'processes.json').write_text(json.dumps({'pids':pids,'commands':cmds},indent=2));(R/'active.json').write_text(json.dumps({'name':args.name,'pids':pids}));print('Started',args.name,pids,flush=True)
