"""Isolated local standard-PX4 SITL, one runtime and world per trial."""
from pathlib import Path
import argparse,subprocess,json,sys,shutil,os
R=Path(__file__).resolve().parent;V=R.parent;P='/work/PX4-Autopilot';C='px4-rtl-land-harmonic';A='/work/a_locked_hover'
ap=argparse.ArgumentParser();ap.add_argument('--name',required=True);ap.add_argument('--payload',type=float,choices=[0,.05],default=0);args=ap.parse_args();D=R/'runs'/args.name
if D.exists():raise SystemExit('试验目录已存在')
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL);top=subprocess.check_output(['docker','top',C,'-eo','pid,args'],text=True)
if '/bin/px4 ' in top or '/px4_morph ' in top:raise SystemExit('另一个 PX4 正在运行')
subprocess.run([sys.executable,str(R/'build_model.py'),'--payload',str(args.payload)],check=True,stdout=subprocess.DEVNULL)
D.mkdir();(D/'logs').mkdir();(D/'parameters').mkdir();runtime=D/'runtime';shutil.copytree(V/'PX4-Autopilot/build/px4_sitl_default/etc',runtime/'etc');shutil.copy2(R/'4995_a_locked',runtime/'etc/init.d-posix/airframes/4995_a_locked')
(D/'startup.sh').write_text('#!/bin/sh\n. ${R}etc/init.d-posix/rcS\nmavlink start -x -u 14651 -o 14650 -t 127.0.0.1 -m onboard -r 1000000\n')
for src,dest in [('preflight.json','preflight.json'),('reference-control-parameters.json','reference-control-parameters.json'),('models/a_locked/model.sdf','model.sdf'),('4995_a_locked','airframe.sh'),('world.sdf','world.sdf')]:shutil.copy2(R/src,D/dest)
env=['docker','exec','-e','GZ_PARTITION=a_locked_hover','-e','GZ_IP=127.0.0.1','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia','-e',f'GZ_SIM_RESOURCE_PATH={A}/models:/work/a_with_arm_fixed/models:/work/a_morphing/airborne/models:/work/delta_fixed/models:{P}/Tools/simulation/gz/models','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}/build/px4_sitl_default/src/modules/simulation/gz_plugins','-e',f'GZ_SIM_SERVER_CONFIG_PATH={P}/src/modules/simulation/gz_bridge/server.config']
cmds={'server':env+[C,'gz','sim','-s','-r',A+'/world.sdf'],'px4':env+['-e',f'PATH={P}/build/px4_sitl_default/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin','-e','PX4_SYS_AUTOSTART=4995','-e','PX4_GZ_STANDALONE=1','-e',f'PX4_GZ_MODELS={A}/models','-e','PX4_GZ_WORLD=a_locked_world','-e','PX4_SIM_MODEL=a_locked','-e','PX4_GZ_MODEL_POSE=0,0,0.35,0,0,0',C,'stdbuf','-oL','-eL',P+'/build/px4_sitl_default/bin/px4','-d','-s',f'{A}/runs/{args.name}/startup.sh','-w',f'{A}/runs/{args.name}/runtime',f'{A}/runs/{args.name}/runtime/etc']}
ids={}
for key,cmd in cmds.items():
 with (D/f'{key}.log').open('w') as f:ids[key]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
(D/'processes.json').write_text(json.dumps({'pids':ids,'commands':cmds},indent=2));(R/'active.json').write_text(json.dumps({'name':args.name,'pids':ids},indent=2));print('Started locked-arm SITL',args.name,ids)
