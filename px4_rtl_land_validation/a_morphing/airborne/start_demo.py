"""Start the articulated A SITL, native feedback, ROS and desktop windows."""
from pathlib import Path
import subprocess,os,json,shutil,sys
R=Path(__file__).resolve().parent;V=R.parents[1];C='px4-rtl-land-harmonic'
subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
top=subprocess.check_output(['docker','top',C,'-eo','pid,args'],text=True)
if any(x in top for x in ['/bin/px4 ','/airborne/px4_morph ']):raise SystemExit('PX4 already running; use existing session or stop it first')
if not (R/'px4_morph').exists():raise SystemExit('Run build_px4.py first')
if not (R/'gz_link').exists():
 subprocess.run(['docker','exec',C,'bash','-lc','g++ -std=c++17 -O2 /work/a_morphing/airborne/gz_link.cpp -o /work/a_morphing/airborne/gz_link $(pkg-config --cflags --libs gz-transport13 gz-msgs10) -pthread'],check=True)
subprocess.run([sys.executable,str(R/'prepare_runtime.py')],check=True)
subprocess.run([sys.executable,str(R/'launch.py')],check=True)
if not (R/'gui.config').exists():raise SystemExit('Missing gui.config')
commands={
 'native_link':['docker','exec','-e','GZ_IP=127.0.0.1','-e','GZ_PARTITION=a_air',C,'/work/a_morphing/airborne/gz_link'],
 'ros':['bash','-c','source /opt/ros/humble/setup.bash && exec /usr/bin/python3 '+str(R/'ros_control.py')],
 'gui':['docker','exec','-e','GZ_IP=127.0.0.1','-e','GZ_PARTITION=a_air','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia','-e','GZ_SIM_RESOURCE_PATH=/work/a_morphing/airborne/models:/work/PX4-Autopilot/Tools/simulation/gz/models',C,'gz','sim','-g','--gui-config','/work/a_morphing/airborne/gui.config']}
pids={}
for name,cmd in commands.items():
 with (R/(name+'-session.log')).open('w') as f:pids[name]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid
windows=subprocess.check_output(['wmctrl','-l'],text=True)
if 'QGroundControl' not in windows:
 config=R/'desktop_config';folder=config/'QGroundControl';folder.mkdir(parents=True,exist_ok=True)
 ini=folder/'QGroundControl.ini'
 if not ini.exists():ini.write_text('[General]\nvirtualJoystick=true\nvirtualJoystickAutoCenterThrottle=true\nappFontPointSize=10\n\n[AutoConnect]\nautoConnectUDP=true\nudpListenPort=14550\nudpTargetHostIP=127.0.0.1\nudpTargetHostPort=18570\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\nautoConnectZeroConf=false\n')
 env=os.environ.copy();env['XDG_CONFIG_HOME']=str(config)
 with (R/'qgc-session.log').open('w') as f:pids['qgc']=subprocess.Popen(['/home/ray/下载/QGroundControl-v5.0.8-x86_64.AppImage'],stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True).pid
(R/'desktop-session.json').write_text(json.dumps(pids,indent=2));print('Started; verify /a_uav/morph/status ready before operating QGC')
