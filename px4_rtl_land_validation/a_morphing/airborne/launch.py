from pathlib import Path
import subprocess,json
R=Path(__file__).resolve().parent;C='px4-rtl-land-harmonic';P='/work/PX4-Autopilot';A='/work/a_morphing/airborne'
base=['docker','exec','-e','GZ_IP=127.0.0.1','-e','GZ_PARTITION=a_air','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia','-e',f'GZ_SIM_RESOURCE_PATH={A}/models:{P}/Tools/simulation/gz/models:{P}/Tools/simulation/gz/worlds','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}/build/px4_sitl_default/src/modules/simulation/gz_plugins','-e',f'GZ_SIM_SERVER_CONFIG_PATH={P}/src/modules/simulation/gz_bridge/server.config']
commands=[base+[C,'gz','sim','-s','-r',A+'/world.sdf'],base+['-e',f'PATH={P}/build/px4_sitl_default/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin','-e','PX4SIM_MORPH_UDP=1','-e','PX4_SYS_AUTOSTART=4996','-e','PX4_GZ_STANDALONE=1','-e',f'PX4_GZ_MODELS={A}/models','-e','PX4_GZ_WORLD=candidate_world','-e','PX4_SIM_MODEL=a_air',C,'stdbuf','-oL','-eL',A+'/px4_morph','-d','-s',A+'/startup.sh','-w',A+'/runtime',A+'/runtime/etc']]
pids=[]
for cmd,name in zip(commands,['server','px4']):
 with (R/(name+'.log')).open('w') as f:pids.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True).pid)
(R/'session.json').write_text(json.dumps({'pids':pids,'commands':commands},indent=2));print(pids)
