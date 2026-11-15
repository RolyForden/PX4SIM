"""Launch isolated candidate SITL without altering upstream PX4 or X500 files."""
import pathlib,subprocess,json
ROOT=pathlib.Path(__file__).resolve().parent
C='px4-rtl-land-harmonic';P='/work/PX4-Autopilot'
base=['docker','exec','-e','GZ_IP=127.0.0.1','-e','LIBGL_ALWAYS_SOFTWARE=0','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia','-e','GZ_PARTITION=px4_candidate_flight','-e',f'GZ_SIM_RESOURCE_PATH=/work/candidate_flight/models:{P}/Tools/simulation/gz/models:{P}/Tools/simulation/gz/worlds','-e',f'GZ_SIM_SYSTEM_PLUGIN_PATH={P}/build/px4_sitl_default/src/modules/simulation/gz_plugins','-e',f'GZ_SIM_SERVER_CONFIG_PATH={P}/src/modules/simulation/gz_bridge/server.config']
top=subprocess.check_output(['docker','top',C,'-eo','pid,args'],text=True)
if P+'/build/px4_sitl_default/bin/px4' in top:raise RuntimeError('A PX4 SITL process already exists; stop the previous simulation first')
server=base+[C,'gz','sim','-s','-r','/work/candidate_flight/candidate_world.sdf']
with (ROOT/'gazebo_server.log').open('w') as f:s=subprocess.Popen(server,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
px4=base+['-e','PX4_SYS_AUTOSTART=4999','-e','PX4_GZ_STANDALONE=1','-e','PX4_GZ_MODELS=/work/candidate_flight/models','-e','PX4_GZ_WORLD=candidate_world','-e','PX4_SIM_MODEL=candidate_rigid',C,'stdbuf','-oL','-eL',P+'/build/px4_sitl_default/bin/px4', '-d','-s','/work/candidate_flight/startup.sh','-w','/work/candidate_flight/runtime','/work/candidate_flight/runtime/etc']
with (ROOT/'sitl_console.log').open('w') as f:p=subprocess.Popen(px4,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
(ROOT/'launch_record.json').write_text(json.dumps({'server':server,'px4':px4,'host_processes':[s.pid,p.pid]},indent=2));print('Started isolated candidate server and PX4',s.pid,p.pid)
