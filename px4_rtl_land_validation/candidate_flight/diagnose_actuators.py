"""Zero-gravity isolated motor/sign check, no PX4 and no flying test."""
import pathlib,subprocess,time,json,xml.etree.ElementTree as E
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parent;C='px4-rtl-land-harmonic';PART='px4_candidate_motor_diag'
base=['docker','exec','-e','GZ_SIM_SYSTEM_PLUGIN_PATH=/work/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins','-e','GZ_PARTITION='+PART,'-e','GZ_SIM_RESOURCE_PATH=/work/candidate_flight/models','-e','GZ_SIM_SERVER_CONFIG_PATH=/work/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config',C]
def gz(*a):return subprocess.check_output(base+['gz',*a],text=True,timeout=20)
w=E.parse(ROOT/'candidate_world.sdf');world=w.find('world');world.set('name','motor_diag');world.find('gravity').text='0 0 0';world.find('physics/max_step_size').text='.001'
for i in range(4):
 inc=E.SubElement(world,'include');E.SubElement(inc,'uri').text='model://candidate_rigid';E.SubElement(inc,'name').text=f'diag_{i}';E.SubElement(inc,'pose').text=f'{i*2} 0 3 0 0 0'
w.write(ROOT/'motor_diag.sdf',encoding='utf-8',xml_declaration=True)
with (ROOT/'motor_diag_server.log').open('w') as log:
 p=subprocess.Popen(base+['gz','sim','-s','-r','/work/candidate_flight/motor_diag.sdf'],stdout=log,stderr=subprocess.STDOUT)
 try:
  for tries in range(30):
   if '/world/motor_diag/clock' in gz('topic','-l'):break
   time.sleep(.3)
  time.sleep(2)
  for i in range(4):
   command='velocity: ['+', '.join('100' if j==i else '0' for j in range(4))+']'
   gz('topic','-t',f'/diag_{i}/command/motor_speed','-m','gz.msgs.Actuators','-p',command)
  time.sleep(.5);results=[]
  for i in range(4):
   data=json.JSONDecoder().raw_decode(gz('topic','-e','-t',f'/world/motor_diag/model/diag_{i}/link/base_link/sensor/imu_sensor/imu','-n','1','--json-output').lstrip())[0]
   angular=data.get('angularVelocity',{});v=np.array([angular.get(k,0) for k in ['x','y','z']]);expected=np.array([[-1,-1,-1],[1,1,-1],[1,-1,1],[-1,1,1]])[i]
   results.append({'motor':i,'gyro_flu_rad_s':v.tolist(),'expected_sign_flu':expected.tolist(),'signs_match':bool(np.all(v*expected>0)),'raw':data})
  (ROOT/'motor_sign_check.json').write_text(json.dumps({'status':'zero gravity isolated actuator diagnostic; not flight','results':results},indent=2));print(json.dumps(results,indent=2))
 finally:
  top=subprocess.check_output(['docker','exec',C,'ps','-eo','pid,args'],text=True)
  for row in top.splitlines():
   a=row.strip().split(None,1)
   if len(a)==2 and a[1].startswith('gz sim -s -r /work/candidate_flight/motor_diag.sdf'):subprocess.run(['docker','exec',C,'kill','-INT',a[0]])
