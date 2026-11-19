"""3 m hover with native arm trajectory, guarded landing and native ULog."""
from pathlib import Path
import json,time,math,sys,shutil,subprocess,hashlib,signal,re
import numpy as np
import flight_link as m
from pymavlink import mavutil
R=Path(__file__).resolve().parent;name=sys.argv[1];D=R/'runs'/name;m.ROOT=D;M=mavutil.mavlink
class Flight(m.Flight):
 def __init__(self,name):
  self.morph={};self.morph_received=0;self.morphfile=(D/'morph-feedback.jsonl').open('r');self.arm={};self.arm_received=0;self.armfile=(D/'arm-feedback.jsonl').open('r')
  super().__init__(name)
 def event(self,what,**kw):
  super().event(what,arm_sim_s=self.arm.get('sim_s'),arm_seq=self.arm.get('seq'),morph_seq=self.morph.get('sequence'),**kw)
 def arm_command(self,command):
  cmd=['bash','-c','source /opt/ros/humble/setup.bash && ros2 topic pub --once /a_uav/coupled/arm/command std_msgs/msg/String '+"'"+'{data: "'+command+'"}'+"'"]
  with (D/'ros-command.log').open('a') as out:subprocess.Popen(cmd,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  self.event('arm_command_sent',command=command)

 def pump(self,duration=.1,setpoint=None):
  end=time.monotonic()+duration
  while time.monotonic()<end:
   self.link.mav.manual_control_send(1,0,0,500,0,0);super().pump(min(.04,max(0,end-time.monotonic())),setpoint)
   while True:
    offset=self.armfile.tell();line=self.armfile.readline()
    if not line:break
    if not line.endswith('\n'):self.armfile.seek(offset);break
    self.arm=json.loads(line);self.arm_received=time.monotonic()
   while True:
    offset=self.morphfile.tell();line=self.morphfile.readline()
    if not line:break
    if not line.endswith('\n'):self.morphfile.seek(offset);break
    self.morph=json.loads(line);self.morph_received=time.monotonic()
   if getattr(self,'guard',False) and self.state.get('armed'):
    if time.monotonic()-self.morph_received>1 or not self.morph.get('ready'):raise RuntimeError('Abort: morph feedback/allocator unhealthy')
    if self.morph.get('tracking_error_deg',0)>8:raise RuntimeError('Abort: morph joint tracking >8deg')
    if time.monotonic()-self.arm_received>2:raise RuntimeError('Abort: arm feedback stale')
    if self.arm.get('closure_m',1)>.001:raise RuntimeError('Abort: native loop closure >1mm')
    if self.arm.get('error'):raise RuntimeError('Abort: '+self.arm['error'])
    s=self.state;o=self.origin
    if max(abs(s.get('roll',0)),abs(s.get('pitch',0)))>math.radians(20):raise RuntimeError('Abort: roll/pitch >20deg')
    if math.hypot(s.get('x',o['x'])-o['x'],s.get('y',o['y'])-o['y'])>2:raise RuntimeError('Abort: horizontal offset >2m')
    h=o['z']-s.get('z',o['z']);vz=s.get('vz',0)
    if h>1 and vz>1.5:raise RuntimeError('Abort: rapid descent above 1m')
    if h>6.5:raise RuntimeError('Abort: unexpected climb above 6.5m')
    if getattr(self,'hover_guard',False):
     outputs=s.get('motor_outputs',[]);upper=990 if outputs and max(outputs)>2 else .98;saturated=bool(outputs) and max(outputs)>=upper
     self.sat_since=(self.sat_since or s.get('boot_ms',0)) if saturated else None
     if self.sat_since and s.get('boot_ms',0)-self.sat_since>1000:raise RuntimeError('Abort: motor output persistently upper-saturated')
 def snapshot(self,tag):
  self.link.mav.param_request_list_send(1,1);self.pump(6)
  if self.params.get('SYS_AUTOSTART')!=4993:raise RuntimeError('Wrong airframe')
  ref=json.loads((D/'reference-control-parameters.json').read_text());changed={k:[v,self.params.get(k)] for k,v in ref.items() if k not in self.params or abs(v-self.params[k])>1e-5*max(1,abs(v))}
  if changed:raise RuntimeError('MC/MPC parameter mismatch: '+json.dumps(changed))
  pre=json.loads((D/'preflight.json').read_text())
  for i in range(4):
   for key,v in zip(['PX','PY','PZ'],pre['rotor_geometry_frd_m'][i]):
    if abs(self.params.get(f'CA_ROTOR{i}_{key}',999)-v)>1e-6:raise RuntimeError('Rotor geometry parameter mismatch')
  (D/'parameters'/f'{tag}.json').write_text(json.dumps(self.params,indent=2));self.event('parameter_snapshot',tag=tag,control_parameters_unchanged=True)
 def stable_sim(self,label,predicate,seconds,timeout=90):
  start=None;end=time.monotonic()+timeout
  while time.monotonic()<end:
   self.pump(.05);boot=self.state.get('boot_ms',0)
   if predicate():
    if start is None:start=boot
    if (boot-start)/1000>=seconds:self.event(label,stable_sim_seconds=(boot-start)/1000);return
   else:start=None
  raise TimeoutError(label)
 def morph_command(self,command):
  cmd=['bash','-c','source /opt/ros/humble/setup.bash && ros2 topic pub --once /a_uav/coupled/morph/command std_msgs/msg/String '+"'"+'{data: "'+command+'"}'+"'"]
  with (D/'ros-command.log').open('a') as out:subprocess.Popen(cmd,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  self.event('morph_command_sent',command=command)
 def morph_to(self,command):
  mode=command[0];goal=np.zeros(8)
  cfg=json.loads((R.parent/'a_morphing/config/morph-config.json').read_text())
  for i,a in enumerate(cfg['arms']):
   if mode=='H':goal[2*i]=math.copysign(25,a['H']['yaw_rad'])
   if mode=='V':goal[2*i+1]=25
  self.morph_command(command)
  self.stable_sim('morph_reached',lambda:self.morph.get('ready') and not self.morph.get('transition') and max(abs(np.array(self.morph['q_deg'])-goal))<.7,1,180)
 def arm_begin(self,command):
  cid=self.arm['command_id'];self.arm_command(command)
  self.wait('arm_motion_accepted',lambda:self.arm.get('command_id',0)>cid,timeout=30)
  return self.arm['command_id']
 def arm_finish(self,cid):
  self.wait('arm_motion_completed',lambda:self.arm.get('completed_id')==cid and not self.arm.get('busy'),timeout=300)
 def arm_demo(self):self.arm_finish(self.arm_begin('demo'))
 def simultaneous(self,mode,armcmd):
  cid=self.arm_begin(armcmd);self.morph_to(mode+':25');self.morph_to('X');self.arm_finish(cid)
 def phase(self,label,action):
  self.event(label+'_start');action();self.event(label+'_end')
 def capture_matrix(self,label):
  offset=(D/'px4.log').stat().st_size
  subprocess.run(['docker','exec','px4-rtl-land-harmonic','/work/PX4-Autopilot/build/px4_sitl_default/bin/px4-control_allocator','status'],check=True,capture_output=True,timeout=8)
  self.pump(.2);raw=(D/'px4.log').read_text(errors='replace')[offset:];rows=[]
  for line in raw.splitlines():
   m=re.match(r'^\s*([0-3])\|(.+)$',line)
   if m:
    vals=[float(x) for x in m.group(2).split()]
    if len(vals)==6:rows.append(vals)
  if len(rows)!=4:raise RuntimeError('PX4 matrix status not captured')
  actual=np.array(rows).T;expected=np.array(self.morph['B_control'])*6.5;expected[np.all(np.abs(expected)<=.05,axis=1)]=0
  error=float(np.max(abs(actual-expected)));record={'label':label,'native_matrix':actual.tolist(),'expected_CT_B':expected.tolist(),'max_abs_error':error,'passed':error<.02,'geometry_record':self.morph,'native_text':raw}
  (D/('matrix-'+label+'.json')).write_text(json.dumps(record,indent=2));self.event('native_matrix_verified',label=label,max_abs_error=error)
  if not record['passed']:raise RuntimeError('Native matrix mismatch')
 def run(self):
  self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL,[375,100000],wait=False);self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL,[230,200000],wait=False)
  self.snapshot('before');self.wait('ready_ground',lambda:self.state.get('landed')==1 and not self.state.get('armed') and all(k in self.state for k in ['x','y','z','alt']) and self.arm.get('ready') and self.arm.get('end_wrench_valid_count')==6 and self.morph.get('ready'),timeout=90)
  self.stable_sim('ground_constraints_settled',lambda:self.arm.get('closure_m',1)<1e-5,5,120);self.capture_matrix('X-ground')
  self.origin={k:self.state[k] for k in ['x','y','z','alt']};self.event('origin',origin=self.origin);self.guard=True;self.sat_since=None
  self.command(M.MAV_CMD_COMPONENT_ARM_DISARM,[1]);self.wait('armed',lambda:self.state.get('armed'),timeout=20);self.event('takeoff_trigger',target_height_m=5)
  self.command(M.MAV_CMD_NAV_TAKEOFF,[0,0,0,float('nan'),float('nan'),float('nan'),self.origin['alt']+5]);target=self.origin['z']-5
  settled=lambda:abs(self.state.get('z',999)-target)<.2 and abs(self.state.get('vz',999))<.2
  self.stable_sim('target_height_settled',settled,3,180);self.mode(4,3);self.hover_guard=True;self.event('baseline_start',target_z=target)
  self.stable_sim('baseline_end',settled,10,180)
  self.phase('H25_transition',lambda:self.morph_to('H:25'));self.capture_matrix('H25');self.phase('H25_arm',self.arm_demo)
  self.phase('H25_return_X',lambda:self.morph_to('X'))
  self.phase('V25_transition',lambda:self.morph_to('V:25'));self.capture_matrix('V25');self.phase('V25_arm',self.arm_demo)
  self.phase('V25_return_X',lambda:self.morph_to('X'))
  self.phase('coupled_H',lambda:self.simultaneous('H','sweep'))
  self.phase('coupled_V',lambda:self.simultaneous('V','circles'))
  self.event('recovery_start');self.stable_sim('recovery_end',settled,10,180);self.capture_matrix('X-final')
  self.hover_guard=False;self.event('land_trigger');self.mode(4,6);self.wait('ground_detected',lambda:self.state.get('landed')==1,timeout=180);self.wait('auto_disarmed',lambda:not self.state.get('armed'),timeout=60);self.guard=False;self.snapshot('after');self.event('test_complete')
 def abort(self,error):
  self.guard=False;self.hover_guard=False;self.event('test_failed',error=str(error));self.arm_command('hold');self.morph_command('HOLD')
  if self.state.get('armed'):
   try:
    self.event('abort_land_requested');self.mode(4,6);self.wait('abort_ground_detected',lambda:self.state.get('landed')==1,timeout=60);self.wait('abort_disarmed',lambda:not self.state.get('armed'),timeout=20)
   except Exception as e:
    self.event('abort_landing_incomplete',error=str(e));subprocess.run(['docker','exec','-e','GZ_PARTITION=a_coupled','px4-rtl-land-harmonic','gz','service','-s','/world/a_coupled_world/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True);self.event('simulation_paused')
passed=False;f=None;error=None
try:
 f=Flight(name);f.run();passed=True
except Exception as e:
 error=str(e)
 if f is not None:f.abort(e)
 else:print(error,flush=True)
finally:
 time.sleep(1)
 files=list((D/'runtime/log').rglob('*.ulg')) if (D/'runtime/log').exists() else []
 u=max(files,key=lambda p:p.stat().st_mtime) if files else None
 if u:
  shutil.copy2(u,D/'logs'/f'{name}.ulg');ulog={'file':f'logs/{name}.ulg','bytes':u.stat().st_size,'sha256':hashlib.sha256(u.read_bytes()).hexdigest()}
 else:ulog=None
 (D/'outcome.json').write_text(json.dumps({'passed_execution':passed,'error':error,'ulog':ulog,'events':f.events if f else []},indent=2))
 if f:f.file.close();f.raw.close();f.armfile.close();f.morphfile.close();f.link.close()
if not passed:raise SystemExit(1)
