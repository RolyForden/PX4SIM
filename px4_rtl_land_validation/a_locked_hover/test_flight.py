"""2 m/20 simulated-second locked-arm hover, guarded landing and native ULog."""
from pathlib import Path
import json,time,math,sys,shutil,subprocess,hashlib,signal
import flight_link as m
from pymavlink import mavutil
R=Path(__file__).resolve().parent;name=sys.argv[1];D=R/'runs'/name;m.ROOT=D;M=mavutil.mavlink
class Flight(m.Flight):
 def pump(self,duration=.1,setpoint=None):
  end=time.monotonic()+duration
  while time.monotonic()<end:
   self.link.mav.manual_control_send(1,0,0,500,0,0);super().pump(min(.04,max(0,end-time.monotonic())),setpoint)
   if getattr(self,'guard',False) and self.state.get('armed'):
    s=self.state;o=self.origin
    if max(abs(s.get('roll',0)),abs(s.get('pitch',0)))>math.radians(20):raise RuntimeError('Abort: roll/pitch >20deg')
    if math.hypot(s.get('x',o['x'])-o['x'],s.get('y',o['y'])-o['y'])>2:raise RuntimeError('Abort: horizontal offset >2m')
    h=o['z']-s.get('z',o['z']);vz=s.get('vz',0)
    if h>1 and vz>1.5:raise RuntimeError('Abort: rapid descent above 1m')
    if h>3.5:raise RuntimeError('Abort: unexpected climb above 3.5m')
    if getattr(self,'hover_guard',False):
     outputs=s.get('motor_outputs',[]);upper=990 if outputs and max(outputs)>2 else .98;saturated=bool(outputs) and max(outputs)>=upper
     self.sat_since=(self.sat_since or s.get('boot_ms',0)) if saturated else None
     if self.sat_since and s.get('boot_ms',0)-self.sat_since>1000:raise RuntimeError('Abort: motor output persistently upper-saturated')
 def snapshot(self,tag):
  self.link.mav.param_request_list_send(1,1);self.pump(6)
  if self.params.get('SYS_AUTOSTART')!=4995:raise RuntimeError('Wrong airframe')
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
 def run(self):
  self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL,[375,100000],wait=False);self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL,[230,200000],wait=False)
  self.snapshot('before');self.wait('ready_ground',lambda:self.state.get('landed')==1 and not self.state.get('armed') and all(k in self.state for k in ['x','y','z','alt']),timeout=30)
  self.origin={k:self.state[k] for k in ['x','y','z','alt']};self.event('origin',origin=self.origin);self.guard=True;self.sat_since=None
  self.command(M.MAV_CMD_COMPONENT_ARM_DISARM,[1]);self.wait('armed',lambda:self.state.get('armed'),timeout=10);self.event('takeoff_trigger',target_height_m=2)
  self.command(M.MAV_CMD_NAV_TAKEOFF,[0,0,0,float('nan'),float('nan'),float('nan'),self.origin['alt']+2]);target=self.origin['z']-2
  settled=lambda:abs(self.state.get('z',999)-target)<.2 and abs(self.state.get('vz',999))<.2
  self.stable_sim('target_height_settled',settled,3,60);self.mode(4,3);self.hover_guard=True;self.event('hover_start',target_z=target)
  self.stable_sim('hover_complete',settled,20,60);self.hover_guard=False;self.event('land_trigger');self.mode(4,6);self.wait('ground_detected',lambda:self.state.get('landed')==1,timeout=60);self.wait('auto_disarmed',lambda:not self.state.get('armed'),timeout=20);self.guard=False;self.snapshot('after');self.event('test_complete')
 def abort(self,error):
  self.guard=False;self.hover_guard=False;self.event('test_failed',error=str(error))
  if self.state.get('armed'):
   try:
    self.event('abort_land_requested');self.mode(4,6);self.wait('abort_ground_detected',lambda:self.state.get('landed')==1,timeout=60);self.wait('abort_disarmed',lambda:not self.state.get('armed'),timeout=20)
   except Exception as e:
    self.event('abort_landing_incomplete',error=str(e));subprocess.run(['docker','exec','-e','GZ_PARTITION=a_locked_hover','px4-rtl-land-harmonic','gz','service','-s','/world/a_locked_world/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True);self.event('simulation_paused')
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
 if f:f.file.close();f.raw.close();f.link.close()
if not passed:raise SystemExit(1)
