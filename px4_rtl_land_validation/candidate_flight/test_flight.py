"""Authorized candidate-only local SITL takeoff/hover/Land check; no real hardware."""
import sys,pathlib,time,json,math,subprocess,datetime,shutil
ROOT=pathlib.Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parent))
import validate
from pymavlink import mavutil
validate.ROOT=ROOT
for folder in ['logs','parameters']:(ROOT/folder).mkdir(exist_ok=True)
class CandidateFlight(validate.Flight):
 def pump(self,duration=.1,setpoint=None):
  super().pump(duration,setpoint)
  if getattr(self,'monitor_flight',False) and self.state.get('armed'):
   s=self.state;o=self.origin
   if max(abs(s.get('roll',0)),abs(s.get('pitch',0)))>math.radians(35) or math.hypot(s.get('x',o['x'])-o['x'],s.get('y',o['y'])-o['y'])>8 or abs(s.get('z',o['z'])-o['z'])>15:
    self.monitor_flight=False
    raise RuntimeError('Candidate flight exceeded attitude/position test envelope')
 def snapshot(self,tag):
  self.link.mav.param_request_list_send(1,1);self.pump(12)
  assert self.params.get('SYS_AUTOSTART')==4999,'Candidate airframe ID not confirmed'
  (ROOT/'parameters'/f'{self.name}_{tag}.json').write_text(json.dumps(self.params,indent=2,sort_keys=True))
  self.event('parameters',airframe=self.params['SYS_AUTOSTART'])
 def run_candidate(self):
  self.snapshot('before');self.wait('ready_ground',lambda:self.state.get('landed')==1 and not self.state.get('armed'),timeout=60)
  origin={k:self.state[k] for k in ['x','y','z','alt']};self.event('origin',origin=origin);self.origin=origin;self.monitor_flight=True
  self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,[1]);self.wait('armed',lambda:self.state.get('armed'),timeout=10)
  self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,[0,0,0,float('nan'),float('nan'),float('nan'),origin['alt']+5])
  def hover():
   s=self.state
   return abs(s.get('z',999)-(origin['z']-5))<.5 and abs(s.get('vz',999))<.3 and math.hypot(s.get('x',999)-origin['x'],s.get('y',999)-origin['y'])<1 and max(abs(s.get('roll',9)),abs(s.get('pitch',9)))<math.radians(10)
  self.wait('hover_confirmed',hover,timeout=90,stable=8);self.event('land_trigger');self.mode(4,6)
  self.wait('ground_detected',lambda:self.state.get('landed')==1,timeout=120);self.wait('auto_disarmed',lambda:not self.state.get('armed'),timeout=30)
  self.snapshot('after');self.event('test_complete')
if __name__=='__main__':
 top=subprocess.check_output(['docker','top','px4-rtl-land-harmonic','-eo','pid,args'],text=True)
 assert '/work/candidate_flight/startup.sh' in top and '/work/candidate_flight/runtime' in top,'Isolated candidate process not confirmed'
 warm=mavutil.mavlink_connection('udpin:127.0.0.1:14650')
 assert warm.wait_heartbeat(timeout=45) is not None,'Candidate startup heartbeat timeout'
 warm.close()
 name='rigid_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
 (ROOT/'logs'/f'{name}_model_config.json').write_text((ROOT/'flight_config.json').read_text())
 shutil.copyfile(ROOT/'models/candidate_rigid/model.sdf',ROOT/'logs'/f'{name}_model.sdf')
 shutil.copyfile(ROOT/'candidate_world.sdf',ROOT/'logs'/f'{name}_world.sdf')
 f=CandidateFlight(name)
 try:f.run_candidate()
 except Exception as e:
  f.monitor_flight=False;f.event('test_failed',error=str(e))
  if f.state.get('armed'):
   try:f.command(mavutil.mavlink.MAV_CMD_NAV_LAND,[0,0,0,float('nan'),float('nan'),float('nan'),float('nan')]);f.wait('recovery_disarmed',lambda:not f.state.get('armed'),timeout=150)
   except Exception as err:f.event('recovery_failed',error=str(err))
  raise
 finally:
  paths=list((ROOT/'runtime/log').rglob('*.ulg'))
  if paths:
   source=max(paths,key=lambda p:p.stat().st_mtime);shutil.copyfile(source,ROOT/'logs'/f'{name}.ulg')
  (ROOT/'last_run.json').write_text(json.dumps({'name':name,'events':f.events,'status':'passed' if f.events[-1]['event']=='test_complete' else 'failed'},indent=2))
