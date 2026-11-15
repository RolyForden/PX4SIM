"""Dedicated a_air SITL regression with attitude/joint-envelope abort."""
import pathlib,sys,json,math,time,datetime,subprocess,shutil,argparse,signal
R=pathlib.Path(__file__).resolve().parent;P=R.parents[1]
import mavlink_flight as m
m.ROOT=R
for d in ['logs','parameters']:(R/d).mkdir(exist_ok=True)
from pymavlink import mavutil
class Flight(m.Flight):
 def pump(self,duration=.1,setpoint=None):
  end=time.monotonic()+duration
  while time.monotonic()<end:
   # Automated tests own the manual-input stream; centered sticks, 20Hz.
   self.link.mav.manual_control_send(1,0,0,500,0,0)
   self.pump_once(min(.05,max(0,end-time.monotonic())),setpoint)
 def pump_once(self,duration=.05,setpoint=None):
  super().pump(duration,setpoint)
  if getattr(self,'guard',False) and self.state.get('armed'):
   o=self.origin;s=self.state
   if max(abs(s.get('roll',0)),abs(s.get('pitch',0)))>math.radians(20) or math.hypot(s.get('x',o['x'])-o['x'],s.get('y',o['y'])-o['y'])>5:raise RuntimeError('Attitude/position envelope')
   p=max((R/'sessions').glob('*.jsonl'),key=lambda p:p.stat().st_mtime)
   with p.open('rb') as f:f.seek(max(0,p.stat().st_size-15000));lines=f.read().decode().splitlines()
   for line in reversed(lines):
    try:r=json.loads(line);break
    except json.JSONDecodeError:pass
   else:raise RuntimeError('Joint log absent')
   if time.time()-r['wall_s']>.8:raise RuntimeError('Joint feedback stale')
   self.latest_joint=r
   if r.get('tracking_error_deg',0)>8:raise RuntimeError('Joint tracking error >8deg')
   if getattr(self,'x_only',True) and max(abs(x) for x in r['q_deg'])>5:raise RuntimeError('X joint deviation >5deg')
 def snapshot(self,tag):
  self.link.mav.param_request_list_send(1,1);self.pump(6)
  assert self.params.get('SYS_AUTOSTART')==4996
  (R/'parameters'/f'{self.name}_{tag}.json').write_text(json.dumps(self.params,indent=2));self.event('parameters',airframe=4996)
 def run(self):
  self.snapshot('before')
  self.wait('ready_ground',lambda:self.state.get('landed')==1 and not self.state.get('armed'),timeout=30)
  self.origin={k:self.state[k] for k in ['x','y','z','alt']};self.event('origin',origin=self.origin);self.guard=True
  self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,[1]);self.wait('armed',lambda:self.state.get('armed'),timeout=10)
  self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,[0,0,0,float('nan'),float('nan'),float('nan'),self.origin['alt']+5])
  self.wait('hover_confirmed',lambda:abs(self.state.get('z',999)-(self.origin['z']-5))<.4 and abs(self.state.get('vz',999))<.25,timeout=60,stable=8)
  if getattr(self,'sequence',[]):
   self.x_only=False
   for command in self.sequence:
    mode,_,angle=command.partition(':');deg=float(angle or 0);goal=[0.]*8
    cfg=json.loads((R.parent/'config/morph-config.json').read_text())
    for i,arm in enumerate(cfg['arms']):
     if mode=='H':goal[2*i]=math.copysign(deg,arm['H']['yaw_rad'])
     if mode=='V':goal[2*i+1]=deg
    self.event('morph_trigger',command=command)
    cmd=['bash','-c','source /opt/ros/humble/setup.bash && ros2 topic pub --once /a_uav/morph/command std_msgs/msg/String '+"'"+'{data: "'+command+'"}'+"'"]
    proc=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    while proc.poll() is None:self.pump(.1)
    if proc.returncode:raise RuntimeError('ROS command failed')
    def reached():
     return max(abs(a-b) for a,b in zip(self.latest_joint['q_deg'],goal))<1 and abs(self.state.get('z',999)-(self.origin['z']-5))<.5 and abs(self.state.get('vz',999))<.3
    self.wait('morph_reached_'+command,reached,timeout=35,stable=4)
  self.event('land_trigger');self.mode(4,6);self.wait('ground_detected',lambda:self.state.get('landed')==1,timeout=80);self.wait('auto_disarmed',lambda:not self.state.get('armed'),timeout=20);
  self.snapshot('after');self.event('test_complete')
if __name__=='__main__':
 def interrupted(signum,frame):raise RuntimeError('test interrupted by signal '+str(signum))
 signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
 parser=argparse.ArgumentParser();parser.add_argument('--sequence',nargs='*',default=[]);args=parser.parse_args()
 normalized=[]
 for command in args.sequence:
  mode,_,value=command.strip().upper().partition(':')
  if mode not in ('X','H','V'):parser.error('Supported shapes: X/H/V')
  angle=0 if mode=='X' else float(value or 25)
  if not 0<=angle<=25:parser.error('Angle range is 0..25deg')
  normalized.append('X' if mode=='X' else f'{mode}:{angle:g}')
 args.sequence=normalized
 name=('air_morph_' if args.sequence else 'air_x_')+datetime.datetime.now().strftime('%Y%m%d_%H%M%S');f=Flight(name);f.sequence=args.sequence;passed=False
 try:f.run();passed=True
 except Exception as e:
  f.guard=False;f.event('test_failed',error=str(e))
  if f.state.get('armed'):f.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,[0,21196])
  subprocess.run(['docker','exec','-e','GZ_PARTITION=a_air','px4-rtl-land-harmonic','gz','service','-s','/world/candidate_world/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','1000','--req','pause: true'],capture_output=True)
  raise
 finally:
  time.sleep(1)  # Allow automatic disarm logging to flush; keep logger available for later flights.
  u=max((R/'runtime/log').rglob('*.ulg'),key=lambda p:p.stat().st_mtime);shutil.copyfile(u,R/'logs'/f'{name}.ulg')
  (R/'last_test.json').write_text(json.dumps({'name':name,'passed':passed,'events':f.events},indent=2))
