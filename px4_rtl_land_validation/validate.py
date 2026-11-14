"""Local X500 SITL validation only; dedicated loopback MAVLink endpoint."""
import argparse, csv, datetime, json, math, pathlib, struct, subprocess, time
from pymavlink import mavutil

ROOT = pathlib.Path(__file__).resolve().parent
M = mavutil.mavlink

class Flight:
    def __init__(self, name):
        top = subprocess.check_output(['docker', 'top', 'px4-rtl-land-harmonic', '-eo', 'pid,args'], text=True)
        if '/work/PX4-Autopilot/build/px4_sitl_default/bin/px4' not in top:
            raise RuntimeError('Dedicated local SITL process not confirmed')
        self.name = name
        self.link = mavutil.mavlink_connection('udpin:127.0.0.1:14650', source_system=250)
        self.state = {}
        self.params = {}
        self.start = time.monotonic()
        self.last_hb = self.last_row = 0
        self.events = []
        self.file = (ROOT/'logs'/f'{name}_telemetry.csv').open('w')
        self.fields = ['elapsed','boot_ms','mode','armed','landed','x','y','z','vx','vy','vz','lat','lon','alt','relative_alt','roll','pitch','yaw']
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()
        self.raw = (ROOT/'logs'/f'{name}_messages.jsonl').open('w')
        self.pump(3)
        if self.state.get('autopilot') != M.MAV_AUTOPILOT_PX4:
            raise RuntimeError('PX4 heartbeat absent')
        for msg_id, hz in [(32,20),(33,10),(30,20),(245,5),(242,1)]:
            self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL,[msg_id,1e6/hz],wait=False)
        self.pump(2)

    def event(self, what, **kw):
        d = {'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'elapsed':round(time.monotonic()-self.start,3),'event':what,**self.state,**kw}
        self.events.append(d)
        (ROOT/'logs'/f'{self.name}_events.json').write_text(json.dumps(self.events,indent=2))
        print(json.dumps(d),flush=True)

    def pump(self, duration=0.1, setpoint=None):
        end=time.monotonic()+duration
        last_sp=0
        while time.monotonic()<end:
            now=time.monotonic()
            if now-self.last_hb>1:
                self.link.mav.heartbeat_send(M.MAV_TYPE_GCS,M.MAV_AUTOPILOT_INVALID,0,0,M.MAV_STATE_ACTIVE)
                self.last_hb=now
            if setpoint is not None and now-last_sp>0.08:
                self.link.mav.set_position_target_local_ned_send(0,1,1,M.MAV_FRAME_LOCAL_NED,0b110111111000,*setpoint,0,0,0,0,0,0,0,0)
                last_sp=now
            msg=self.link.recv_match(blocking=True,timeout=0.02)
            if msg and msg.get_srcSystem()==1:
                t=msg.get_type()
                if t=='HEARTBEAT':
                    self.state.update(autopilot=msg.autopilot,mode=msg.custom_mode,armed=bool(msg.base_mode&M.MAV_MODE_FLAG_SAFETY_ARMED))
                elif t=='LOCAL_POSITION_NED':
                    self.state.update({k:getattr(msg,k) for k in ['x','y','z','vx','vy','vz']})
                    self.state['boot_ms']=msg.time_boot_ms
                elif t=='GLOBAL_POSITION_INT':
                    self.state.update(lat=msg.lat/1e7,lon=msg.lon/1e7,alt=msg.alt/1000,relative_alt=msg.relative_alt/1000)
                elif t=='ATTITUDE':
                    self.state.update({k:getattr(msg,k) for k in ['roll','pitch','yaw']})
                elif t=='EXTENDED_SYS_STATE': self.state['landed']=msg.landed_state
                elif t=='PARAM_VALUE':
                    # PX4 encodes integer parameters bytewise in PARAM_VALUE.float.
                    value=float(msg.param_value)
                    if msg.param_type==M.MAV_PARAM_TYPE_INT32:
                        value=struct.unpack('<i',struct.pack('<f',value))[0]
                    self.params[msg.param_id]=value
                if t in ['COMMAND_ACK','STATUSTEXT','HOME_POSITION','AUTOPILOT_VERSION']:
                    self.raw.write(json.dumps({'elapsed':now-self.start,**msg.to_dict()})+'\n'); self.raw.flush()
                    if t=='COMMAND_ACK': self.last_ack=msg
            if now-self.last_row>=0.1:
                self.writer.writerow({'elapsed':round(now-self.start,3),**{k:self.state.get(k,'') for k in self.fields[1:]}})
                self.file.flush(); self.last_row=now

    def command(self, cmd, params=(), wait=True):
        self.last_ack=None
        self.link.mav.command_long_send(1,1,cmd,0,*(list(params)+[0]*(7-len(params))))
        if wait:
            end=time.monotonic()+6
            while time.monotonic()<end:
                self.pump()
                if self.last_ack and self.last_ack.command==cmd:
                    self.event('command_ack',command=cmd,result=self.last_ack.result)
                    if self.last_ack.result not in (M.MAV_RESULT_ACCEPTED,M.MAV_RESULT_IN_PROGRESS):
                        raise RuntimeError(f'Command {cmd} rejected: {self.last_ack.result}')
                    return
            raise TimeoutError(f'No ACK for {cmd}')

    def wait(self, label, predicate, timeout=90, setpoint=None, stable=0):
        end=time.monotonic()+timeout; since=None
        while time.monotonic()<end:
            self.pump(0.1,setpoint)
            if predicate():
                since=since or time.monotonic()
                if time.monotonic()-since>=stable:
                    self.event(label); return
            else: since=None
        raise TimeoutError(label+': '+str(self.state))

    def snapshot(self, tag):
        self.link.mav.param_request_list_send(1,1)
        self.pump(12)
        (ROOT/'parameters').mkdir(exist_ok=True)
        (ROOT/'parameters'/f'{self.name}_{tag}.json').write_text(json.dumps(self.params,indent=2,sort_keys=True))
        for key in ['SYS_AUTOSTART','MPC_LAND_SPEED','RTL_RETURN_ALT','RTL_DESCEND_ALT','RTL_LAND_DELAY']:
            if key not in self.params: raise RuntimeError('Missing parameter '+key)
        if self.params['SYS_AUTOSTART']!=4001: raise RuntimeError('Not X500 SITL')
        self.event('parameters',values={k:self.params[k] for k in ['MPC_LAND_SPEED','RTL_RETURN_ALT','RTL_DESCEND_ALT','RTL_LAND_DELAY']})

    def set_param(self, name, value):
        self.link.mav.param_set_send(1,1,name.encode(),value,M.MAV_PARAM_TYPE_REAL32)
        self.pump(2)
        if abs(self.params.get(name,-1)-value)>1e-4: raise RuntimeError('Parameter readback failed')
        self.event('parameter_changed',parameter=name,value=value)

    def mode(self, main, sub=0, setpoint=None):
        if setpoint is None:
            self.command(M.MAV_CMD_DO_SET_MODE,[M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,main,sub])
        else:
            self.command(M.MAV_CMD_DO_SET_MODE,[M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,main,sub],wait=False)
        self.wait('mode_confirmed',lambda:self.state.get('mode')==((main<<16)|(sub<<24)),timeout=10,setpoint=setpoint)

    def run(self, kind, speed=None):
        self.snapshot('before')
        self.wait('ready_ground',lambda:self.state.get('landed')==M.MAV_LANDED_STATE_ON_GROUND and not self.state.get('armed'),timeout=60)
        original=self.params['MPC_LAND_SPEED']
        if speed is not None: self.set_param('MPC_LAND_SPEED',speed)
        self.pump(3)
        origin={k:self.state[k] for k in ['x','y','z','alt']}
        self.event('origin',origin=origin)
        self.command(M.MAV_CMD_COMPONENT_ARM_DISARM,[1])
        self.wait('armed',lambda:self.state.get('armed'),timeout=10)
        self.command(M.MAV_CMD_NAV_TAKEOFF,[0,0,0,float('nan'),float('nan'),float('nan'),origin['alt']+10])
        self.wait('hover_confirmed',lambda:abs(self.state.get('z',999)-(origin['z']-10))<0.7 and abs(self.state.get('vz',999))<0.25,timeout=90,stable=5)
        if kind=='rtl':
            target=(origin['x']+40,origin['y'],origin['z']-10)
            self.pump(2,target)
            self.mode(6,setpoint=target)
            self.wait('away_hover_confirmed',lambda:math.hypot(self.state.get('x',999)-target[0],self.state.get('y',999)-target[1])<1 and abs(self.state.get('vz',999))<0.25,timeout=90,setpoint=target,stable=5)
            self.mode(4,3)
            self.pump(3)
            self.event('rtl_trigger')
            self.mode(4,5)
            self.wait('returned_home_horizontal',lambda:math.hypot(self.state['x']-origin['x'],self.state['y']-origin['y'])<2,timeout=180)
        else:
            self.event('land_trigger')
            self.mode(4,6)
        self.wait('ground_detected',lambda:self.state.get('landed')==M.MAV_LANDED_STATE_ON_GROUND,timeout=240)
        self.wait('auto_disarmed',lambda:not self.state.get('armed'),timeout=30)
        self.pump(3)
        if speed is not None: self.set_param('MPC_LAND_SPEED',original)
        self.snapshot('after')
        self.event('test_complete')

if __name__=='__main__':
    a=argparse.ArgumentParser(); a.add_argument('kind',choices=['rtl','land']); a.add_argument('--name',required=True); a.add_argument('--speed',type=float)
    args=a.parse_args(); f=Flight(args.name)
    try: f.run(args.kind,args.speed)
    except Exception as e:
        f.event('test_failed',error=str(e))
        if f.state.get('armed'):
            try: f.command(M.MAV_CMD_NAV_LAND,[0,0,0,float('nan'),float('nan'),float('nan'),float('nan')]); f.wait('recovery_disarmed',lambda:not f.state.get('armed'),timeout=240)
            except Exception as recovery: f.event('recovery_failed',error=str(recovery))
        raise
