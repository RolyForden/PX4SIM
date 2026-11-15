"""ROS commands -> smooth joint targets; measured joints -> PX4 rotor geometry."""
import json,math,socket,struct,time,pathlib
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
R=pathlib.Path(__file__).resolve().parent
from geometry import CONFIG,geometry
MAGIC=0x41555631
NAMES=[f'{k}_{i}' for i in range(4) for k in ('yaw','pitch')]
class Control(Node):
 def __init__(self):
  super().__init__('a_air_control');self.send=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  self.feed=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.feed.bind(('127.0.0.1',14783));self.feed.setblocking(False)
  self.ack=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.ack.bind(('127.0.0.1',14781));self.ack.setblocking(False)
  self.q=None;self.last_feed=0;self.last_sim_advance=0;self.sim_s=-1;self.last_ack=0;self.ack_seq=0;self.seq=0;self.last_geometry=0;self.start=None;self.target=np.zeros(8);self.fault=None;self.bad_since=None;self.prev_status=0
  self.joint=self.create_publisher(JointState,'/a_uav/joint_states',10);self.status=self.create_publisher(String,'/a_uav/morph/status',10);self.matrix=self.create_publisher(String,'/a_uav/allocation/matrix',10)
  self.create_subscription(String,'/a_uav/morph/command',self.command,10);self.create_timer(.02,self.tick)
  logdir=R/'sessions';logdir.mkdir(exist_ok=True);self.log=(logdir/(time.strftime('%Y%m%d_%H%M%S')+'.jsonl')).open('w')
  self.get_logger().info('A flight ROS ready: X, H:25, V:25; /a_uav/morph/status reports PX4 matrix acknowledgement')
 def command(self,m):
  try:
   mode,_,value=m.data.strip().upper().partition(':');assert mode in ('X','H','V');deg=0 if mode=='X' else float(value or 25);assert 0<=deg<=25
   assert self.q is not None and time.monotonic()-self.last_ack<.6,'PX4 geometry acknowledgement unavailable'
   assert self.start is None,'previous transition in progress'
   goal=np.zeros(8)
   for i,a in enumerate(CONFIG['arms']):
    if mode=='H':goal[2*i]=math.copysign(math.radians(deg),a['H']['yaw_rad'])
    if mode=='V':goal[2*i+1]=math.radians(deg)
   assert mode=='X' or max(abs(self.q))<math.radians(2),'return to X before changing branch'
   self.origin=self.q.copy();self.target=goal;self.duration=max(5.,1.875*float(max(abs(goal-self.origin)))/math.radians(5));self.start=time.monotonic();self.fault=None
   self.get_logger().info(f'{mode} {deg:g}deg transition {self.duration:.1f}s')
  except Exception as e:self.get_logger().error('Command rejected: '+str(e))
 def tick(self):
  now=time.monotonic()
  while True:
   try:
    data=self.feed.recv(4096)
    if len(data)==48:
     magic,seq,*v=struct.unpack('<II8fd',data)
     if magic==MAGIC:
      self.q=np.array(v[:8]);self.last_feed=now
      if v[8]>self.sim_s:self.sim_s=v[8];self.last_sim_advance=now
   except BlockingIOError:break
  while True:
   try:
    data=self.ack.recv(4096)
    if len(data)==8:
     magic,seq=struct.unpack('<II',data)
     if magic==MAGIC:self.last_ack=now;self.ack_seq=seq
   except BlockingIOError:break
  fresh=self.q is not None and now-self.last_feed<.25 and now-self.last_sim_advance<.5
  healthy=fresh and now-self.last_ack<.6
  if self.start is not None and not healthy:
   self.start=None;self.target=self.q.copy();self.fault='feedback/allocator timeout; transition held'
  if self.q is not None:
   if self.start is not None:
    u=min(1.,(now-self.start)/self.duration);w=10*u**3-15*u**4+6*u**5;target=self.origin+w*(self.target-self.origin)
    if u>=1 and max(abs(self.q-self.target))<math.radians(1):self.start=None
   else:target=self.target
   tracking_error=float(max(abs(self.q-target)))
   if tracking_error>math.radians(5):
    self.bad_since=self.bad_since or now
    if now-self.bad_since>.5:
     self.start=None;self.target=self.q.copy();target=self.target;self.fault='joint tracking error >5deg';healthy=False
   else:self.bad_since=None
   self.send.sendto(struct.pack('<I8f',MAGIC,*target),('127.0.0.1',14782))
  if fresh and now-self.last_geometry>.045:
   self.last_geometry=now;self.seq+=1;c,r,d,B=geometry(self.q)
   self.send.sendto(struct.pack('<II24f',MAGIC,self.seq,*np.column_stack((r,d)).reshape(-1)),('127.0.0.1',14780))
   record={'wall_s':time.time(),'sim_s':self.sim_s,'sequence':self.seq,'px4_ack':self.ack_seq,'q_deg':np.degrees(self.q).tolist(),'target_deg':np.degrees(target).tolist(),'tracking_error_deg':math.degrees(tracking_error),'com_flu_m':c.tolist(),'B':B.tolist(),'feedback_age_ms':1000*(now-self.last_feed),'allocator_ack_age_ms':1000*(now-self.last_ack) if self.last_ack else None}
   self.log.write(json.dumps(record)+'\n');self.log.flush();self.matrix.publish(String(data=json.dumps(record)))
   self.joint.publish(JointState(header=self._header(),name=NAMES,position=self.q.tolist()))
  if now-self.prev_status>.5:
   self.prev_status=now;msg={'ready':healthy and self.fault is None,'transition':self.start is not None,'fault':self.fault,'joint_deg':None if self.q is None else np.degrees(self.q).round(2).tolist(),'px4_matrix_ack':self.ack_seq,'command_sequence':self.seq};self.status.publish(String(data=json.dumps(msg)));self.get_logger().info(json.dumps(msg))
 def _header(self):
  from std_msgs.msg import Header
  return Header(stamp=self.get_clock().now().to_msg(),frame_id='base_link')
def main():
 rclpy.init();n=Control()
 try:rclpy.spin(n)
 finally:n.log.close();n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
