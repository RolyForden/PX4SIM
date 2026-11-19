"""ROS 2 Cartesian commands -> CAD IK -> native Gazebo joint controller."""
from pathlib import Path
import socket,struct,json,time,re,math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from kinematics import HOME,inverse,forward,profile
R=Path(__file__).resolve().parent
class Arm(Node):
 def __init__(self):
  super().__init__('a_delta_native_control');self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',14841));self.sock.setblocking(False)
  self.pub=self.create_publisher(String,'/a_uav/arm_native/status',10);self.jp=self.create_publisher(JointState,'/a_uav/arm_native/joint_states',10);self.pp=self.create_publisher(Point,'/a_uav/arm_native/position',10)
  self.create_subscription(String,'/a_uav/arm_native/command',self.command,10);self.create_subscription(Point,'/a_uav/arm_native/target',self.target,10)
  self.latest=None;self.received=0.;self.plan=[];self.index=0;self.start=None;self.goal=HOME.copy();self.reference=HOME.copy();self.q=np.zeros(3);self.command_id=0;self.completed_id=0;self.error=None;self.absolute=True;self.active_command='home';self.sim_previous=None
  (R/'results').mkdir(exist_ok=True);self.log=(R/'results/session.jsonl').open('w');self.last_log=-1;self.create_timer(.02,self.tick)
 def line(self,a,b,speed=.015):
  a=np.array(a);b=np.array(b);L=np.linalg.norm(b-a);duration=profile(0,L,speed)[1]
  return {'a':a,'b':b,'length':L,'duration':duration,'speed':speed,'kind':'line'}
 def circle(self,a,center,cw=False,speed=.015):
  a=np.array(a);center=np.array(center);r=np.linalg.norm(a[:2]-center[:2])
  if not .0005<=r<=.03:raise ValueError('圆半径应在 0.5–30 mm 之间')
  L=2*math.pi*r
  return {'a':a,'center':center,'radius':r,'angle':math.atan2(a[1]-center[1],a[0]-center[0]),'length':L,'duration':profile(0,L,speed)[1],'speed':speed,'cw':cw,'kind':'circle','b':a.copy()}
 def sample(self,p,t):
  u,_=profile(t,p['length'],p['speed'])
  if p['kind']=='line':return p['a']+(p['b']-p['a'])*u
  th=p['angle']+(-1 if p['cw'] else 1)*2*math.pi*u
  return p['center']+np.array([p['radius']*math.cos(th),p['radius']*math.sin(th),0])
 def begin(self,plans,name):
  if not self.latest or time.monotonic()-self.received>.5:raise ValueError('尚未收到 Gazebo 实际状态')
  if self.plan:raise ValueError('机械臂正在运动，请等待完成')
  old=np.array(self.latest['q_rad'])
  for p in plans:
   for t in np.linspace(0,p['duration'],max(2,int(p['duration']/.02)+1)):old=inverse(self.sample(p,t),old)
  self.plan=plans;self.index=0;self.start=self.latest['sim_s'];self.command_id+=1;self.error=None;self.active_command=name
 def target(self,msg):
  try:self.begin([self.line(self.latest['position_m'] if self.latest else HOME,[msg.x,msg.y,msg.z])],'target')
  except (ValueError,TypeError) as e:self.reject(e)
 def reject(self,e):self.error=str(e);self.get_logger().warning(str(e))
 def command(self,msg):
  text=msg.data.strip()
  try:
   current=np.array(self.latest['position_m']) if self.latest else HOME.copy()
   if text.lower()=='demo':
    c=HOME+np.array([0,0,-.015]);a=c+np.array([.015,0,0]);down=HOME+np.array([0,0,-.025]);plans=[self.line(current,HOME),self.line(HOME,down),self.line(down,HOME),self.line(HOME,HOME+[.015,0,0]),self.line(HOME+[.015,0,0],HOME),self.line(HOME,a),self.circle(a,c),self.line(a,HOME)]
   elif text.lower() in ['home','g28']:plans=[self.line(current,HOME)]
   elif text.upper() in ['G90','G91']:
    self.absolute=text.upper()=='G90';return
   else:
    values={k:float(v) for k,v in re.findall(r'([GMXYZFIJ])\s*([-+]?\d*\.?\d+)',text.upper())}
    if values.get('G') not in [0,1,2,3]:raise ValueError('支持 home、demo、G0/G1、整圆 G2/G3、G28、G90/G91')
    speed=values.get('F',900)/60000
    if not 0<speed<=.03:raise ValueError('F 范围：0 < F ≤ 1800 mm/min')
    goal=current.copy()
    for i,k in enumerate('XYZ'):
     if k in values:goal[i]=values[k]/1000+(0 if self.absolute else current[i])
    if values['G'] in [0,1]:plans=[self.line(current,goal,speed)]
    else:
     if 'I' not in values or 'J' not in values or np.linalg.norm(goal-current)>1e-6:raise ValueError('本轮 G2/G3 支持 I/J 整圆，终点保持起点')
     center=current+[values['I']/1000,values['J']/1000,0];plans=[self.circle(current,center,values['G']==2,speed)]
   self.begin(plans,text)
  except (ValueError,TypeError) as e:self.reject(e)
 def tick(self):
  try:
   while True:
    data,_=self.sock.recvfrom(8192);self.latest=json.loads(data);self.received=time.monotonic()
  except BlockingIOError:pass
  if not self.latest:return
  fresh=time.monotonic()-self.received<.5
  if fresh:
   if self.plan:
    plan=self.plan[self.index];elapsed=max(0,self.latest['sim_s']-self.start);self.reference=self.sample(plan,elapsed)
    self.q=inverse(self.reference,self.q)
    # Finish only after the actual pose has settled at the segment end.
    if elapsed>=plan['duration'] and np.linalg.norm(np.array(self.latest['position_m'])-plan['b'])<.0005:
     self.index+=1
     if self.index==len(self.plan):self.plan=[];self.completed_id=self.command_id;self.reference=plan['b'].copy()
     else:self.start=self.latest['sim_s']
    elif elapsed>plan['duration']+5:
     self.error='实际末端未在 5 s 内到达目标';self.plan=[];self.q=np.array(self.latest['q_rad'])
   self.sock.sendto(struct.pack('<I3f',0x44454c54,*self.q),('127.0.0.1',14840))
  elif self.plan:
   self.error='Gazebo 实际状态超时，轨迹已停止';self.plan=[]
  state={**self.latest,'ready':fresh,'busy':bool(self.plan),'command':self.active_command,'command_id':self.command_id,'completed_id':self.completed_id,'segment':self.index,'reference_m':self.reference.tolist(),'tracking_error_m':float(np.linalg.norm(np.array(self.latest['position_m'])-self.reference)),'error':self.error,'model':'native ball + fixed-proxy closed chain; gravity dynamics','platform_fk_error_m':float(np.linalg.norm(forward(self.latest['q_rad'])-np.array(self.latest['position_m'])))}
  m=String();m.data=json.dumps(state);self.pub.publish(m);
  if state['busy'] or state['sim_s']-self.last_log>=.2:
   self.log.write(m.data+'\n');self.log.flush();self.last_log=state['sim_s']
  j=JointState();j.header.stamp=self.get_clock().now().to_msg();j.name=[f'act_{i}' for i in range(3)];j.position=[float(x) for x in self.latest['q_rad']];self.jp.publish(j);p=Point();p.x,p.y,p.z=[float(x) for x in self.latest['position_m']];self.pp.publish(p)
def main():
 rclpy.init();node=Arm()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.log.close();node.sock.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
