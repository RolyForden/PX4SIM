"""CAD-based Delta IK/FK, in the aircraft FLU frame (metres/radians)."""
from pathlib import Path
import json,math
import numpy as np
CONFIG=json.loads((Path(__file__).parent/'config.json').read_text())
HOME=np.array(CONFIG['home_m'])
S=np.array([g['shoulder_m'] for g in CONFIG['legs']]);A=np.array([g['axis'] for g in CONFIG['legs']]);E=np.array([np.mean(g['elbows_home_m'],axis=0) for g in CONFIG['legs']]);O=np.array([np.mean(g['platform_offsets_m'],axis=0) for g in CONFIG['legs']])
LIMIT=.6

def rotation(a,q):
 K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
 return np.eye(3)+math.sin(q)*K+(1-math.cos(q))*(K@K)

def elbows(q):return np.array([s+rotation(a,t)@(e-s) for s,a,e,t in zip(S,A,E,q)])

def inverse(p,previous=None):
 p=np.asarray(p,float)
 if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('目标必须是三个有限坐标')
 if np.linalg.norm(p-HOME)>.065:raise ValueError('目标超出本轮台架的 65 mm 范围')
 previous=np.zeros(3) if previous is None else np.asarray(previous);q=[]
 for s,a,e,o,old in zip(S,A,E,O,previous):
  d=e-s;parallel=a*np.dot(a,d);u=d-parallel;w=np.cross(a,u);v=p+o-s-parallel
  aa=np.dot(v,u);bb=np.dot(v,w);cc=(np.dot(v,v)+np.dot(u,u)-.2**2)/2;rho=math.hypot(aa,bb)
  if rho<1e-12 or abs(cc)>rho+1e-12:raise ValueError('目标不可达')
  phase=math.atan2(bb,aa);angle=math.acos(np.clip(cc/rho,-1,1));candidates=[(phase+sign*angle+math.pi)%(2*math.pi)-math.pi for sign in [-1,1]];valid=[x for x in candidates if abs(x)<=LIMIT]
  if not valid:raise ValueError('目标超出主动关节限位 ±0.6 rad')
  q.append(min(valid,key=lambda x:abs(x-old)))
 return np.array(q)

def forward(q,guess=HOME):
 p=np.array(guess,float).copy();centers=elbows(q)-O
 for _ in range(30):
  d=p-centers;f=np.sum(d*d,axis=1)-.04
  if np.max(np.abs(f))<1e-13:return p
  step=np.linalg.solve(2*d,f)
  if not np.isfinite(step).all() or np.linalg.norm(step)>.1:raise ValueError('正解未收敛')
  p-=step
 raise ValueError('正解未收敛')

def profile(t,L,v=.015,a=.03):
 """Same triangular/trapezoidal interpolation as DeltaRobot.c, using SI units."""
 if L<=1e-12:return 1.,0.
 ta=min(v/a,math.sqrt(L/a));T=2*ta+max(0,L-v*v/a)/v
 if t>=T:return 1.,T
 if t<=0:return 0.,T
 if t<ta:s=.5*a*t*t
 elif t<T-ta:s=.5*a*ta*ta+v*(t-ta)
 else:s=L-.5*a*(T-t)**2
 return s/L,T
