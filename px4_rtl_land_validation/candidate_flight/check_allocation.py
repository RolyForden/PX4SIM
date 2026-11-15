"""Offline bounded thrust allocation; independent of PX4 flight allocator."""
import itertools,json,pathlib
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parent

def allocate(B,target,lower,upper,weights):
 """Enumerate active bounds (4 motors), solve weighted least squares on free inputs."""
 B=np.asarray(B,float);target=np.asarray(target,float);n=B.shape[1]
 lower=np.broadcast_to(lower,(n,));upper=np.broadcast_to(upper,(n,));weights=np.asarray(weights,float)
 if n>6:raise ValueError('Small offline solver only; not intended for realtime control')
 best=None
 for active in itertools.product([-1,0,1],repeat=n):
  active=np.array(active);fixed=active!=0;free=~fixed;t=np.where(active<0,lower,upper).copy()
  rhs=target-B[:,fixed]@t[fixed]
  if free.any():t[free]=np.linalg.lstsq(weights[:,None]*B[:,free],weights*rhs,rcond=None)[0]
  if np.any(t<lower-1e-9) or np.any(t>upper+1e-9):continue
  residual=B@t-target;score=np.linalg.norm(weights*residual)
  if best is None or score<best[0]-1e-12:best=(score,t.copy(),residual)
 if best is None:raise ValueError('Invalid thrust bounds')
 return {'thrust_N':best[1].tolist(),'residual':best[2].tolist(),'weighted_residual_norm':float(best[0])}

if __name__=='__main__':
 c=json.loads((ROOT/'flight_config.json').read_text());p=np.array([r['position_frd_m'] for r in c['rotors']]);d=np.array([r['axis_frd'] for r in c['rotors']]);km=np.array([r['km_m'] for r in c['rotors']])
 B=np.vstack([(np.cross(p,d)-km[:,None]*d).T,d.T]);maxT=c['motor_constant_N_per_rad_s2']*c['max_rad_s']**2
 # Explicit normalization: moments scaled by 0.2m reference length, forces by 1N.
 weights=np.array([5,5,5,1,1,1.]);target=np.array([0,0,0,0,0,-c['total_mass_kg']*9.8]);hover=allocate(B,target,0,maxT,weights)
 impossible=allocate(B,np.array([0,0,0,0,0,-8*maxT]),0,maxT,weights)
 assert hover['weighted_residual_norm']<1e-8 and min(hover['thrust_N'])>=0
 assert impossible['weighted_residual_norm']>1 and max(impossible['thrust_N'])<=maxT+1e-8
 result={'status':'offline feasibility under assumed candidate parameters, NOT PX4 integration or real-aircraft proof','matrix_rows':['tau_x','tau_y','tau_z','Fx','Fy','Fz'],'B':B.tolist(),'rank':int(np.linalg.matrix_rank(B)),'thrust_bounds_N':[0,maxT],'weights':weights.tolist(),'hover':hover,'unachievable_request':impossible}
 (ROOT/'allocation_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
