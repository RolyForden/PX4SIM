"""Demonstration kinematics and physical thrust effectiveness, not a flight controller.

Single demonstration input: +1 = H, 0 = X, -1 = V/upward dihedral.
Virtual hinges, angles, CoG and torque ratios are uncalibrated, explicit assumptions.
All numeric geometry is a reference-only test fixture, NOT final aircraft parameters.
"""
import json,math,pathlib
import numpy as np

ROOT=pathlib.Path(__file__).resolve().parent
NAMES=['LF','LR','RF','RR']
SIGNS=np.array([[1,-1],[-1,-1],[1,1],[-1,1]]) # FRD: forward, right
BASE=np.array([[sx*.175030,sy*.175030,-.03005] for sx,sy in SIGNS])
HINGES=np.array([[sx*.045,sy*.045,0.] for sx,sy in SIGNS])
DEMO_KM=np.array([-.01,.01,.01,-.01]) # illustrative signed torque/thrust ratio in metres

def rotation(axis,angle):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def configuration(command):
    if not -1<=command<=1:raise ValueError('Demonstration input must be within [-1,1]')
    alpha=math.radians(25)*max(command,0);beta=math.radians(25)*max(-command,0)
    positions=[];directions=[];angles=[]
    for (sx,sy),base,hinge in zip(SIGNS,BASE,HINGES):
        yaw=-sx*sy*alpha
        radial=(base-hinge).copy();radial[2]=0;radial/=np.linalg.norm(radial)
        pitch_axis=np.cross(radial,[0,0,-1.])
        R=rotation([0,0,1],yaw)@rotation(pitch_axis,beta)
        positions.append(hinge+R@(base-hinge));directions.append(R@np.array([0,0,-1.]))
        angles.append({'yaw_frd_rad':yaw,'dihedral_rad':beta})
    return np.array(positions),np.array(directions),angles

def effectiveness(positions,directions,km,reference):
    positions=np.asarray(positions,float);directions=np.asarray(directions,float)
    km=np.asarray(km,float);reference=np.asarray(reference,float)
    if positions.shape!=directions.shape or positions.shape[1]!=3:raise ValueError('N by 3 geometry required')
    if not np.allclose(np.linalg.norm(directions,axis=1),1,atol=1e-9):raise ValueError('Thrust directions must be unit vectors')
    moments=np.cross(positions-reference,directions)-km[:,None]*directions
    return np.vstack([moments.T,directions.T])

def verify():
    p,d,_=configuration(0);np.testing.assert_allclose(p,BASE,atol=1e-12)
    B=effectiveness(p,d,DEMO_KM,np.zeros(3))
    np.testing.assert_allclose(B[0],-p[:,1]);np.testing.assert_allclose(B[1],p[:,0]);np.testing.assert_allclose(B[2],DEMO_KM)
    np.testing.assert_allclose(B[5],-np.ones(4))
    force=np.array([1.,2.,3.,4.]);reference=np.array([.01,-.02,.03])
    for q in [-1,-.5,0,.5,1]:
        p,d,_=configuration(q);B=effectiveness(p,d,DEMO_KM,reference)
        expected=np.r_[np.sum(np.cross(p-reference,d*force[:,None])-DEMO_KM[:,None]*d*force[:,None],axis=0),np.sum(d*force[:,None],axis=0)]
        np.testing.assert_allclose(B@force,expected,atol=1e-12)
        assert np.linalg.matrix_rank(B)<=4
    p,d,_=configuration(-1)
    np.testing.assert_allclose(d[:,2],-math.cos(math.radians(25)))
    assert np.all(p[:,2]<BASE[:,2]) # positive dihedral raises motors in FRD
    return {'zero_input_matches_reference':True,'vertical_column_signs':True,'direct_force_moment_sum':True,'unit_axes':True,'dihedral_raises_motors':True,'at_most_four_independent_thrust_inputs':True}

if __name__=='__main__':
    checks=verify();results={}
    for name,q in [('X',0),('H',1),('V',-1)]:
        p,d,a=configuration(q);B=effectiveness(p,d,DEMO_KM,np.zeros(3))
        results[name]={'demo_input':q,'positions_frd_m':p.tolist(),'thrust_unit_axes_frd':d.tolist(),'joint_angles':a,
                       'B_tau_xyz_force_xyz_per_newton':B.tolist(),'rank':int(np.linalg.matrix_rank(B))}
    out={'status':'REFERENCE ONLY, NOT final aircraft; illustrative geometry and assumed parameters','coordinate_frame':'FRD','input':'actual rotor thrust T_i in N, NOT PWM or throttle percent',
         'row_order':['tau_x','tau_y','tau_z','F_x','F_y','F_z'],'rotor_order':NAMES,
         'formula':'B_i = [ cross(r_i-reference,d_i) - km_i*d_i ; d_i ]',
         'px4_relation':'E_i = CT_i*B_i; physical model T_i=CT_i*u_i^2, not linear raw throttle',
         'assumptions':{'hinges_frd_m':HINGES.tolist(),'reference_frd_m':[0,0,0],'signed_km_m':DEMO_KM.tolist(),'H_demo_angle_deg':25,'V_demo_angle_deg':25,
                        'frame_and_motor_origins':'provisional CAD-derived placement, not surveyed thrust centres','single_drive':'H and V use different branches; simultaneous independent control not assumed'},
         'checks':checks,'configurations':results}
    (ROOT/'allocation_demo.json').write_text(json.dumps(out,indent=2));print(json.dumps({'checks':checks,'ranks':{k:v['rank'] for k,v in results.items()}},indent=2))
