"""Set X/H/V on the non-flying kinematic bench and verify measured rotor poses."""
import argparse,datetime,json,math,pathlib,subprocess,time
import numpy as np
from kinematics import configuration,effectiveness,DEMO_KM,NAMES

ROOT=pathlib.Path(__file__).resolve().parent
PREFIX=['docker','exec','-e','GZ_PARTITION=px4_morph_bench','px4-rtl-land-harmonic']
def gz(*args):return subprocess.check_output(PREFIX+['gz',*args],text=True,stderr=subprocess.STDOUT,timeout=12)
def measured():
    text=gz('topic','-e','-t','/world/morph_bench/pose/info','-n','1','--json-output');data=json.loads(text)
    (ROOT/'latest_gazebo_pose.json').write_text(text)
    return {p['name']:p for p in data['pose']}
def main(mode):
    command={'X':0,'H':1,'V':-1}[mode];p,d,angles=configuration(command)
    # Always neutralize first so H and V are not simultaneously commanded.
    for name in NAMES:
        for kind in ['yaw','pitch']:gz('topic','-t',f'/morph_bench/{kind}_{name}/cmd_pos','-m','gz.msgs.Double','-p','data: 0')
    time.sleep(2)
    for name,a in zip(NAMES,angles):
        for kind,value in [('yaw',-a['yaw_frd_rad']),('pitch',a['dihedral_rad'])]:
            gz('topic','-t',f'/morph_bench/{kind}_{name}/cmd_pos','-m','gz.msgs.Double','-p',f'data: {value}')
    time.sleep(3)
    poses=measured();actual=[];actual_axes=[]
    for name in NAMES:
        pose=poses['rotor_'+name];pt=pose.get('position',{});actual.append(np.array([pt.get(x,0) for x in ['x','y','z']])*[1,-1,-1])
        q=pose.get('orientation',{});x,y,z,w=[q.get(k,1 if k=='w' else 0) for k in ['x','y','z','w']]
        up=np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
        actual_axes.append(up*[1,-1,-1])
    error=np.linalg.norm(np.array(actual)-p,axis=1);axis_error=np.linalg.norm(np.array(actual_axes)-d,axis=1)
    result={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'mode':mode,'demo_input':command,
            'rotor_order':NAMES,'expected_frd_m':p.tolist(),'measured_frd_m':np.array(actual).tolist(),
            'position_errors_m':error.tolist(),'axis_errors':axis_error.tolist(),
            'within_2mm_and_0p01_axis':bool(np.max(error)<.002 and np.max(axis_error)<.01),
            'B_from_measured_poses':effectiveness(np.array(actual),np.array(actual_axes),DEMO_KM,np.zeros(3)).tolist(),
            'status':'REFERENCE ONLY, NOT final aircraft; kinematic tool check, not flight or hardware validation'}
    (ROOT/f'verification_{mode}.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if not result['within_2mm_and_0p01_axis']:raise SystemExit('Measured pose differs from analytic geometry')

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('mode',choices=['X','H','V']);main(a.parse_args().mode)
