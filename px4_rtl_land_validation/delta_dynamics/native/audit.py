"""Check actual native closed-chain records and reject hidden motion forcing."""
from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np
R=Path(__file__).resolve().parent;P=R.parent/'native_probe'
probes=json.loads((P/'results.json').read_text());direct=[r for r in probes if r['mode']=='direct'];proxy=[r for r in probes if r['mode']=='proxy'];checks={'direct_sdf_failed_to_close':all(r['max_gap_m']>.1 for r in direct),'native_proxy_minimal_closed':all(r['exit_code']==0 and r['max_gap_m']<1e-7 for r in proxy)}
summary=[]
for name in ['gravity_empty','payload_50g','payload_50g_rotated']:
 s=json.loads((R/'results'/name/'summary.json').read_text());rows=[json.loads(l) for l in (R/'results'/name/'feedback.jsonl').read_text().splitlines()];w=[json.loads(l) for l in (R/'results'/name/'mount_wrench.jsonl').read_text().splitlines()];expected=(.05+s['settings']['payload']+s['settings']['proxy_total_mass_kg'])*9.81;err=abs(s['static_support_force_n'][2]-expected);entry={'case':name,'passed':bool(s['passed'] and s['max_closure_m']<1e-5 and err<.002 and s['max_base_displacement_m']<1e-6),'closure_m':s['max_closure_m'],'support_force_error_with_proxy_mass_n':err,'fixture_wrench_samples':sum(r['valid'] for r in w)};summary.append(entry)
 checks[name]=entry['passed'] and entry['fixture_wrench_samples']>100
source=(R/'controller.cpp').read_text();forbidden=[x for x in ['JointPositionReset','JointVelocityReset','SetWorldPoseCmd','SetLinearVelocity','SetAngularVelocity','AddWorldForce','AddWorldWrench'] if x in source];checks['no_manual_passive_motion_or_force']=not forbidden
root=E.parse(R/'world.sdf').getroot();checks['twelve_native_ball_joints']=len(root.findall(".//joint[@type='ball']"))==12
checks['runtime_fixed_loop_creation']='components::DetachableJoint({platform,proxy[i][k],"fixed"})' in source
out={'passed':all(checks.values()),'checks':checks,'cases':summary,'forbidden_api_found':forbidden,'flight_tested':False};(R/'results/validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));assert out['passed']
