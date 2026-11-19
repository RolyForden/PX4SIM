"""Cross-check physics implementation, independent statics and time-step refinement."""
from pathlib import Path
import json,numpy as np,xml.etree.ElementTree as E
R=Path(__file__).resolve().parent;D=R/'results'
def read(p):return json.loads((D/p).read_text())
loaded=read('gravity_payload_50g/summary.json');fine=read('payload_50g_half_step/summary.json');freefall=read('freefall-check.json');comparison=read('comparison.json')
dtau=float(np.max(np.abs(np.array(loaded['static_joint_torque_nm'])-fine['static_joint_torque_nm'])));dforce=abs(loaded['static_support_force_n'][2]-fine['static_support_force_n'][2])
source=(R/'controller.cpp').read_text();forbidden=[s for s in ['JointPositionReset','JointVelocityReset','SetWorldPoseCmd','SetLinearVelocity','SetAngularVelocity'] if s in source]
w=E.parse(R/'world.sdf').getroot();links=[l for name in ['delta_structure','delta_platform'] for l in w.findall(f"world/model[@name='{name}']/link")]
audit={'passive_motion_overrides_found':forbidden,'all_manipulator_links_gravity_enabled':all(x.findtext('gravity')=='true' for x in links),'native_ball_joint_count':len(w.findall(".//joint[@type='ball']")),'actuated_revolute_count':len(w.findall(".//joint[@type='revolute']")),'world_gravity':w.findtext('world/gravity')}
result={'passed':bool(comparison['passed'] and freefall['passed'] and fine['passed'] and dtau<.001 and dforce<.01 and not forbidden and audit['all_manipulator_links_gravity_enabled'] and audit['native_ball_joint_count']==6),'half_step_static_torque_difference_nm':dtau,'half_step_support_force_difference_n':dforce,'freefall_control':freefall,'model_audit':audit,'comparison':'comparison.json','flight_started':False}
(D/'validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2));assert result['passed']
