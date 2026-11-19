"""Restore native moving linkage and motors in one world, with PX4 attached by name."""
from pathlib import Path
import argparse, copy, json, hashlib, sys, xml.etree.ElementTree as E
import numpy as np
R=Path(__file__).resolve().parent;V=R.parent
sys.path.insert(0,str(V/'a_with_arm_fixed'))
from mass_properties import records,aggregate
ap=argparse.ArgumentParser();ap.add_argument('--payload',type=float,choices=[0,.05],default=0);args=ap.parse_args()
case='empty' if args.payload==0 else 'payload_50g'
src=V/'a_with_arm_fixed/results'/case/'world.sdf'
t=E.parse(src);w=t.find('world');w.set('name','a_arm_world')
air=w.find("model[@name='a_with_arm_fixed']");platform=w.find("model[@name='integrated_platform']")
air.set('name','a_arm');air.find('pose').text='0 0 .35 0 0 0'
p=np.fromstring(platform.findtext('pose'),sep=' ');p[2]-=.3;platform.find('pose').text=' '.join(map(str,p))
air.remove(air.find("joint[@name='fixture_fixed']"))
# The native loop needs 0.2 ms physics steps. Sample the IMU at every step so
# short contact/constraint impulses are integrated rather than undersampled.
air.find("link[@name='base_link']/sensor[@name='imu_sensor']/update_rate").text='5000'
for pl in list(air.findall('plugin')):air.remove(pl)
original=E.parse(V/'a_morphing/airborne/models/a_air/model.sdf').find('model')
for i in range(4):
 air.remove(air.find(f"joint[@name='rotor_{i}_joint']"))
 air.append(copy.deepcopy(original.find(f"joint[@name='rotor_{i}_joint']")))
for pl in original.findall('plugin'):
 if 'MulticopterMotorModel' in pl.get('name',''):air.append(copy.deepcopy(pl))
pl=E.SubElement(air,'plugin',filename='liba_arm_hover.so',name='a_arm_hover::Controller')
E.SubElement(pl,'payload').text=str(args.payload);E.SubElement(pl,'gravity').text='9.81'
total=aggregate(records(air,True)+records(platform,True));total['com_m'][2]-=.35
refname='empty' if args.payload==0 else 'load50g';ref=V/'a_locked_hover/baselines'/refname
prior=json.loads((ref/'preflight.json').read_text())
np.testing.assert_allclose(total['mass_kg'],prior['mass_properties']['mass_kg'],atol=1e-12)
np.testing.assert_allclose(total['com_m'],prior['mass_properties']['com_m'],atol=1e-12)
np.testing.assert_allclose(total['inertia_com_kg_m2'],prior['mass_properties']['inertia_com_kg_m2'],atol=1e-12)
assert not any(j.findtext('parent')=='world' for m in [air,platform] for j in m.findall('joint'))
assert len(air.findall("joint[@type='ball']"))==12
E.indent(t);t.write(R/'world.sdf',encoding='utf-8',xml_declaration=True)
f=(ref/'airframe.sh').read_text().replace('a_locked_world','a_arm_world').replace('PX4_SIM_MODEL=a_locked','PX4_SIM_MODEL=a_arm')
(R/'4994_a_arm').write_text(f)
(R/'reference-control-parameters.json').write_bytes((ref/'reference-control-parameters.json').read_bytes())
prior.update(payload_kg=args.payload,mass_properties=total,source_integrated_sdf=str(src.relative_to(V)),source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),world_sha256=hashlib.sha256((R/'world.sdf').read_bytes()).hexdigest(),rigid_lock_method=None,arm_constraints='3 actuated revolutes, 12 native balls, 6 runtime fixed loop closures',dt_s=.0002,imu_update_rate_hz=5000,allocation='initial home CoG geometry; no online CA parameter writes',motion_speed_m_s=.015,motion_acceleration_m_s2=.03)
(R/'preflight.json').write_text(json.dumps(prior,indent=2))
print('Moving arm world prepared:',args.payload,'kg payload,',total['mass_kg'],'kg total')
