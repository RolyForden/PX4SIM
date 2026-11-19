"""Check saved model snapshots, parameter readback and source fingerprints."""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as E
R=Path(__file__).resolve().parent
checks=[]
for s in sorted((R/'runs').glob('*/summary.json')):
 d=s.parent;p=json.loads((d/'preflight.json').read_text());m=E.parse(d/'model.sdf').find('model');b=json.loads((d/'parameters/before.json').read_text());a=json.loads((d/'parameters/after.json').read_text());ref=json.loads((d/'reference-control-parameters.json').read_text())
 assert hashlib.sha256((R.parent/p['source_integrated_sdf']).read_bytes()).hexdigest()==p['source_sha256']
 assert hashlib.sha256((d/'model.sdf').read_bytes()).hexdigest()==p['model_sha256']
 assert m.findtext('static','false')!='true' and not any(j.findtext('parent')=='world' for j in m.findall('joint'))
 assert sum(j.get('type')=='revolute' for j in m.findall('joint'))==4
 assert all(j.get('type') in ['revolute','fixed'] for j in m.findall('joint'))
 assert len(m.findall('plugin'))==4
 assert all('MulticopterMotorModel' in x.get('name','') for x in m.findall('plugin'))
 changes={k:[b.get(k),a.get(k)] for k in b.keys()|a.keys() if b.get(k)!=a.get(k)}
 # Commander increments these flight bookkeeping counters after landing.
 assert set(changes)<= {'COM_FLIGHT_UUID','LND_FLIGHT_T_LO'},changes
 for k,v in ref.items():assert abs(a[k]-v)<1e-5*max(1,abs(v))
 for i in range(4):
  expected=dict(zip(['AX','AY','AZ'],p['rotor_axes_frd'][i]));expected.update(CT=6.5,KM=p['rotor_KM_m'][i]);expected.update(zip(['PX','PY','PZ'],p['rotor_geometry_frd_m'][i]))
  for key,v in expected.items():assert abs(a[f'CA_ROTOR{i}_{key}']-v)<1e-6
 checks.append({'trial':d.name,'source_and_model_hashes_match':True,'no_world_anchor':True,'only_four_rotors_revolute':True,'motor_plugins_only':True,'parameter_changes':changes,'control_parameter_count':len(ref),'control_parameters_unchanged':True,'rotor_geometry_axes_CT_KM_readback_matches':True})
assert len(checks)==6
(R/'results/audit.json').write_text(json.dumps(checks,indent=2))
print('Six model, source hash and parameter audits passed.')
