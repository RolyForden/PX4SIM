"""Copy the original ULog after PX4 shutdown has flushed its final buffer."""
from pathlib import Path
import sys,json,hashlib,shutil
R=Path(__file__).resolve().parent;D=R/'runs'/sys.argv[1]
if (R/'active.json').exists():
 assert json.loads((R/'active.json').read_text())['name']!=D.name,'Stop this trial after landing before finalizing'
o=json.loads((D/'outcome.json').read_text());sources=list((D/'runtime/log').rglob('*.ulg'));assert len(sources)==1
src=sources[0];dest=D/'logs'/f'{D.name}.ulg'
metadata={'file':str(dest.relative_to(D)),'bytes':src.stat().st_size,'sha256':hashlib.sha256(src.read_bytes()).hexdigest()}
if o.get('ulog')!=metadata:o['ulog_before_shutdown_snapshot']=o.get('ulog')
shutil.copy2(src,dest);o['ulog']=metadata;o['ulog_finalized_after_shutdown']=True
(D/'outcome.json').write_text(json.dumps(o,indent=2));print('Finalized original ULog:',D.name,metadata['bytes'])
