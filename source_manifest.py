"""Prevent delivery of an earlier bit after source changes during a long build."""
from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parent
paths=list((root/'user_source').rglob('*'))+list((root/'rtl').glob('*.v'))
paths+=list((root/'td_project').glob('*.al'))+list((root/'td_project').glob('*.sdc'))
paths += [root/p for p in ['build.ps1','build.tcl','compose.py','generate_uart.py']]
hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths) if p.is_file()}
dest=root/'build_sources.json'
if '--capture' in sys.argv:dest.write_text(json.dumps(hashes,indent=2),encoding='utf-8')
else:assert json.loads(dest.read_text())==hashes,'Sources changed during/after the build; rebuild before delivery'
print('Build source manifest verified:',len(hashes))
