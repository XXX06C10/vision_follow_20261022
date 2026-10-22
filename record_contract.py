"""Pin the already passed source/script lineage before vision integration."""
from pathlib import Path
import hashlib
import json
import sys

sys.stdout.reconfigure(encoding='utf-8')
root=Path(__file__).resolve().parent
base=root.parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
projects=['d36a_autocal_latch_20261020','d36a_position5_20261020','d36a_autocal_20261020']
entries={}
for project in projects:
    directory=base/project
    files=[directory/name for name in ['build.ps1','build.tcl','run_sim.ps1','compose.py',project+'.adc',project+'.sdc']]
    files+=list((directory/'rtl').glob('*.v'))
    files.append(directory/'output'/f'{project}.bit')
    entries[project]={str(p.relative_to(directory)).replace('\\','/'):sha(p) for p in files}
contract={'status':'BASELINES_FROZEN_INTEGRATION_NOT_BUILT','date':'2026-10-20',
    'user_requirement':'旧视觉/电机工程仅供参考；代码开发必须严格按照一路已运行成功的脚本和控制行为。',
    'references':entries,'reference_project':'lab_hd_1_mipi_hdmi_1080p_steel_ball_motor_manual',
    'forbidden_trusted_motor_source':'steel_ball_motor_manual.v',
    'next_milestone':'coherent vision+encoder+bounded target observation, followed by limited single-axis motion',
    'no_new_bit_delivered_this_turn':True}
destination=root/'baseline_contract.json'
if destination.exists():
    previous=json.loads(destination.read_text(encoding='utf-8'))
    assert previous['references']==entries,'Golden source/script hash changed; inspect before updating contract'
else:
    destination.write_text(json.dumps(contract,ensure_ascii=False,indent=2),encoding='utf-8')
for project,files in entries.items():
    for rel,digest in files.items():assert sha(base/project/rel)==digest
print('Verified baseline source/script/bit hashes:',sum(map(len,entries.values())))
print('Integration status:',json.loads(destination.read_text(encoding='utf-8'))['status'])
