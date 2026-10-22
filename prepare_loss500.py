"""One-time immutable archive of VIS5 before loss tolerance changes."""
from pathlib import Path
import hashlib,json,shutil,zipfile
root=Path(__file__).resolve().parent
archive=Path('D:/AAAProject/FPGA/AAA_LookHere/成功版本归档/20261022_VIS5_短丢容忍前')
assert not archive.exists(),'Archive already exists; do not overwrite'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
bit=root/'output/vision_follow_20261022.bit'
assert sha(bit)=='1890c0545bd035ef84b0b8406653483c17826084f8b423d0f2ee671327f2c062'
files=[]
for p in root.rglob('*'):
    if not p.is_file():continue
    rel=p.relative_to(root)
    if any(x.endswith('_Runs') or x in ('__pycache__','.git') for x in rel.parts):continue
    if p.suffix in ('.vvp','.vcd','.db','.logw') or p.name.endswith('.combined.v'):continue
    if rel.parts[0]=='td_project' and p.suffix not in ('.al','.sdc'):continue
    files.append(p)
for p in files:
    dest=archive/'project'/p.relative_to(root);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
run=root/f'{root.name}_Runs/phy_1'
for suffix in ['_phy.area','_phy.ts','_pr.timing']:
    dest=archive/'reports'/f'{root.name}{suffix}';dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(run/dest.name,dest)
(archive/'恢复说明.md').write_text('''# 恢复短丢容忍修改前VIS5

停止电机后，使用project/output/vision_follow_20261022.bit按既有方法加载SRAM，器件PH1P35MDG324。配套上位机project/host/start_panel.cmd。

SHA256：1890c0545bd035ef84b0b8406653483c17826084f8b423d0f2ee671327f2c062。

这是原一帧无效即取消的连续PD版本，不包含500ms宽容。包含20261022_191240_780.txt实测日志；用户报告存在识别短丢导致SR3，不表示已通过持续稳定验收。没有写Flash。
''',encoding='utf-8')
hashes={p.relative_to(archive).as_posix():sha(p) for p in archive.rglob('*') if p.is_file()}
(archive/'manifest.json').write_text(json.dumps({'files':hashes,'bit_sha256':sha(bit)},ensure_ascii=False,indent=2),encoding='utf-8')
dest=archive.with_suffix('.zip')
with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED) as z:
    for p in archive.rglob('*'):
        if p.is_file():z.write(p,p.relative_to(archive).as_posix())
with zipfile.ZipFile(dest) as z:assert z.testzip() is None
lineage={'archive':str(archive),'archive_zip':str(dest),'old_bit_sha256':sha(bit),'date':'2026-10-22','approved':'normal red45/ratio2, local red35/ratio1.5 within48sensor pixels,2 missed frame history,500ms valid-ball grace,200ms result watchdog; only running AUTO may coast; all motor bounds retained'}
(root/'loss500_lineage.json').write_text(json.dumps(lineage,ensure_ascii=False,indent=2),encoding='utf-8')
print('Archived',len(hashes),'files; old bit verified')
