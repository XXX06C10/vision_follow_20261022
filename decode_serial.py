"""Decode VIS=5 camera/ROI telemetry files. All wire numbers are hex."""
from pathlib import Path
import argparse,csv,json,re
ROOT=Path(__file__).resolve().parent
LAYOUT=json.loads((ROOT/'frame_layout.json').read_text())['layout']
PATTERN=re.compile('^'+' '.join(f'{key}='+('5' if kind=='literal' else f'(?P<{key}>[0-9A-F]{{{width}}})') for key,width,_,kind in LAYOUT)+'$')
def parse_record(line):
    m=PATTERN.fullmatch(line.strip())
    if not m:return None
    r={k:int(v,16) for k,v in m.groupdict().items()}
    if any(r[k] not in (0,1) for k in ['V','EN','Z','B','CB','MB','OK','RD','AR','CZ']):return None
    for k,bits in [('E1',32),('E2',32),('P',32),('T',8),('MT',32),('ME',32),('MC',32),('VE',16),('AT',8)]:
        if r[k] & (1<<(bits-1)):r[k]-=1<<bits
    return r
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('--csv',type=Path);args=p.parse_args()
    records=[r for line in args.capture.read_text(encoding='utf-8-sig',errors='replace').splitlines() if (r:=parse_record(line))]
    if not records:raise SystemExit('No complete VIS=5 records found')
    summary={'records':len(records),'valid_records':sum(r['V']==1 for r in records),
             'fault_records':sum(r['F']!=0 for r in records),'last':records[-1],
             'target_min':min(r['T'] for r in records),'target_max':max(r['T'] for r in records),
             'coasting_target_records':sum(r['V']==0 and r['AR']==1 and r['T']!=0 for r in records),
             'invalid_noncoasting_nonzero_target_records':sum(r['V']==0 and r['AR']==0 and r['T']!=0 for r in records)}
    if args.csv:
        with args.csv.open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=records[0].keys());writer.writeheader();writer.writerows(records)
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
