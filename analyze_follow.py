"""Summarize VIS5 captures without manual notes; times use the nominal 250ms report period."""
from pathlib import Path
import argparse,json,math,statistics
from decode_serial import parse_record

def metrics(rows):
    usable=[r for r in rows if r['V'] and r['AGE']<200 and r['F']==0]
    errors=[r['BX']-r['C'] for r in usable]
    return {'reports':len(rows),'usable_reports':len(usable),
        'estimated_report_span_seconds':round(max(0,len(rows)-1)*0.25,2),
        'error_pixels':None if not errors else {
            'mean':round(statistics.mean(errors),3),'mean_absolute':round(statistics.mean(map(abs,errors)),3),
            'rms':round(math.sqrt(statistics.mean(e*e for e in errors)),3),'peak_absolute':max(map(abs,errors)),
            'within_configured_deadzone_percent':round(100*sum(abs(r['BX']-r['C'])<=r['DZ'] for r in usable)/len(usable),2)},
        'P_range':None if not rows else [min(r['P'] for r in rows),max(r['P'] for r in rows)],
        'fault_codes':sorted({r['F'] for r in rows if r['F']}),
        'invalid_or_stale_reports':sum(not r['V'] or r['AGE']>=200 for r in rows),
        'encoder_error_max':max((r['X1'] for r in rows),default=0)}

def analyze(lines):
    rows=[];malformed=0
    for line in lines:
        r=parse_record(line)
        if r is not None:rows.append(r)
        elif line.strip().startswith('VIS='):malformed+=1
    sessions=[];active=[]
    for r in rows:
        if r['MD']==1 and r['AR']==1:
            # A center/config change or elapsed-time rollback starts a separate window.
            if active and (r['C']!=active[-1]['C'] or r['SEQ']!=active[-1]['SEQ'] or r['DS']!=active[-1]['DS'] or r['EL']<active[-1]['EL']):
                sessions.append((active,'capture boundary/config changed'));active=[]
            active.append(r)
        elif active:
            sessions.append((active,r['SR']));active=[]
    if active:sessions.append((active,'capture ended while running'))
    result={'input_complete_reports':len(rows),'malformed_VIS_lines':malformed,'all_reports':metrics(rows),
        'automatic_sessions':[],
        'limits':'Pixel statistics describe only captured valid reports. Nominal report period250ms; dropped lines distort time estimates. No angle/length calibration, automatic pass/fail, or physical stability certification.'}
    for i,(rr,stop) in enumerate(sessions,1):
        result['automatic_sessions'].append({'number':i,'center':rr[0]['C'],
            'settings':{k:rr[0][k] for k in ['G','L','DZ','KD','SL','RU']},'last_EL_seconds':rr[-1]['EL'],
            'stop_reason':stop,'whole':metrics(rr),'last_40_reports':metrics(rr[-40:])})
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path,nargs='?');p.add_argument('--output',type=Path)
    a=p.parse_args();root=Path(__file__).resolve().parent
    capture=a.capture
    if capture is None:
        logs=list((root/'host/logs').glob('*.txt'))
        if not logs:p.error('No host/logs/*.txt yet; specify a VIS5 capture path')
        capture=max(logs,key=lambda f:f.stat().st_mtime)
    result=analyze(capture.read_text(encoding='utf-8-sig',errors='replace').splitlines())
    if not result['input_complete_reports']:p.error('No complete VIS5 reports in capture')
    result['capture']=str(capture.resolve());dest=a.output or capture.with_suffix('.analysis.json')
    dest.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2));print('Saved:',dest)

if __name__=='__main__':main()
