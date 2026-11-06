"""Repeatable isolated full-ROS physical regression, with no robot transport."""
import json,subprocess,argparse,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
CASES=[('default',.72,0.,.76),('low',.72,0.,.59),('near_low',.60,0.,.64),('far',.79,0.,.73),('left',.72,.10,.73),('right',.72,-.10,.73),('far_repeat',.79,0.,.73)]
WIDE_CASES=[('near_high',.55,0.,.85),('far_low',.85,0.,.65),('left_wide',.72,.20,.73),('right_wide',.72,-.20,.73),('low_offset',.65,.10,.60),('high_offset',.65,-.10,.85)]
EDGE_CASES=[('short_approach',.65,0.,.55),('very_low',.70,0.,.50)]
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--suite',choices=['standard','wide','edge'],default='standard');p.add_argument('--cases',default='');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);results=[]
 selected_cases={'standard':CASES,'wide':WIDE_CASES,'edge':EDGE_CASES}[a.suite]
 if a.cases:
  names=set(a.cases.split(','));selected_cases=[row for row in selected_cases if row[0] in names]
  if names!={row[0] for row in selected_cases}:p.error('Unknown case name')
 for name,x,y,z in selected_cases:
  out=a.output/name;started=time.monotonic()
  with (a.output/(name+'.log')).open('w') as log:
   result=subprocess.run([str(ROOT/'run.sh'),'--output',str(out.resolve()),'--bottle-x',str(x),'--bottle-y',str(y),'--table-z',str(z),'--tcp-m','.095'],stdout=log,stderr=subprocess.STDOUT)
  metrics=json.loads((out/'metrics.json').read_text()) if (out/'metrics.json').exists() else {}
  row=dict(case=name,parameters=dict(bottle_x=x,bottle_y=y,table_z=z,tcp_m=.095),exit_code=result.returncode,success=metrics.get('success',False),elapsed_sec=round(time.monotonic()-started,2),min_hold_lift_m=metrics.get('min_hold_lift_m'),bilateral_hold_fraction=metrics.get('bilateral_hold_fraction'))
  if (out/'pipeline_failure.json').exists():row['failure']=json.loads((out/'pipeline_failure.json').read_text())
  results.append(row);(a.output/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps({k:v for k,v in row.items() if k!='failure'},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
