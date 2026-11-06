"""Verify the saved physical result independently of the demo success flag."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parent
out=ROOT/'output_pad_middle_default';assets=ROOT/'site/dist/assets'
m=json.loads((out/'metrics.json').read_text());trace=json.loads((out/'trace.json').read_text());frames=json.loads((out/'frames.json').read_text())
hold=[r for r in trace if r['phase']=='hold']
assert hold and hold[-1]['t']-hold[0]['t']>=1.9
assert min(r['bottle_lift_m'] for r in hold)>.08
assert np.mean([r['contacts']==2 for r in hold])>.9
assert np.isclose(min(r['bottle_lift_m'] for r in hold),m['min_hold_lift_m'],atol=.001)
assert np.max(np.abs(frames[0]['qpos'][:6]))<1e-4
xml=ET.parse(out/'scene.xml').getroot()
base=xml.find(".//body[@name='base_link']");bottle=xml.find(".//body[@name='bottle']")
assert base.find('joint') is None
assert np.allclose(np.fromstring(xml.find(".//site[@name='tcp']").get('pos'),sep=' '),[0,0,.095])
assert m['tcp_m']==.095
assert m['pad_support_verified']
assert m['hold_contact_inset_median_m']>.008
assert m['selected']['height_fraction']==.5
assert bottle.find('freejoint') is not None
assert xml.find(".//weld") is None
assert len(xml.findall('.//joint[@type="free"]'))==0
assert m['physical_attachment'] is False and m['object_pose_written_during_rollout'] is False
web=json.loads((assets/'metrics.json').read_text());assert web==m
motion=json.loads((assets/'motion.json').read_text());assert len(motion)==len(frames)
result=dict(status='PASS',hold_samples=len(hold),sampled_min_hold_lift_m=min(r['bottle_lift_m'] for r in hold),
            sampled_bilateral_fraction=float(np.mean([r['contacts']==2 for r in hold])),
            zero_start_verified=True,bottle_freejoint=True,no_weld=True,web_matches_physics=True)
(ROOT/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
