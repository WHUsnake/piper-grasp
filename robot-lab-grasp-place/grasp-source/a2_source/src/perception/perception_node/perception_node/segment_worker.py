"""One captured RGB image -> actual YOLO mask. No simulator object IDs."""
import argparse,json
from pathlib import Path
import numpy as np
from ultralytics import YOLO

def main():
    p=argparse.ArgumentParser();p.add_argument('--image',required=True);p.add_argument('--output',required=True);p.add_argument('--weights',required=True);a=p.parse_args()
    m=YOLO(a.weights);r=m.predict(a.image,conf=.3,device='cpu',retina_masks=True,verbose=False)[0]
    choices=[(float(b.conf.item()),i) for i,b in enumerate(r.boxes) if r.names[int(b.cls.item())]=='bottle']
    if not choices or r.masks is None:raise RuntimeError('YOLO did not segment a bottle; oracle fallback disabled')
    score,i=max(choices);mask=r.masks.data[i].cpu().numpy()>.5
    np.save(a.output,mask);Path(a.output+'.json').write_text(json.dumps(dict(method='YOLO11s-seg',confidence=score,oracle=False,pixels=int(mask.sum()))))
if __name__=='__main__':main()
