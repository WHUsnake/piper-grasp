#!/usr/bin/env python3
"""Replay archived target clouds without ROS, hardware or wall-clock age bypasses.

Without recorded base rotation this reports camera-frame proposals only, and
explicitly skips horizontal acceptance. Pass measured acquisition-time rotation
as --base-rotation JSON file for a physically meaningful horizontal selection.
"""
import argparse
import json
import os
import sys
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/manipulation/grasp_generation'))
from grasp_generation.graspnet_backend import adapt
from grasp_generation.worker_client import Worker


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'src/manipulation/grasp_generation/config/graspnet.json')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--base-rotation',type=Path)
    args=p.parse_args(); cfg=json.loads(args.config.read_text())
    search=args.samples.resolve().parents[2]
    sys.path.insert(0,str(search))
    import yaml
    from p2.input import read
    input_cfg=yaml.safe_load((search/'config/p2.graspnet.yaml').read_text())
    rb=np.array(json.loads(args.base_rotation.read_text())) if args.base_rotation else np.eye(3)
    adapter=dict(cfg['adapter'])
    if not args.base_rotation:
        adapter.update(horizontal_tolerance_deg=90.,closing_tolerance_deg=90.)
    records=[]
    start=time.monotonic()
    worker_python = os.path.expandvars(cfg.get('worker_python', ''))
    if not worker_python or worker_python.startswith('${'):
        worker_python = os.environ.get('GRASPNET_PYTHON', sys.executable)
    worker=Worker(worker_python,ROOT/'src/manipulation/grasp_generation/grasp_generation/worker.py',str(args.config.resolve()))
    load=time.monotonic()-start
    try:
        for entry in json.loads(args.samples.read_text())['observations']:
            obs=read(entry['path'],input_cfg,entry.get('selection'))
            reply=worker.predict(obs['target'])
            cs,counts=adapt(reply['rows'],rb,adapter)
            for c in cs:
                axis=Rotation.from_quat(c['quaternion']).as_matrix()[:,2]
                c['pregrasp_position_m']=(np.array(c['position'])-c['pregrasp_distance']*axis).tolist()
            records.append(dict(path=entry['path'],frame_id=obs['manifest']['frame_id'],
                                target_points=len(obs['target']),raw_count=len(reply['rows']),
                                filtered_count=len(cs),inference_sec=reply['inference_sec'],
                                rejected=counts,candidates=cs,raw_rows=reply['rows']))
    finally:worker.close()
    report=dict(execute=False,mode='offline',model=cfg['model'],load_and_warmup_sec=load,
                horizontal_filter_verified=bool(args.base_rotation),tcp_geometry_verified=False,
                moveit_verified=False,records=records)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),load_and_warmup_sec=load,
                         records=[{k:r[k] for k in ('target_points','raw_count','filtered_count','inference_sec')} for r in records])))

if __name__=='__main__':main()
