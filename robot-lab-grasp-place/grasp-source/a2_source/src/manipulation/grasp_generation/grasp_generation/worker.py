"""Persistent isolated CUDA worker. JSON lines on stdin/stdout; no ROS or devices."""
import contextlib
import json
import sys
import time
import traceback
from grasp_generation.graspnet_backend import Predictor


def main():
    with open(sys.argv[1]) as f: config = json.load(f)
    # Upstream libraries may print during import. Keep stdout a strict protocol.
    with contextlib.redirect_stdout(sys.stderr): predictor = Predictor(config['model'])
    print(json.dumps({'ready': True}), flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            start = time.monotonic()
            with contextlib.redirect_stdout(sys.stderr): rows = predictor.predict(request['points'])
            reply = dict(id=request['id'], rows=rows.tolist(), inference_sec=time.monotonic()-start)
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            reply = dict(error=str(exc))
        print(json.dumps(reply, allow_nan=False), flush=True)

if __name__ == '__main__': main()
