"""Local simulation transport. No ROS, SDK or motion is imported by the client."""

import json
import socket
import time


def call(path, method, **arguments):
    with socket.socket(socket.AF_UNIX) as stream:
        stream.settimeout(10)
        # Retry only connect-before-send under concurrent sensor load. Never
        # replay a request after bytes have been submitted to the world.
        deadline = time.monotonic() + .5
        while True:
            try:
                stream.connect(str(path))
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.005)
        stream.sendall(
            (json.dumps({"method": method, **arguments}, allow_nan=False) + "\n").encode()
        )
        with stream.makefile("rb") as file:
            raw = file.readline(8 * 1024 * 1024 + 1)
        if len(raw) > 8 * 1024 * 1024:
            raise ValueError("Simulation response exceeds limit")
        value = json.loads(raw)
        if set(value) == {"error"}:
            raise RuntimeError(value["error"])
        return value
