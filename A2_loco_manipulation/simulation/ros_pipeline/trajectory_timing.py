"""Monotone time scaling: preserves every collision-checked joint-path segment."""
import math

def eased_path_time(elapsed, duration, ramp=.3):
    if duration <= 0:
        raise ValueError('Positive trajectory duration required')
    ramp=min(ramp,duration)
    if ramp <= 0:
        raise ValueError('Positive ramp required')
    elapsed=max(0.,float(elapsed));total=duration+ramp
    def edge(t):return t/2-ramp/(2*math.pi)*math.sin(math.pi*t/ramp)
    if elapsed < ramp:return edge(elapsed)
    if elapsed <= total-ramp:return elapsed-ramp/2
    if elapsed < total:return duration-edge(total-elapsed)
    return float(duration)
