"""Independent per-joint freshness and stable capture; no ROS dependency."""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class Sample:
    position: float
    received: float
    sequence: int


class JointCache:
    def __init__(self, names, max_age=.5, max_skew=.2, clock=time.monotonic):
        self.names = tuple(names)
        self.max_age, self.max_skew, self.clock = max_age, max_skew, clock
        self.samples = {}
        self.sequence = 0

    def update(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('Invalid joint state shape')
        if any(not math.isfinite(q) for q in positions):
            raise ValueError('Nonfinite joint position')
        now = self.clock()
        self.sequence += 1
        for name, q in zip(names, positions):
            if name in self.names:
                self.samples[name] = Sample(q, now, self.sequence)

    def invalidate(self, name):
        self.samples.pop(name, None)

    def snapshot(self):
        now = self.clock()
        result = {}
        for name in self.names:
            s = self.samples.get(name)
            if s is None or not 0 <= now-s.received <= self.max_age:
                raise ValueError(f'Missing or stale joint: {name}')
            result[name] = s
        stamps = [s.received for s in result.values()]
        if max(stamps)-min(stamps) > self.max_skew:
            raise ValueError('Joint readings are too far apart in time')
        return result


class StableCapture:
    def __init__(self, tolerance=.005, required=3):
        self.tolerance, self.required = tolerance, required
        self.history = []

    def feed(self, snapshot):
        if self.history:
            last = self.history[-1]
            if set(last) != set(snapshot):
                self.history.clear()
            elif any(snapshot[n].sequence <= last[n].sequence for n in snapshot):
                return None
        if self.history and any(
                max([s[n].position for s in self.history]+[snapshot[n].position]) -
                min([s[n].position for s in self.history]+[snapshot[n].position]) > self.tolerance
                for n in snapshot):
            self.history.clear()
        self.history.append(snapshot)
        if len(self.history) >= self.required:
            return {n: s.position for n, s in snapshot.items()}
        return None
