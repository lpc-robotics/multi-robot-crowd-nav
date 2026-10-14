"""Read-only, per-step route acceptance shared by CPU and rendered probes."""
from __future__ import annotations

import math


class RouteTracker:
    def __init__(self, route, goal_radius):
        if not route.cyclic or len(route.points) < 2:
            raise ValueError("route acceptance requires a cyclic route with >=2 goals")
        self.route = route
        self.radius = goal_radius
        self.index = None
        self.last_stamp = None
        self.samples = 0
        self.visits = []

    def queue(self, index):
        return self.route.points[index:] + self.route.points[:index]

    def observe(self, stamp_ns, x, y, goals):
        goals = tuple(goals)
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError("nonfinite route position")
        if self.last_stamp is not None:
            delta = stamp_ns - self.last_stamp
            if delta <= 0 or delta > 25_000_001:
                raise ValueError("missing or nonmonotonic authoritative route samples")
        if self.index is None:
            matches = [i for i in range(len(goals)) if self.queue(i) == goals]
            if len(matches) != 1:
                raise ValueError("initial queue does not uniquely match configured route")
            self.index = matches[0]
        else:
            following = (self.index + 1) % len(self.route.points)
            if goals == self.queue(following):
                distance = math.dist((x, y), self.route.points[self.index])
                if distance > self.radius + 1e-6:
                    raise ValueError("goal advanced before arrival")
                self.visits.append({"goal": self.route.names[self.index],
                                    "stamp_ns": stamp_ns, "distance_m": distance})
                self.index = following
            elif goals != self.queue(self.index):
                raise ValueError("skipped, reordered or changed route goals")
        self.samples += 1
        self.last_stamp = stamp_ns

    def report(self):
        return {"samples": self.samples, "completed_cycles": len(self.visits) // len(self.route.points),
                "current_goal": None if self.index is None else self.route.names[self.index],
                "period": list(self.route.names), "visits": self.visits}
