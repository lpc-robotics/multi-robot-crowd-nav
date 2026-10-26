"""Stop on coherent clock stalls without inventing simulation-time progress."""
from dataclasses import dataclass, field


@dataclass
class ClockStallGate:
    maximum_wall_s: float = 30.0
    maximum_sim_gap_ns: int = 100_001_000
    started: float | None = None
    clock_seen: float | None = None
    stamp_ns: int | None = None
    robot_stamps: dict = field(default_factory=dict)
    recoveries: int = 0
    last_gap_s: float = 0.0

    @property
    def active(self):
        return self.started is not None

    def request_started_after_resume(self, started, now):
        """Exclude only the overlap with the confirmed stalled clock interval.

        Detection takes one observation timeout. That initial interval also
        belongs to the stall, and a request can have been sent within it.
        Keep any wait accumulated before the stall; never reset its deadline.
        """
        return started + max(0.0, now-max(started, self.clock_seen))

    def update(self, now, stamp_ns, clock_seen, robots, timeout):
        """Return (hold, paused_wall_time_to_exclude_from_service_deadlines).

        A running clock with one missing robot is never a recoverable stall.
        Recovery requires a newer clock and a NEW sample from EVERY robot.
        """
        if not self.active:
            if clock_seen is None or not robots:
                return False, 0.0
            ages = [now-seen for _, seen in robots.values()]
            # Allow delivery-order jitter around the same 0.6s cohort deadline.
            coherent = (now-clock_seen >= timeout-.02 and
                        min(ages) >= timeout-.02 and max(ages) > timeout and
                        all(abs(stamp_ns-s) <= self.maximum_sim_gap_ns for s, _ in robots.values()))
            if not coherent:
                return False, 0.0
            self.started, self.clock_seen, self.stamp_ns = now, clock_seen, stamp_ns
            self.robot_stamps = {name: stamp for name, (stamp, _) in robots.items()}
        if now-self.clock_seen > self.maximum_wall_s:
            raise ValueError("simulation_clock_stall_timeout")
        if stamp_ns < self.stamp_ns:
            raise ValueError("clock_rewind")
        if stamp_ns-self.stamp_ns > self.maximum_sim_gap_ns:
            raise ValueError("simulation_time_gap_after_stall")
        fresh = (stamp_ns > self.stamp_ns and clock_seen > self.started and now-clock_seen <= timeout and
                 robots.keys() == self.robot_stamps.keys() and
                 all(seen > self.started and now-seen <= timeout and stamp > self.robot_stamps[name]
                     and -.050001 <= (stamp_ns-stamp)/1e9 <= .100001
                     for name, (stamp, seen) in robots.items()))
        if not fresh:
            return True, 0.0
        held = now-self.started
        self.last_gap_s = now-self.clock_seen
        self.recoveries += 1
        self.started = None
        return False, held
