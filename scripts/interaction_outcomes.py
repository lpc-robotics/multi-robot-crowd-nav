"""Navigation exposure observations, separate from crowd-model validity.

Durations count only consecutive sampled endpoints below the threshold. They
are lower bounds on observed exposure, not interpolated contact durations.
Negative circle clearance is envelope overlap, not confirmed mesh contact.
"""
import copy
import math


class ClearanceObservations:
    def __init__(self, policy='record', reference_m=.1, max_interval_s=.25):
        if policy not in ('record', 'strict'):
            raise ValueError('unknown clearance policy')
        if not math.isfinite(reference_m) or reference_m < 0 or not math.isfinite(max_interval_s) or max_interval_s <= 0:
            raise ValueError('invalid clearance reporting settings')
        self.policy = policy
        self.reference_m = reference_m
        self.max_interval_s = max_interval_s
        self.pairs = {}

    def observe(self, person, robot, stamp_ns, clearance_m):
        if not math.isfinite(clearance_m) or stamp_ns < 0:
            raise ValueError('invalid clearance observation')
        key = (person, robot)
        p = self.pairs.setdefault(key, dict(samples=0, duplicate_samples=0,
            duplicate_below_reference_samples=0, duplicate_envelope_overlap_samples=0, last_stamp_ns=None,
            minimum_clearance_m=None, unobserved_interval_s=0., below_reference=[], envelope_overlap=[]))
        last = p['last_stamp_ns']
        if last is not None and stamp_ns < last:
            raise ValueError('clearance observation clock rewind')
        p['minimum_clearance_m'] = clearance_m if p['minimum_clearance_m'] is None else min(p['minimum_clearance_m'], clearance_m)
        dt = 0. if last is None else (stamp_ns-last)/1e9
        if last == stamp_ns:
            p['duplicate_samples'] += 1
            p['duplicate_below_reference_samples'] += int(clearance_m < self.reference_m)
            p['duplicate_envelope_overlap_samples'] += int(clearance_m < 0.)
            return
        gap = dt > self.max_interval_s
        if gap: p['unobserved_interval_s'] += dt
        p['samples'] += 1; p['last_stamp_ns'] = stamp_ns
        for name, threshold in [('below_reference', self.reference_m), ('envelope_overlap', 0.)]:
            episodes = p[name]
            active = episodes[-1] if episodes and episodes[-1]['open'] else None
            if active and (gap or clearance_m >= threshold):
                active['open'] = False
                active['end_reason'] = 'sampling_gap' if gap else 'observed_exit'
                active = None
            if clearance_m < threshold:
                if active is None:
                    active = dict(start_ns=stamp_ns, last_observed_ns=stamp_ns,
                        observed_contiguous_s=0., samples=0, minimum_clearance_m=clearance_m,
                        open=True, end_reason='record_end', left_censored=last is None or gap)
                    episodes.append(active)
                else:
                    active['observed_contiguous_s'] += dt
                active['last_observed_ns'] = stamp_ns; active['samples'] += 1
                active['minimum_clearance_m'] = min(active['minimum_clearance_m'], clearance_m)

    def strict_failure(self):
        return self.policy == 'strict' and any(
            p['minimum_clearance_m'] is not None and p['minimum_clearance_m'] < self.reference_m
            for p in self.pairs.values())

    def report(self):
        rows = []
        for (person, robot), values in sorted(self.pairs.items()):
            p = copy.deepcopy(values)
            for name in ('below_reference', 'envelope_overlap'):
                p[name+'_observed_s'] = sum(e['observed_contiguous_s'] for e in p[name])
                p[name+'_samples'] = sum(e['samples'] for e in p[name])
                for e in p[name]: e['right_censored'] = e.pop('open') or e['end_reason'] == 'sampling_gap'
            rows.append(dict(person=person, robot=robot, **p))
        return dict(schema_version=1, policy=self.policy, reference_clearance_m=self.reference_m,
            reference_semantics='engineering reporting threshold, not a universal comfort boundary',
            duration_semantics='consecutive below-threshold sample endpoints; no interpolation across crossings or gaps',
            overlap_semantics='circle envelope overlap only; not proof of physical mesh contact',
            maximum_contiguous_interval_s=self.max_interval_s, pairs=rows)
