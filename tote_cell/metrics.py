"""Shift-level KPIs for tote handling, computed from a timestamped event log.

The KPIs mirror what an industrial pilot reports (throughput, pick-and-place
success, continuous autonomous operation), plus falls, which matter for a
legged robot and not for a wheeled one.
"""

from dataclasses import dataclass, field

EVENT_KINDS = {
    "pick_attempt",       # robot starts a grasp on a tote
    "pick_success",       # tote lifted clear of the stack
    "place_success",      # tote at rest in the place zone (tasks.tote_placed)
    "drop",               # tote ended on the floor (tasks.tote_dropped)
    "fall",               # robot fell (tasks.robot_fallen)
    "intervention",       # a reset or manual help was needed
}


@dataclass
class Event:
    t: float              # simulation time in seconds
    kind: str
    tote: str = ""

    def __post_init__(self):
        if self.kind not in EVENT_KINDS:
            raise ValueError(f"unknown event kind {self.kind!r}")


@dataclass
class EventLog:
    events: list = field(default_factory=list)

    def add(self, t, kind, tote=""):
        self.events.append(Event(t, kind, tote))

    def count(self, kind):
        return sum(e.kind == kind for e in self.events)


def summarize(log, duration_s):
    """KPIs for a run of `duration_s` seconds of simulated time."""
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    attempts = log.count("pick_attempt")
    picks = log.count("pick_success")
    places = log.count("place_success")

    # Longest stretch without an intervention or a fall
    stops = sorted(e.t for e in log.events if e.kind in ("intervention", "fall"))
    edges = [0.0, *stops, duration_s]
    longest_autonomous = max(b - a for a, b in zip(edges, edges[1:]))

    return {
        "duration_min": duration_s / 60,
        "tote_moves_per_hour": places / duration_s * 3600,
        "pick_success_rate": picks / attempts if attempts else None,
        "pick_and_place_success_rate": places / attempts if attempts else None,
        "drops": log.count("drop"),
        "falls": log.count("fall"),
        "interventions": log.count("intervention"),
        "longest_autonomous_min": longest_autonomous / 60,
    }
