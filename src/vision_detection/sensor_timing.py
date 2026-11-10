"""Model a single busy inference pipeline on the simulation clock.

Webots waits for synchronous controllers. Without a delayed result buffer,
expensive inference would freeze the world and appear to have zero latency.
This buffer withholds detections until their measured processing time plus
configured acquisition delay has elapsed in SIMULATION time.
"""


class DelayedResult:
    """Withhold one result until its simulated acquisition/processing deadline."""

    def __init__(self):
        """Start with an idle inference slot."""
        self.pending = None

    def submit(self, capture_time, delay_s, payload):
        """Schedule a single captured observation; reject an accidental backlog."""
        if self.pending is not None:
            raise RuntimeError("Inference pipeline is busy")
        if delay_s < 0:
            raise ValueError("Processing delay must be nonnegative")
        self.pending = (capture_time + delay_s, capture_time, payload)

    def poll(self, timestamp):
        """Return and clear a ready result, or None while it is still processing."""
        if self.pending is None or timestamp < self.pending[0]:
            return None
        _, capture_time, payload = self.pending
        self.pending = None
        return capture_time, payload


def limited_speed(previous, target, dt, acceleration, deceleration):
    """Apply measured/test braking assumptions to forward-only motor commands."""
    step = (acceleration if target > previous else deceleration) * dt
    return (
        min(target, previous + step)
        if target > previous
        else max(target, previous - step)
    )
