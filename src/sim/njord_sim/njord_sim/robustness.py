"""Explicit synthetic perturbations; these are not measured vessel statistics."""

import math


def opponent_motion(t, meeting, variable_speed=False):
    """Return displacement and its analytic derivative along the NPC heading."""
    speed = 1.2861111111
    if not variable_speed:
        return (t - meeting) * speed, speed
    # Bounded 0.886..1.686 m/s, with continuous acceleration/deceleration.
    omega = 2 * math.pi / 30.0
    return ((t - meeting) * speed + 0.4 / omega * (1 - math.cos(omega * t)),
            speed + 0.4 * math.sin(omega * t))


def detection_missing(t, period, duration):
    """Delay the first outage by one period, then hide the NPC periodically."""
    return period > 0 and t >= period and t % period < duration
