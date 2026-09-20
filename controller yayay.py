"""Speed Control: your cruise controller.  This is the file you edit.

Work through the parts in order (see README.md).  After each part run

    python check.py part1      (or part2, part3, part4)

and when every check passes, `python run.py` drives the full scenarios.

Every tick (50 times a second) the simulator calls
``SpeedController.compute(state, target)``:

* ``state`` is a CarState from localization.  You mostly need ``state.v``
  (measured speed, m/s, a little noisy) and ``state.pitch`` (road slope in
  radians, positive uphill).
* ``target`` is a SpeedTarget from the planner:
    target.v              the speed the plan wants right where you are (m/s)
    target.a              how fast target.v is changing as you drive (m/s^2)
    target.stop_distance  how far ahead the next stop point is (m), or None

and you return a CarTBS: a throttle request ``t`` (0 to 5 m/s^2) and a brake
request ``b`` (-10 to 0 m/s^2).  Ignore ``s`` (steering); the road is straight.

The glue in ``compute`` is already written.  You fill in the TODOs.
"""

from __future__ import annotations

import math  # you will want math.sin in Part 3
import numbers

from common.messages import CarState, CarTBS

DT = 0.02  # [s] time between calls
GRAVITY = 9.81  # [m/s^2]
ROLLING_DECEL = 0.15  # [m/s^2] rolling resistance of the car


# =============================================================================
# Part 1: from "how hard to accelerate" to throttle and brake
# =============================================================================


def to_tbs(accel: float) -> CarTBS:
    if accel > 0:
        return CarTBS(t=min(accel, 5.0), b=0.0)
    elif accel < 0:
        return CarTBS(t=0.0, b=max(accel, -10.0))
    else:
        return CarTBS(t=0, b=0)


class SpeedController:
    def __init__(self):
        # TODO (Part 1): choose a proportional gain.  Units: (m/s^2) per (m/s).
        self.kp = 2
        # TODO (Part 2): choose an integral gain.  Units: (m/s^2) per (m).
        self.ki = 0.366
        self.integral = 0.0  # accumulated speed error [m], used in Part 2
        # Add anything else you want to remember between calls here.

    # -------------------------------------------------------------------------
    # Part 1: proportional control
    # -------------------------------------------------------------------------

    def p_term(self, error: float) -> float:
        return error * self.kp

    # -------------------------------------------------------------------------
    # Part 2: integral control
    # -------------------------------------------------------------------------
    def i_term(self, error: float) -> float:
        self.integral += error * DT
        if -3 < self.integral < 3:
            return self.integral * self.ki
        else:
            self.integral = max(min(self.integral, 2.9), 2.9)
            return self.integral * self.ki

        """Integral term: remove the error that P control leaves behind.

        With P control alone the car settles a little below the target speed,
        because something (rolling resistance) keeps pushing back, and P only
        pushes when there is an error.  The integral adds up the error over
        time (``self.integral += error * DT``) so that even a small error that
        lasts a long time produces a push.

        The trap is "windup": if the error is large for a long time (for
        example while the car is accelerating as hard as it can), the
        integral grows huge and the car overshoots badly later.  Limit it:
        clamp ``self.integral`` to a sensible range (the simplest fix), or
        only add to it while the error is small.

        Return ``self.ki * self.integral``.
        """

    # -------------------------------------------------------------------------
    # Part 3: feedforward
    # -------------------------------------------------------------------------
    def feedforward(self, target, state: CarState) -> float:
        if target.a > 0:
            target.a += ROLLING_DECEL
        if state.pitch > 0:
            target.a += GRAVITY * math.sin(state.pitch)
        return target.a
        """Feedforward: the acceleration you need even when your speed is perfect.

        Feedback (P and I) only reacts to errors.  If you already know a
        force is coming, you can cancel it before it causes an error:
          * the plan itself wants ``target.a`` (speeding up or slowing down),
          * on a hill, gravity pulls back with ``GRAVITY * sin(state.pitch)``,
          * rolling resistance slows a moving car by about ``ROLLING_DECEL``.
        Return the sum of the pieces you think belong here.
        """

    # -------------------------------------------------------------------------
    # Part 4: stopping on the line
    # -------------------------------------------------------------------------
    def stop_logic(self, accel: float, state: CarState, target) -> float:
        d = target.stop_distance
        v = state.v
        if d is None:
            return accel
        # still moving
        if 5 < d:
            return accel
        # short
        if 0.3 <= d <= 5:
            accel = -(v**2 / (2 * d)) + GRAVITY * math.sin(state.pitch) + ROLLING_DECEL
            return accel
        # stopped
        if 0 <= d <= 0.3:
            self.integral = 0.0
            accel = -1.5
            return accel
        self.integral = 0.0
        return accel
        """Take over near a stop so the car stops ON the line.  Return the request.

        ``target.stop_distance`` is how far the stop point is (None if there
        is none nearby).  It becomes NEGATIVE once you are past the point, so
        guard any division by it.  The speed target already slows you down,
        but the brakes respond 0.4 s late, so plain PID stops early or late.
        Close to the line (say within 5 m) there are three cases.  "Stopped"
        means your measured speed is below about 0.15 m/s: the speed sensor
        is noisy, so it never reads exactly 0.

          1. Still moving: slow down exactly as hard as needed to stop at the
             line.  From speed v with distance d left (d > 0), the car must
             slow at v^2 / (2 d).  Gravity and rolling resistance already do part of
             that (just like in Part 3), so the request is
                 -v**2 / (2*d) + GRAVITY * sin(pitch) + ROLLING_DECEL
          2. Stopped, but short of the line (more than ~0.3 m away): creep
             forward gently: a little more than it takes to hold the car
             still, e.g. 0.5 + GRAVITY * sin(pitch) + ROLLING_DECEL.
          3. Stopped at the line: HOLD the brake (a small negative request,
             e.g. -1.5 m/s^2) so you do not roll, and reset your integral so
             it does not wind up while you wait.

        Far from a stop, return ``accel`` unchanged.  When the stop is
        released, ``target.stop_distance`` jumps to the next stop and you
        drive off by yourself.
        """

    # -------------------------------------------------------------------------
    # The glue (already written).  You may change it, but you should not need to.
    # -------------------------------------------------------------------------
    def compute(self, state: CarState, target) -> CarTBS:
        error = target.v - state.v
        accel = _if_done(self.p_term, error, default=None)  # Part 1
        if accel is None:
            raise NotImplementedError("Part 1: p_term() is not written yet")
        accel += _if_done(self.i_term, error)  # Part 2
        accel += _if_done(self.feedforward, target, state)  # Part 3
        accel = _if_done(self.stop_logic, accel, state, target, default=accel)  # Part 4
        return to_tbs(accel)


def _if_done(fn, *args, default=0.0):
    """Call a TODO function; until it is written, use `default` instead."""
    try:
        value = fn(*args)
    except NotImplementedError:
        return default

    if not isinstance(value, numbers.Real) or not math.isfinite(value):
        raise TypeError(
            f"{fn.__name__}() returned {value!r}; it must be a number (m/s^2)"
        )

    return value