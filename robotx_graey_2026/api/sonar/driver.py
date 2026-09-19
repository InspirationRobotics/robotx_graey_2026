"""Piece 7: the state machine. Decides what the sub does.

It never sees a blob. It only sees offset, span, trend and staleness - the
things Memory reports. That separation is what stops a change in the detector
from breaking the behaviour, and vice versa.

It does not command the vehicle either. It returns an ACTION describing what
should happen, and a thin ROS wrapper carries it out. That keeps this file free
of rospy and rclpy, so the same file runs on Onyx and on Graey.

    SEARCH    don't know where it is. Yaw through headings, sweep at each.
    APPROACH  know where it is, but not standing over it. Strafe across.
    ORIENT    standing over it. Turn to run along it.
    FOLLOW    lined up. Move, stay centred, watch for bends.
    LOST      was following, isn't now. Widen out and try to re-acquire.

APPROACH exists because of a geometry trap. Turning to point along the pipe
puts the pipe fore-aft - which is the sonar's blind direction. That is fine
once you are directly over it, because the bit beside you still shows. It is
useless if the pipe is several metres away, because then it is ahead of you and
invisible. So you have to close the distance sideways FIRST, while it is still
broadside and clearly visible, and only then turn.
"""
from . import detect as D
from . import library as L
from . import settings as S
from .memory import Memory

SEARCH, APPROACH, ORIENT, FOLLOW, LOST, DONE = (
    "SEARCH", "APPROACH", "ORIENT", "FOLLOW", "LOST", "DONE")

# actions handed back to whatever is driving the vehicle
YAW_TO = "yaw_to"        # absolute heading, degrees
YAW_BY = "yaw_by"        # relative turn, degrees, positive to starboard
FORWARD = "forward"      # metres
STRAFE = "strafe"        # metres sideways, positive to starboard
HOLD = "hold"
FINISHED = "finished"

# which sweep settings each state uses
_SWEEP_FOR = {SEARCH: "SEARCH", APPROACH: "APPROACH", ORIENT: "ORIENT",
              FOLLOW: "FOLLOW", LOST: "LOST", DONE: "FOLLOW"}


class Action:
    def __init__(self, kind, value=None, why=""):
        self.kind = kind
        self.value = value
        self.why = why

    def __repr__(self):
        v = "" if self.value is None else f" {self.value:+.2f}"
        return f"Action({self.kind}{v}: {self.why})"


class Driver:
    """Drives the sub towards whatever object you name.

    target is the name of an object in the library - "pvc_pipe", "pipeline" -
    or any of the other forms library.resolve() takes. Nothing about a pipeline
    is written into this file; swapping the hunt to a different object is
    swapping the string.

    The Driver does not score anything itself. It carries the resolved profile
    so there is ONE source of truth: run perceive(sweep, driver.profile) and the
    state machine cannot be steering towards a target the detector was never
    looking for. That mismatch is silent and would look exactly like a broken
    detector.
    """

    def __init__(self, target, memory=None, start_heading_deg=0.0,
                 library_path=None):
        self.target = target
        self.profile = L.resolve(
            target, **({"path": library_path} if library_path else {}))
        if self.profile is None:
            # Without a profile perceive() never fills in p.best, so every state
            # would miss on every sweep and the sub would search for ever while
            # printing that it found nothing. Better to say so at the door.
            raise ValueError(
                "the Driver needs a target - it decides where the sub goes, and "
                "with nothing to score against it will search for ever. Name an "
                "object from the library, or pass ideals like "
                "'height_m=1.5,brightness=140'.")

        self.memory = memory or Memory()
        self.state = SEARCH

        # The driver's own progress through a yaw scan. This is bookkeeping
        # about what it has done, not knowledge about the target, so it lives
        # here rather than in Memory.
        self._scan_headings = self._plan_scan(start_heading_deg)
        self._scan_index = 0
        self._lost_attempts = 0
        self._broadside_heading = None
        self._broadside_span = None
        self._align_tries = 0

    @staticmethod
    def _plan_scan(start_heading_deg):
        """Headings to visit during a search.

        Only 180 degrees, not 360: the scan plane reaches out both sides of the
        sub, so half a turn already points it at every horizontal bearing.
        Steps are no wider than the beam's fore-aft thickness, or wedges of
        water go unswept between stops.
        """
        n = int(round(S.SEARCH_YAW_SPAN_DEG / S.SEARCH_YAW_STEP_DEG))
        return [(start_heading_deg + i * S.SEARCH_YAW_STEP_DEG) % 360.0
                for i in range(n)]

    def sweep_state(self):
        return _SWEEP_FOR.get(self.state, "SEARCH")

    def tick(self, perception, heading_deg):
        handler = {SEARCH: self._search, APPROACH: self._approach,
                   ORIENT: self._orient, FOLLOW: self._follow,
                   LOST: self._lost}.get(self.state)
        if handler is None:
            return Action(FINISHED, why="done")
        return handler(perception, heading_deg)

    # -------------------------------------------------------------- states
    def _search(self, p, heading_deg):
        self.memory.record_scan_point(heading_deg, p.best)
        self._scan_index += 1

        if self._scan_index < len(self._scan_headings):
            return Action(YAW_TO, self._scan_headings[self._scan_index],
                          f"search {self._scan_index + 1}/{len(self._scan_headings)}")

        best = self.memory.best_scan_heading()
        self.memory.clear_scan()
        self._scan_index = 0

        if best is None:
            return Action(HOLD, why="full scan found nothing - wrong place?")

        self._broadside_heading, det = best
        self._broadside_span = det.span_deg
        self.state = APPROACH
        return Action(YAW_TO, self._broadside_heading,
                      f"widest return {det.span_deg:.0f} deg there - going back to it")

    def _approach(self, p, heading_deg):
        """Hold the broadside heading and slide sideways until overhead.

        Sideways, not forward: at this heading the pipe is beside us and fully
        visible, and it stays visible the whole way across. Turning to face it
        first would hide it in the fore-aft blind zone for the entire approach.
        """
        if not p.found:
            self.memory.record_miss()
            if self.memory.stale:
                self.state = SEARCH
                self.memory.forget()
                return Action(YAW_TO, self._scan_headings[0],
                              "lost it during approach - searching again")
            return Action(HOLD, why=f"missed sweep {self.memory.misses}")

        self.memory.record(p.best, heading_deg)
        offset = p.best.offset_m

        if abs(offset) <= S.APPROACH_CENTRED_M:
            self.state = ORIENT
            self._align_tries = 0
            return Action(YAW_BY, 90.0,
                          f"overhead (offset {offset:+.2f} m) - turning to run along it")

        step = min(S.APPROACH_STEP_M, abs(offset))
        return Action(STRAFE, step if offset > 0 else -step,
                      f"offset {offset:+.2f} m - sliding across")

    def _orient(self, p, heading_deg):
        """Confirm we now run along the pipe: the span should have collapsed."""
        if not p.found:
            self.memory.record_miss()
            if self.memory.stale:
                self.state = SEARCH
                self.memory.forget()
                return Action(YAW_TO, self._scan_headings[0],
                              "lost it while turning - searching again")
            return Action(HOLD, why="confirming alignment")

        self.memory.record(p.best, heading_deg)

        # Judge alignment by how far the span has DROPPED from its broadside
        # value, not against a fixed number of degrees. Span is a relative
        # signal - it depends on range, on how much pipe is in view, and on the
        # fact that a zigzag has no single direction - so an absolute threshold
        # gets stuck. Half the broadside width means the turn did its job.
        aligned = (p.best.orientation == D.ALONG or
                   (self._broadside_span and
                    p.best.span_deg <= self._broadside_span * S.ALIGNED_SPAN_FRACTION))
        if aligned:
            self.state = FOLLOW
            return Action(FORWARD, S.FOLLOW_STEP_M,
                          f"aligned, span {p.best.span_deg:.0f} deg "
                          f"(was {self._broadside_span:.0f}) - following")

        self._align_tries += 1
        if self._align_tries >= 2:
            # Two turns and still broadside means the widest-span heading was
            # wrong. Rescan rather than keep spinning.
            self.state = SEARCH
            self.memory.forget()
            return Action(YAW_TO, self._scan_headings[0],
                          "alignment failed twice - searching again")

        # 90 degrees the other way from broadside also runs along the pipe.
        return Action(YAW_BY, 180.0,
                      f"span still {p.best.span_deg:.0f} deg - trying the other way")

    def _follow(self, p, heading_deg):
        if not p.found:
            self.memory.record_miss()
            if self.memory.stale:
                self.state = LOST
                self._lost_attempts = 0
                return Action(HOLD, why="lost the pipe")
            return Action(HOLD, why=f"missed sweep {self.memory.misses}")

        self.memory.record(p.best, heading_deg)

        bend = self.memory.bend()
        if bend is not None:
            # Elbows are 45 degrees, so the new heading is one of exactly two
            # options and the drift direction says which. Turn and re-check
            # rather than searching.
            turn = S.BEND_TURN_DEG if bend == "starboard" else -S.BEND_TURN_DEG
            self.memory.forget()        # old offsets belong to the old segment
            return Action(YAW_BY, turn, f"pipe bending to {bend} - taking the elbow")

        if not self.memory.centred:
            offset = self.memory.offset
            step = min(S.APPROACH_STEP_M, abs(offset))
            return Action(STRAFE, step if offset > 0 else -step,
                          f"offset {offset:+.2f} m - re-centring")

        return Action(FORWARD, S.FOLLOW_STEP_M,
                      f"on track, offset {self.memory.offset:+.2f} m")

    def _lost(self, p, heading_deg):
        """Widen out gradually. Usually it is a metre away, not gone."""
        if p.found:
            self.memory.record(p.best, heading_deg)
            self.state = FOLLOW
            return Action(HOLD, why="re-acquired")

        self._lost_attempts += 1
        if self._lost_attempts >= 3:
            self.state = SEARCH
            self._scan_index = 0
            self.memory.forget()
            return Action(YAW_TO, self._scan_headings[0],
                          "re-acquire failed - full search")
        return Action(HOLD, why=f"re-acquiring {self._lost_attempts}/3")


def explain(perception):
    """Turn a perception's failure reason into something worth printing.

    Four genuinely different problems. Collapsing them into "no result" throws
    away exactly the information you need when something goes wrong in water
    you cannot see into.
    """
    return {
        D.OK: "ok",
        D.NO_FLOOR: "cannot find the bottom - too shallow, too steep, or nothing reflecting",
        D.NOTHING_ABOVE_FLOOR: "bottom found, nothing above it - probably the wrong place",
        D.NOTHING_SCORED: "found something, but nothing matching the target",
    }.get(perception.reason, perception.reason)
