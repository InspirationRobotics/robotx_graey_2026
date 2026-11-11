"""Runner for the radio submerge test, built on the UNCHANGED WaypointMission.

WaypointMission (the GUI mission runner) is used exactly as written, so the radio test gets the
same arming, control-watchdog, pilot-takeover, abort and GUIDED command path as 5-2-5. This
subclass only:
  * swaps in RadioSubmergeRun as the mission logic,
  * taps the frames WaypointMission already receives for the boat's radio lines, the Bar30
    pressure and the Cube's depth (listen only: nothing extra is sent),
  * adds the pressure reading to the observation the logic sees,
  * appends the log rows to the CSV each tick.

gui_node starts it for the 'radio_submerge_start' action:
    python3 -m robotx_graey_2026.api.navigation.radio_submerge_mission --ros-args -p plan_json:=...
It holds the same 'waypoint_mission' process lock, so it can never run beside another mission.
"""
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.navigation.waypoint_mission import WaypointMission
from robotx_graey_2026.api.navigation.simple_sequence import SimpleSequenceRun
from robotx_graey_2026.api.navigation.radio_submerge import (RadioSubmergeRun, RadioLog, BOAT_SYSID,
    BOAT_TEXT_TYPE, radio_log_path, write_radio_header, append_radio_rows, tap_drain)


class RadioSubmergeMission(WaypointMission):

    def __init__(self):
        self._choosing_logic = True
        super().__init__()
        if type(self.logic) is not SimpleSequenceRun:
            raise RuntimeError('WaypointMission did not build the expected 5-2-5 base logic')
        # Replace the logic before any timer has run: nothing has been commanded yet.
        self.logic = RadioSubmergeRun({}, self.logic.run_id, time.monotonic())
        self.radio_log = RadioLog(time.monotonic())
        self.write_error = self.tap_error = None
        self.cube_z = (-math.inf, None)
        self.radio_csv = radio_log_path(Path(self.result_file).parent if self.result_file
                                        else Path.home()/'robotx_ws/logs', self.logic.run_id)
        write_radio_header(self.radio_csv)
        tap_drain(self.link, self.radio_receive, self.radio_failed)

    def get_parameter(self, name):
        # WaypointMission picks its logic from the plan kind and only knows its own kinds. While
        # it starts up, show it the 5-2-5 kind so it builds that (same GUIDED/arming path);
        # __init__ then swaps in the radio test. Every later read returns the real parameter.
        parameter = super().get_parameter(name)
        if name != 'plan_json' or not getattr(self, '_choosing_logic', False):
            return parameter
        self._choosing_logic = False
        plan = json.loads(parameter.value or '{}')
        if plan.get('kind') != 'radio_submerge':
            raise ValueError('radio_submerge_mission only runs the radio submerge test')
        return SimpleNamespace(value=json.dumps(dict(plan, kind='simple_sequence')))

    def radio_failed(self, exc):
        # A frame the log could not read; kept so the status shows it. Logged once per kind.
        if self.tap_error != str(exc):
            self.get_logger().error('RADIO_LOG could not read a frame: %s' % exc)
        self.tap_error = str(exc)

    def latest_cube_z(self, now):
        stamp, z = self.cube_z
        return z if 0 <= now-stamp < .5 else None

    def radio_receive(self, kind, m):
        now = time.monotonic()
        # The boat's lines reach us through the Cube's TELEM2 radio.
        if kind == 'TUNNEL' and m.get_srcSystem() == BOAT_SYSID and m.payload_type == BOAT_TEXT_TYPE:
            self.radio_log.boat_line(bytes(m.payload)[:m.payload_length], now, self.logic,
                                     self.latest_cube_z(now))
        elif m.get_srcSystem() == 1 and m.get_srcComponent() == 1:
            if kind == 'SCALED_PRESSURE3':          # the configured Bar30 stream
                self.radio_log.pressure_sample(m.press_abs, now)
            elif kind == 'LOCAL_POSITION_NED':
                self.cube_z = (now, m.z)

    def observation(self, now):
        o = super().observation(now)
        o['pressure_hpa'] = self.radio_log.pressure_hpa(now)
        return o

    def update_radio_log(self, now):
        """Depth row every 0.5 s while the test runs; append everything pending to the CSV.
        A failed write is reported in the status, never allowed to stop the mission loop."""
        if self.logic.stage not in self.logic.terminal:
            self.radio_log.tick(now, self.logic, self.latest_cube_z(now))
        rows = self.radio_log.rows()
        if rows:
            try:
                append_radio_rows(self.radio_csv, rows)
                self.write_error = None
            except OSError as exc:
                if self.write_error != str(exc):
                    self.get_logger().error('RADIO_LOG write failed: %s' % exc)
                self.write_error = str(exc)
        self.logic.radio_status = dict(log_file=str(self.radio_csv),
                                       log_error=self.write_error or self.tap_error,
                                       **self.radio_log.summary(now))

    def tick(self):
        self.update_radio_log(time.monotonic())
        super().tick()


def main():
    run(RadioSubmergeMission, ownership='waypoint_mission')


if __name__ == '__main__':
    main()
