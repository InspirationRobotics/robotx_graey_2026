"""The radio page's mission buttons and Start navigation, handed to the GUI node's own paths.

Chris, 12 Nov 2026. radio_status starts nothing itself. A start or a stop that arrives over the
radio is POSTed to gui_node (/api/waypoint on 8090) exactly as its own Missions page would send
it: gui_node checks the kill switch's permission and that nothing else is running, and it is
gui_node that starts waypoint_mission. One path into a mission, whoever asks.

Every answer carries a note, one of status_codec's NOTE_REFUSED_*: the status frame has one byte
for why a start was refused, and the page turns it back into words.
"""
import json
import urllib.error
import urllib.request

from . import status_codec as codec

GUI = 'http://127.0.0.1:8090/api/waypoint'
CALIBRATION = 'http://127.0.0.1:8090/api/calibration'     # the Sensors page's Start navigation
CALIBRATION_ACTIONS = ('calibrate', 'cancel', 'align_heading')  # the Calibrate tab's; prepare is its own button
REFUSALS = (('permission unavailable', codec.NOTE_REFUSED_KILLED),      # gui_node's own words
            ('Take manual control', codec.NOTE_REFUSED_MODE),           # maintenance_safety.py
            ('already running', codec.NOTE_REFUSED_JOB),                # calibration_jobs.py
            ('Stop the mission', codec.NOTE_REFUSED_BUSY),
            ('mission is running', codec.NOTE_REFUSED_BUSY),
            ('commissioning', codec.NOTE_REFUSED_NAV),
            ('Dive', codec.NOTE_REFUSED_DEPTH),                             # radio_route.py and dive_spin.py
            ('Route', codec.NOTE_REFUSED_ROUTE))


def refusal_note(text):
    """gui_node's (or radio_route's) words for a refused start -> the status frame's one-byte note."""
    return next((note for part, note in REFUSALS if part in text), codec.NOTE_REFUSED)


def post(url, body, timeout=3.0):
    """(reply dict) from gui_node. A refusal is HTTP 409 with the reason in 'msg'."""
    request = urllib.request.Request(url, data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as r:
            return json.loads(r.read(1 << 16))
    except urllib.error.HTTPError as error:
        return json.loads(error.read(1 << 16))


class MissionRelay:
    def __init__(self, url=GUI, post=post, log=lambda text: None, calibration=CALIBRATION):
        self.url, self.post, self.log, self.calibration_url = url, post, log, calibration

    def ask(self, body, url=None):
        try:
            reply = self.post(url or self.url, body)
        except (OSError, ValueError) as error:              # gui_node is down, or not answering JSON
            self.log('mission request %s: the GUI node did not answer (%s)' % (body['action'], error))
            return False, codec.NOTE_REFUSED_NO_GUI, None
        if isinstance(reply, dict) and reply.get('ok'):
            self.log('mission request %s: accepted' % body['action'])
            return True, codec.NOTE_NONE, reply.get('run_id')
        text = str(reply.get('msg') if isinstance(reply, dict) else reply)
        self.log('mission request %s refused: %s' % (body['action'], text))
        return False, refusal_note(text), None

    def start(self, mission):
        """(ok, note, run id): a dive, route or full run (radio_route.mission_plan says what each is)."""
        return self.ask(dict(action='radio_mission_start', mission=mission))

    def start_navigation(self):
        """(ok, note): the Sensors page's Start navigation, with "floating at the surface" confirmed
        by the operator on the radio page. It zeroes the pressure depth and turns on navigation
        output to the Cube; the lat/lon map lines up on the first GPS fix (continuous_estimator.py)."""
        return self.ask(dict(action='prepare', surface_confirmed=True), self.calibration_url)[:2]

    def calibration(self, body=None):
        """The radio page's Calibrate tab over WiFi (gui_node listens on localhost only): its
        /api/calibration state (body None), or one of its buttons: (http code, reply)."""
        if body is None:
            with urllib.request.urlopen(self.calibration_url, timeout=3.0) as r:
                return 200, json.loads(r.read(4 << 20))
        if body.get('action') not in CALIBRATION_ACTIONS:
            return 400, dict(ok=False, msg='Unknown calibration action')
        try:
            reply = self.post(self.calibration_url, body, timeout=5.0)
        except (OSError, ValueError) as error:
            return 502, dict(ok=False, msg='the GUI node did not answer: %s' % error)
        self.log('calibration %s: %s' % (body['action'], reply))
        return (200 if isinstance(reply, dict) and reply.get('ok') else 409), reply

    def stop(self, run_id):
        return self.ask(dict(action='abort', run_id=run_id))[:2]
