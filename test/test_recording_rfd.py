"""Record page 'RFD data': radio submerge test rows go into a recording package only when ticked."""
import csv
import io
import json
import shutil
import tarfile
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from robotx_graey_2026.api.gui.recording_service import RecordingService
from robotx_graey_2026.api.navigation.radio_submerge import (append_radio_rows, copy_radio_rows,
    radio_log_path, write_radio_header)

RUN_A, RUN_B = 'a'*32, 'b'*32


class FakeBag:
    def __init__(self):self.recording=False
    def start(self,path,topics):self.recording=True;return True,'started'
    def stop(self):self.recording=False;return True,'closed'
    def status(self,*args):return dict(recording=self.recording,error='')
    def tick(self):pass


def row(wall, number):
    return dict(time_utc=datetime.fromtimestamp(wall, timezone.utc).isoformat(timespec='milliseconds'),
                t_s=1.0, kind='heard', stage='DOWN_1', msg_number=number,
                msg_text='the boat is sending this msg %d' % number, raw_depth_m=10.5,
                relative_depth_m=.1, cube_z_m='', cube_relative_m='', press_abs_hpa=1023.)


class RfdRecordingTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root, self.logs = Path(folder.name)/'recordings', Path(folder.name)/'logs'
        self.logs.mkdir()
        self.service = RecordingService(self.root, request=self.no_service, bag=FakeBag(),
            free=lambda: 5*1024**3, start_worker=False,
            persistence=lambda _: ('/fixture', 'synthetic', True), radio_log_dir=self.logs)
        self.topics = [('/graey/dvl/velocity', ['geometry_msgs/msg/TwistWithCovarianceStamped'])]
        self.mission(None)

    def no_service(self, *args):
        raise AssertionError('RFD data must not ask any camera/sonar service')

    def mission(self, run):
        data = dict(run_id=run, plan=dict(kind='radio_submerge')) if run else {}
        self.service.update_sources(self.topics, dict(streams=dict(waypoint=dict(age=.1, data=data))))

    def log(self, run, *rows):
        path = radio_log_path(self.logs, run)
        if not path.exists():
            write_radio_header(path)
        append_radio_rows(path, list(rows))

    def start(self, **selection):
        body = dict(action='start', name='rfd_test', camera=False, sonar=False, sensors=False, topics=[])
        body.update(selection)
        self.service.submit(body)
        self.service.start(self.service.commands.get_nowait())
        return self.service.session

    def saved(self, run):
        s = self.service.session
        path = self.root/s['name']/'rfd'/s['owner_token']/(run+'.csv')
        with path.open(newline='', encoding='utf-8') as f:
            return [int(r['msg_number']) for r in csv.DictReader(f)]

    def package(self):
        out = io.BytesIO()
        self.service.archive_into(self.service.session['name'], out)
        with tarfile.open(fileobj=io.BytesIO(out.getvalue()), mode='r:gz') as tar:
            return tar.getnames()

    def test_only_rows_logged_while_recording_go_in_the_package(self):
        s = self.start(rfd=True)
        self.assertTrue(s['active'])
        self.assertTrue(s['sources']['rfd']['waiting'])
        t0 = s['started_wall']
        self.mission(RUN_A)
        self.log(RUN_A, row(t0-10, 1), row(t0+1, 2), row(t0+2, 3))     # 1 was before Start recording
        self.service.sample()
        self.assertEqual(self.saved(RUN_A), [2, 3])
        self.log(RUN_A, row(t0+3, 4))
        self.service.sample()
        self.assertEqual(self.saved(RUN_A), [2, 3, 4])
        self.log(RUN_A, row(t0+4, 5))
        self.service.stop()                                             # Stop copies up to the end
        self.assertEqual(self.saved(RUN_A), [2, 3, 4, 5])
        source = self.service.session['sources']['rfd']
        self.assertEqual((source['rows'], source['acquired'], source['recording']), (4, True, False))
        self.assertFalse(self.service.session['active'])
        manifest = json.loads((self.root/s['name']/'session.json').read_text(encoding='utf-8'))
        self.assertEqual(manifest['sources']['rfd']['runs'], [RUN_A])
        self.assertIn('%s/rfd/%s/%s.csv' % (s['name'], s['owner_token'], RUN_A), self.package())

    def test_package_refused_when_saved_rfd_rows_are_missing(self):
        s = self.start(rfd=True)
        self.mission(RUN_A)
        self.log(RUN_A, row(s['started_wall']+1, 1))
        self.service.stop()
        shutil.rmtree(self.root/s['name']/'rfd')
        with self.assertRaises(ValueError):
            self.package()

    def test_each_test_in_one_session_gets_its_own_file(self):
        s = self.start(rfd=True)
        t0 = s['started_wall']
        self.mission(RUN_A)
        self.log(RUN_A, row(t0+1, 1))
        self.service.sample()
        self.mission(RUN_B)
        self.log(RUN_B, row(t0+2, 7))
        self.log(RUN_A, row(t0+2, 2))                                   # A's last rows still arrive
        self.service.sample()
        self.assertEqual((self.saved(RUN_A), self.saved(RUN_B)), ([1, 2], [7]))
        self.assertEqual(self.service.session['sources']['rfd']['runs'], [RUN_A, RUN_B])

    def test_waits_for_a_test_and_packages_without_rfd_rows(self):
        s = self.start(rfd=True)
        self.service.sample()
        source = self.service.session['sources']['rfd']
        self.assertEqual((source['rows'], source['waiting'], source['acquired']), (0, True, False))
        self.service.stop()
        self.assertFalse(any('/rfd/' in name for name in self.package()))

    def test_unticked_rfd_puts_nothing_in_the_session(self):
        s = self.start(sensors=True, topics=['/graey/dvl/velocity'])
        self.mission(RUN_A)
        self.log(RUN_A, row(s['started_wall']+1, 1))
        self.service.sample()
        self.service.stop()
        self.assertNotIn('rfd', self.service.session['sources'])
        self.assertFalse((self.root/s['name']/'rfd').exists())

    def test_rfd_selection_must_be_a_checkbox_value(self):
        with self.assertRaises(ValueError):
            self.service.submit(dict(action='start', name='x', rfd='yes'))
        with self.assertRaises(ValueError):
            self.service.submit(dict(action='start', name='x', rfd=False))   # nothing selected


class RecordTabTests(unittest.TestCase):
    """The laptop Record tab's 'record' operation reaches /api/recording with the RFD choice."""

    def test_rfd_choice_is_passed_through_like_sonar(self):
        from robotx_graey_2026.api.gui.fleet_adapter import FleetAdapter
        sent = []
        def request(header, body):
            sent.append((header['path'], json.loads(body) if body else None))
            return 200, 'application/json', b'{"ok": true, "msg": "accepted"}'
        adapter = FleetAdapter(request)
        adapter.operation(dict(op='record', action='start', camera=False, sonar=True, rfd=True, topics=[]))
        adapter.operation(dict(op='record', action='start', camera=False, sonar=True, topics=[]))
        self.assertEqual([(path, body['sonar'], body['rfd']) for path, body in sent],
                         [('/api/recording', True, True), ('/api/recording', True, False)])


class CopyRowsTests(unittest.TestCase):
    def test_first_copy_skips_rows_from_before_the_session_then_follows_the_cursor(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        src, dst = Path(folder.name)/'src.csv', Path(folder.name)/'out'/'dst.csv'
        dst.parent.mkdir()
        write_radio_header(src)
        append_radio_rows(src, [row(100., 1), row(200., 2)])
        self.assertEqual(copy_radio_rows(src, dst, None, since_wall=150.), (2, 1))
        append_radio_rows(src, [row(50., 3)])                           # later rows are never time-filtered
        self.assertEqual(copy_radio_rows(src, dst, 2, since_wall=150.), (3, 1))
        self.assertEqual(copy_radio_rows(src, dst, 3, since_wall=150.), (3, 0))


if __name__ == '__main__':
    unittest.main()
