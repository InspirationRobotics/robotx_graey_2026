import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
from robotx_graey_2026.api.navigation.pipeline_recording import PipelineRecording


class RecordingTests(unittest.TestCase):
    def test_configure_confirm_record_and_restore_without_hardware(self):
        with tempfile.TemporaryDirectory() as tmp, patch('threading.Thread.start'):
            recorder = PipelineRecording(tmp, 'a'*32)
        # The worker makes the directory; use a new temporary root for this run.
        with tempfile.TemporaryDirectory() as tmp:
            recorder.folder = Path(tmp)
            recorder.desired = True
            closed = Mock()
            closed.is_set.side_effect = [False, False, True]
            recorder.closed = closed
            calls = []
            settings = dict(map=True, range=4, sector=[180, 0])
            sonar = {}

            def request(port, path, body=None):
                calls.append((port, path, body))
                if path == '/status':
                    return json.dumps(dict(webUp=True))
                if path == '/state':
                    return json.dumps(settings)
                if path == '/mapdata':
                    return json.dumps(dict(recording=sonar or None))
                if path == '/range':
                    settings['range'] = float(body)
                if path == '/sector':
                    settings['sector'] = list(map(float, body.split()))
                if path == '/record' and body.startswith('start '):
                    sonar.update(name='rec_'+body[6:], sweeps=1)
                return 'ok'

            recorder.request = request
            proc = Mock()
            proc.poll.return_value = None

            def launch(argv, **kwargs):
                folder = Path(argv[-1])
                (folder/'camera-status.json').write_text(json.dumps(dict(frames=10, monotonic=time.monotonic())))
                return proc

            with patch('subprocess.Popen', side_effect=launch):
                recorder.worker()
            self.assertTrue(recorder.state['ready'], recorder.state)
            self.assertEqual(settings['range'], 4)
            self.assertEqual(settings['sector'], [180, 0])
            self.assertIn((8095, '/record', 'stop'), calls)
            proc.terminate.assert_called_once()

    def test_restore_uses_existing_sonar_settings_interface(self):
        recorder = PipelineRecording.__new__(PipelineRecording)
        recorder.request = Mock()
        recorder.restore_settings(dict(range=8, sector=[90, 180]))
        self.assertEqual(recorder.request.call_args_list[0].args, (8095, '/range', '8'))
        self.assertEqual(recorder.request.call_args_list[1].args, (8095, '/sector', '90 180'))
