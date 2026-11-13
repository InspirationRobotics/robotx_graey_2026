import threading
import unittest

from robotx_graey_2026.api.navigation.onboard_recording import OnboardMissionRecording


class OnboardMissionRecordingTests(unittest.TestCase):
    def test_camera_and_sonar_start_at_se_and_stop_in_order(self):
        calls=[]
        stopped=threading.Event()
        def request(command):
            calls.append(command)
            if command is None:
                return {'session': {'name':'fixture','sources': {
                    'camera': {'recording':True},
                    'sonar': {'recording':True}},'errors':[]}}
            if command['action']=='stop':stopped.set()
            return {'ok':True,'msg':command['action']}
        recorder=OnboardMissionRecording('abc123',lambda _:None,request=request)
        self.assertFalse(recorder.snapshot()['requested'])
        recorder.start()
        self.assertTrue(recorder.started_event.wait(2))
        state=recorder.snapshot()
        self.assertTrue(state['camera'] and state['sonar'] and state['recording'])
        self.assertEqual(calls[0],dict(action='start',name='rc_dive_abc123',
            camera=True,sonar=True,sensors=False,topics=[]))
        recorder.stop();recorder.close(2)
        self.assertTrue(stopped.is_set())
        self.assertEqual(calls[-1],dict(action='stop'))
        self.assertTrue(recorder.snapshot()['stopped'])

    def test_failure_never_prevents_stop_or_blocks_caller(self):
        calls=[]
        def request(command):
            calls.append(command)
            if command and command['action']=='start':raise OSError('camera offline')
            return {'ok':True,'msg':'stopped'}
        recorder=OnboardMissionRecording('run',lambda _:None,request=request)
        recorder.start()
        self.assertTrue(recorder.started_event.wait(2))
        self.assertIn('camera offline',recorder.snapshot()['errors'])
        recorder.close(2)
        self.assertEqual(calls[-1],dict(action='stop'))


if __name__ == '__main__':unittest.main()
