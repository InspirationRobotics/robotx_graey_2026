import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from robotx_graey_2026.api.gui.mission_store import MissionStore,validate_mission
from robotx_graey_2026.api.gui.recording_service import RecordingService
from robotx_graey_2026.api.navigation.route_plan import validate_route
from robotx_graey_2026.api.navigation.competition_route import CompetitionRouteRun
from robotx_graey_2026.api.gui.gcs_transport import ocs_request


class TestMissionLibrary(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.store=MissionStore(self.folder.name)
        self.points=[dict(kind='waypoint',lat=1.28,lon=103.85)]
    def test_round_trip_all_three_mission_choices(self):
        for kind in ('waypoints','pipeline','simple_sequence'):
            points=self.points if kind=='waypoints' else [dict(kind='pipeline_start',lat=1.28,lon=103.85),dict(kind='pipeline_end',lat=1.281,lon=103.85)]
            self.store.save(kind,dict(kind=kind,competition=True,points=points))
            self.assertEqual(self.store.get(kind)['kind'],kind)
        self.assertEqual(len(self.store.list()),3)
    def test_library_paths_never_escape(self):
        for name in ('../outside','x/y','x\\y','',None,'a'*65):
            with self.assertRaises(ValueError):self.store.path(name)
    def test_invalid_and_mixed_routes_refused(self):
        for value in (dict(kind='unknown'),dict(kind='waypoints',points=[]),dict(kind='pipeline',points=self.points)):
            with self.assertRaises(ValueError):validate_mission(value)
    def test_delete_changes_only_named_mission(self):
        for name in ('a','b'):self.store.save(name,dict(kind='waypoints',points=self.points))
        self.store.delete('a');self.assertEqual(self.store.list()[0]['name'],'b')
    def test_new_pipeline_can_use_operator_recording_without_auto_recorder(self):
        plan=validate_route(dict(points=[dict(kind='pipeline_start',lat=1.28,lon=103.85),dict(kind='pipeline_end',lat=1.281,lon=103.85)],recording_mode='manual'))
        logic=CompetitionRouteRun(plan,'test',0)
        self.assertEqual(logic.route['recording_mode'],'manual')
        self.assertFalse(logic.recording_requested)
        self.assertEqual(logic.route['pipeline_depth_m'],4.)
    def test_invalid_recording_mode_and_oversized_route_refused(self):
        for value in (dict(points=self.points,recording_mode='guess'),dict(points=self.points*257)):
            with self.assertRaises(ValueError):validate_route(value)


class FakeBag:
    def __init__(self):self.recording=False;self.start_count=self.stop_count=0
    def start(self,path,topics):self.recording=True;self.start_count+=1;return True,'started'
    def stop(self):self.recording=False;self.stop_count+=1;return True,'closed'
    def status(self,*args):return dict(recording=self.recording,error='')
    def tick(self):pass


class TestRecordingService(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.bag=FakeBag();self.calls=[];self.camera=None;self.sonar=None;self.free=5*1024**3
        self.service=RecordingService(self.folder.name,request=self.request,bag=self.bag,free=lambda:self.free,start_worker=False,persistence=lambda _:('/fixture','synthetic',True))
        self.service.update_sources([('/graey/dvl/velocity',['geometry_msgs/msg/TwistWithCovarianceStamped'])],dict(streams={}))
        self.command=dict(action='start',name='test',camera=False,sonar=False,sensors=True,topics=['/graey/dvl/velocity'])
    def request(self,port,path,body=None):
        self.calls.append((port,path,body))
        if port==8096:
            if path=='/status':return dict(recording_session_v1=True,camUp=True,webUp=True,recording=bool(self.camera),recFile=self.camera,recBytes=100)
            if path.startswith('/record/start'):
                self.camera=path.split('name=')[1]+'.mjpg';return dict(recording=True,recFile=self.camera)
            if path=='/record/stop':self.camera=None;return dict(recording=False)
        if path=='/state':return dict(recording_session_v1=True,map=True)
        if path=='/mapdata':return dict(recording=dict(name=self.sonar) if self.sonar else None)
        if path=='/record':self.sonar=None if body==b'stop' else body.decode()[6:];return 'ok'
        raise ValueError(path)
    def test_sensor_only_does_not_contact_camera_or_sonar(self):
        self.service.start(self.command);self.service.sample();self.service.stop()
        self.assertEqual(self.calls,[]);self.assertEqual(self.bag.start_count,1);self.assertEqual(self.bag.stop_count,1)
        self.assertTrue(list(Path(self.folder.name).glob('*/telemetry.jsonl')))
    def test_camera_and_sonar_are_explicit_and_independent(self):
        command=dict(self.command,camera=True,sonar=True,sensors=False)
        self.service.start(command);self.service.sample()
        self.assertTrue(self.service.session['sources']['camera']['recording'])
        self.assertTrue(self.service.session['sources']['sonar']['recording'])
        self.assertEqual(self.bag.start_count,0)
        self.service.stop();self.assertIsNone(self.camera);self.assertIsNone(self.sonar)
    def test_recording_starts_sources_without_any_viewer_and_stop_keeps_sources(self):
        original=self.request
        running={'camUp':False,'webUp':False}
        def request(port,path,body=None):
            if path in ('/camera/start','/start?tool=map'):
                self.calls.append((port,path,body))
                running['camUp' if path=='/camera/start' else 'webUp']=True
                return dict(msg='ok')
            result=original(port,path,body)
            if path=='/status':result.update(running)
            return result
        self.service.request_service=request
        self.service.start(dict(self.command,camera=True,sonar=True,sensors=False))
        self.service.sample();self.service.stop()
        self.assertTrue(all(running.values()))
        self.assertTrue(all(self.service.session['sources'][k]['acquired'] for k in ('camera','sonar')))
        self.assertFalse(any(path in ('/camera/stop','/stop') for _,path,_ in self.calls))
        manifest=json.loads((Path(self.service.session['directory'])/'session.json').read_text())
        self.assertEqual(set(manifest['sources']),{'camera','sonar'})
    def test_never_takes_over_or_stops_external_recording(self):
        self.camera='other_operator.mjpg';self.sonar='other_survey'
        self.service.start(dict(self.command,camera=True,sonar=True))
        self.service.stop()
        self.assertEqual(self.camera,'other_operator.mjpg');self.assertEqual(self.sonar,'other_survey')
        self.assertFalse(any(path.startswith('/record/start') or path=='/record/stop' for _,path,_ in self.calls))
    def test_ownership_changed_after_start_does_not_stop_new_owner(self):
        self.service.start(dict(self.command,camera=True,sonar=True,sensors=False))
        self.camera='new_owner.mjpg';self.sonar='new_owner';self.calls=[]
        self.service.stop()
        self.assertEqual(self.camera,'new_owner.mjpg');self.assertEqual(self.sonar,'new_owner')
        self.assertFalse(any(body==b'stop' or path=='/record/stop' for _,path,body in self.calls))
    def test_disk_floor_refuses_start_and_stops_active_bag(self):
        self.free=1024**3
        with self.assertRaisesRegex(ValueError,'2 GiB'):self.service.start(self.command)
        self.free=5*1024**3;self.service.start(self.command);self.free=1024**3;self.service.sample()
        self.assertFalse(self.bag.recording);self.assertIn('disk',self.service.error)
    def test_missing_source_is_partial_failure_not_claimed_recording(self):
        self.service.request_service=Mock(side_effect=OSError('unavailable'))
        self.service.start(dict(self.command,camera=True,sonar=True))
        self.assertTrue(self.service.session['sources']['sensors']['recording'])
        self.assertFalse(self.service.session['sources']['camera']['recording'])
        self.assertEqual(len(self.service.session['errors']),2)
        self.service.stop()
    def test_nothing_selected_or_topic_option_injection_refused(self):
        for command in (dict(self.command,sensors=False),dict(self.command,topics=['--all']),dict(self.command,name='../escape')):
            with self.assertRaises(ValueError):self.service.submit(command)
    def test_busy_request_not_replayed_and_manifest_durable(self):
        self.service.submit(self.command)
        with self.assertRaises(ValueError):self.service.submit(self.command)
        self.service.start(self.service.commands.get_nowait());self.service.stop()
        manifests=list(Path(self.folder.name).glob('*/session.json'))
        self.assertEqual(len(manifests),1);self.assertFalse(json.loads(manifests[0].read_text())['active'])
        self.assertEqual(len(self.service.sessions()),1)
    def test_snapshot_does_not_wait_on_recording_service_io(self):
        self.service.published=dict(active=True,name='test')
        self.assertTrue(self.service.snapshot()['session']['active'])


class TestOcsRunProxy(unittest.TestCase):
    def test_controls_use_existing_ocs_endpoints_and_no_retry(self):
        response=Mock(status=200);response.read.return_value=b'{"result":null}'
        conn=Mock();conn.getresponse.return_value=response
        with patch('http.client.HTTPConnection',return_value=conn) as factory:
            status,kind,body=ocs_request(8099,'/api/run/declare','POST',b'{"callsign":"pilot"}')
            self.assertEqual(status,200);factory.assert_called_once_with('127.0.0.1',8099,timeout=3)
            self.assertEqual(conn.request.call_args.args[1],'/api/declare')
        with patch('http.client.HTTPConnection',side_effect=OSError('lost')) as factory:
            with self.assertRaises(OSError):ocs_request(8099,'/api/run/end','POST',b'{}')
            self.assertEqual(factory.call_count,1)
    def test_no_arbitrary_ocs_urls_or_injection(self):
        for path,method,body in [('/api/run/anything','POST',b'{}'),('/api/run/declare','GET',b''),('/api/run/end','POST',b'{"vehicle_id":"other"}')]:
            with self.assertRaises(ValueError):ocs_request(8099,path,method,body)
