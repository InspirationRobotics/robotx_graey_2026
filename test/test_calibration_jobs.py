import copy
import tempfile
import unittest
from unittest.mock import patch
from robotx_graey_2026.api.gui.calibration_jobs import CalibrationJobs
from robotx_graey_2026.api.navigation.maintenance_safety import maintenance_reason
from robotx_graey_2026.api.navigation.vn_calibration import VNCalibration


class CalibrationJobsTests(unittest.TestCase):
    def test_retired_compass_actions_are_rejected(self):
        for action in ('compass_start','compass_capture','compass_cancel'):
            with self.assertRaisesRegex(ValueError,'Unknown'):
                self.job.request(dict(action=action,confirmed=True))

    def test_land_alignment_job_never_prepares_pressure_or_commissions(self):
        self.job.job=dict(busy=True,cancel=False,true_heading_deg=90.)
        self.job.run('align_land')
        self.assertEqual(self.calls,[('suspend',None),('pause',None),('align_land',90.)])
        self.assertEqual(self.job.snapshot()['phase'],'aligned_land')
        self.assertFalse(self.job.waypoint_permission)

    def test_retained_prepare_requires_current_land_reference(self):
        with self.assertRaisesRegex(ValueError,'Capture land north first'):
            self.job.request(dict(action='prepare',confirmed=True,surface_confirmed=True,alignment_mode='retained'))

    def test_automatic_alignment_requires_prepare_clearance_and_qualified_inputs(self):
        from unittest.mock import Mock
        self.job.maneuver=Mock()
        with self.assertRaisesRegex(ValueError,'Wait for qualified'):
            self.job.request(dict(action='align_auto',clearance_confirmed=True,max_distance_m=5.))
        self.job.job=dict(busy=False,action='prepare',phase='prepared')
        with self.assertRaisesRegex(ValueError,'Confirm clear'):
            self.job.request(dict(action='align_auto',max_distance_m=5.))
        with self.assertRaisesRegex(ValueError,'Wait for qualified'):
            self.job.request(dict(action='align_auto',clearance_confirmed=True,max_distance_m=5.))
        self.put('bridge',dict(gps_eligible=True,inputs=dict(vn=True,dvl=True),gps_admission=dict(qualified=True)))
        with patch('threading.Thread'):
            self.job.request(dict(action='align_auto',clearance_confirmed=True,max_distance_m=5.))
        self.job.maneuver.assert_called_once_with('start',5.)
        self.assertTrue(self.job.snapshot()['auto_started'])
        self.assertFalse(self.job.waypoint_permission)
        self.job.request(dict(action='cancel'))
        self.job.maneuver.assert_called_with('abort',None)

    def test_dvl_recovery_requires_surface_confirmation_and_safe_controller(self):
        with self.assertRaisesRegex(ValueError,'floating stationary'):
            self.job.request(dict(action='recover_dvl',confirmed=True))
        self.c['armed']=True
        with self.assertRaisesRegex(ValueError,'disarmed'):
            self.job.request(dict(action='recover_dvl',confirmed=True,surface_confirmed=True))
        self.assertEqual(self.calls,[])

    def test_dvl_recovery_uses_retained_alignment_without_pause_or_heading_capture(self):
        self.put('supervisor',dict(state='DVL continuation',north_alignment=dict(aligned=True)))
        self.job.job=dict(busy=True,cancel=False,alignment_mode='known',true_heading_deg=90.)
        def service(name,value=None):
            self.calls.append((name,value))
            if name=='recover_dvl':self.put('bridge',dict(published_monotonic=self.now,fault=''))
            if name=='suspend' and self.now>=.6:
                self.put('supervisor',dict(state='DVL continuation',north_alignment=dict(aligned=True)))
            return 'ok'
        self.job.service=service
        self.job.run('recover_dvl')
        self.assertEqual(self.calls,[('suspend',None),('recover_dvl',None),('suspend',None),('commission',None)])
        self.assertTrue(self.job.waypoint_permission)

    def test_recovery_clears_old_latch_only_after_clean_bridge_reset(self):
        self.put('supervisor',dict(state='Fault',reason='DVL continuity lost'))
        self.job.job=dict(busy=True,cancel=False)
        count=[0]
        def service(name,value=None):
            result=self.service(name,value)
            if name=='suspend':
                count[0]+=1
                if count[0]==2:
                    self.assertGreaterEqual(self.now,.6)
                    self.assertFalse(self.nav['streams']['bridge']['data']['fault'])
                    self.put('supervisor',dict(north_alignment=dict(aligned=True)))
            return result
        self.job.service=service
        self.job.run('prepare')
        self.assertTrue(self.job.waypoint_permission)
        self.assertEqual(count[0],2)

    def test_recovery_does_not_clear_fault_without_clean_reset_status(self):
        self.job.job=dict(busy=True,cancel=False)
        def service(name,value=None):
            result=self.service(name,value)
            if name=='prepare':self.put('bridge',dict(published_monotonic=0.,fault='Still invalid'))
            return result
        self.job.service=service
        self.job.run('prepare')
        self.assertFalse(self.job.waypoint_permission)
        self.assertEqual(sum(name=='suspend' for name,value in self.calls),1)
        self.assertIn('No clean bridge',self.job.snapshot()['message'])

    def test_known_heading_requires_confirmation_and_numeric_true_bearing(self):
        for value in (None,True,'90',-1,360,float('nan')):
            with self.assertRaises(ValueError):
                self.job.request(dict(action='prepare',confirmed=True,surface_confirmed=True,
                    alignment_mode='known',heading_confirmed=True,true_heading_deg=value))
        with self.assertRaises(ValueError):
            self.job.request(dict(action='prepare',confirmed=True,surface_confirmed=True,
                alignment_mode='known',heading_confirmed=False,true_heading_deg=90.))

    def test_known_heading_preparation_captures_before_commission(self):
        self.job.job=dict(busy=True,cancel=False,alignment_mode='known',true_heading_deg=90.)
        def service(name,value=None):
            self.service(name,value)
            if name=='align_heading':
                self.put('supervisor',dict(north_alignment=dict(aligned=True)))
            return 'ok'
        self.job.service=service
        self.job.run('prepare')
        self.assertEqual(self.calls,[('suspend',None),('pause',None),('prepare',True),('suspend',None),('align_heading',90.),('commission',None)])
        self.assertTrue(self.job.waypoint_permission)

    def test_known_heading_cancel_during_settling_never_captures_or_commissions(self):
        self.job.job=dict(busy=True,cancel=False,alignment_mode='known',true_heading_deg=90.)
        def sleep(seconds):
            self.now+=seconds
            self.job.job['cancel']=True
        self.job.sleep=sleep
        self.job.run('prepare')
        self.assertNotIn(('align_heading',90.),self.calls)
        self.assertFalse(self.job.waypoint_permission)

    def setUp(self):
        self.c = dict(online=True,vehicle_fresh=True,armed=False,sa_active=True,
                      esc_commanded_on=False,mission_claimed=False,autonomous=False)
        self.nav = dict(streams={})
        self.now, self.calls = 0., []
        self.refresh_status=True
        self.job = CalibrationJobs(lambda:self.c,lambda:self.nav,self.service,lambda:False,
                                   clock=lambda:self.now,sleep=self.sleep)

    def service(self, name, value=None):
        self.calls.append((name,value))
        if name=='prepare':
            self.put('bridge',dict(published_monotonic=self.now,fault=''))
        if name=='suspend' and self.now>=.6 and self.refresh_status and 'supervisor' in self.nav['streams']:
            self.nav['streams']['supervisor']['data']['published_monotonic']=self.now
        return name+' accepted'

    def sleep(self, seconds):
        self.now += seconds

    def put(self, name, value):
        if name == 'supervisor':
            value.setdefault('published_monotonic',self.now)
        self.nav['streams'][name] = dict(fresh=True,data=value)

    def test_prepare_ignores_cached_maintenance_fault_until_new_status(self):
        self.put('supervisor',dict(state='Fault',reason='Navigation maintenance; Prepare required before reuse',
                                  published_monotonic=-1.))
        self.job.job = dict(busy=True,cancel=False)
        def sleep(seconds):
            self.now += seconds
            self.put('supervisor',dict(north_alignment=dict(aligned=True)))
        self.job.sleep = sleep
        self.job.run('prepare')
        self.assertEqual(self.job.snapshot()['phase'],'ready')
        self.assertTrue(self.job.waypoint_permission)

    def test_prepare_missing_new_status_times_out_without_permission(self):
        self.refresh_status=False
        self.put('supervisor',dict(state='Fault',published_monotonic=-1.))
        self.job.job = dict(busy=True,cancel=False)
        self.job.run('prepare')
        self.assertEqual(self.job.snapshot()['phase'],'failed')
        self.assertIn('No fresh supervisor',self.job.snapshot()['message'])
        self.assertFalse(self.job.waypoint_permission)

    def test_prepare_new_fault_is_not_ignored(self):
        self.put('supervisor',dict(state='Fault',reason='DVL continuity fault'))
        self.job.job = dict(busy=True,cancel=False)
        self.job.run('prepare')
        self.assertEqual(self.job.snapshot()['message'],'DVL continuity fault')
        self.assertFalse(self.job.waypoint_permission)

    def test_unconfirmed_or_armed_request_cannot_start(self):
        for field, value in (('armed',True),('sa_active',False),('online',False),('vehicle_fresh',False),('esc_commanded_on',True)):
            old = self.c[field]
            self.c[field] = value
            with self.assertRaises(ValueError):
                self.job.request(dict(action='calibrate',confirmed=True))
            self.c[field] = old
        with self.assertRaises(ValueError):
            self.job.request(dict(action='calibrate',confirmed=False))
        with self.assertRaises(ValueError):
            self.job.request(dict(action='prepare',confirmed=True,surface_confirmed=False))
        self.assertEqual(self.calls,[])

    def test_prepare_finishes_without_alignment_and_without_waypoint_permission(self):
        self.put('supervisor',dict(north_alignment=dict(aligned=False,reason='Need motion')))
        self.job.job=dict(busy=True,cancel=False,action='prepare')
        self.job.run('prepare')
        self.assertEqual(self.calls,[('suspend',None),('pause',None),('prepare',True),('suspend',None)])
        self.assertFalse(self.job.waypoint_permission)
        self.assertEqual(self.job.snapshot()['phase'],'prepared')
        self.assertFalse(self.job.snapshot()['busy'])

    def test_alignment_finishes_without_commissioning(self):
        from unittest.mock import Mock
        self.job.maneuver=Mock()
        self.job.job=dict(busy=True,cancel=False,auto_started_at=0.)
        self.put('north_maneuver',dict(stage='COMPLETE',published_monotonic=0.))
        self.put('bridge',dict(north_alignment=dict(aligned=True)))
        self.job.monitor_alignment()
        self.assertEqual(self.job.snapshot()['phase'],'aligned')
        self.assertFalse(self.job.snapshot()['busy'])
        self.assertFalse(self.job.waypoint_permission)
        self.assertEqual(self.calls,[])

    def test_prepare_timeout_and_cancellation_do_not_grant_permission(self):
        for cancel in (True,False):
            self.job.job = dict(busy=True,cancel=cancel)
            self.job.run('prepare')
            self.assertFalse(self.job.waypoint_permission)
            self.assertEqual(self.job.snapshot()['phase'],'failed')

    def test_old_success_cannot_complete_a_new_calibration(self):
        self.put('vn_calibration',dict(phase='collected',log='old',reason='old success'))
        self.job.job = dict(busy=True,cancel=False)
        def sleep(seconds):
            self.assertTrue(self.job.snapshot()['busy'])
            self.now += seconds
            self.put('vn_calibration',dict(phase='failed',log='new',reason='Coverage missing'))
        self.job.sleep = sleep
        self.job.run('calibrate')
        self.assertEqual(self.job.snapshot()['phase'],'failed')
        self.assertEqual(self.job.snapshot()['message'],'Coverage missing')
        self.assertFalse(self.job.waypoint_permission)

    def test_cancel_during_commission_ack_never_enables_gui(self):
        self.put('supervisor',dict(north_alignment=dict(aligned=True)))
        self.job.job = dict(busy=True,cancel=False)
        def service(name, value=None):
            self.service(name,value)
            if name == 'commission':
                self.job.request(dict(action='cancel'))
            return 'ok'
        self.job.service = service
        self.job.run('prepare')
        self.assertFalse(self.job.waypoint_permission)
        self.assertEqual(self.job.snapshot()['phase'],'failed')


class VNCalibrationTests(unittest.TestCase):
    @staticmethod
    def reply(port, body):
        if body == 'VNRRG,44':
            return ['0','3','1']
        if body == 'VNRRG,35':
            return ['1','1','1','1']
        return ['1']

    def test_incomplete_coverage_stays_failed_and_is_never_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            now = [0.]
            c = VNCalibration(folder=folder,clock=lambda:now[0])
            with patch('robotx_graey_2026.api.navigation.vn_calibration.transact',side_effect=self.reply), \
                 patch('robotx_graey_2026.api.navigation.vn_calibration.relative_heading'):
                c.start(object())
                for i in range(120):
                    now[0] += 1
                    c.sample((0,0,0))
                c.stop(object())
                self.assertEqual(c.phase,'failed')
                self.assertFalse(c.status()['accepted'])
                self.assertFalse(c.status()['saved'])

    def test_all_six_faces_are_counted_and_freeze_ack_is_required(self):
        with tempfile.TemporaryDirectory() as folder:
            now = [0.]
            c = VNCalibration(folder=folder,clock=lambda:now[0])
            with patch('robotx_graey_2026.api.navigation.vn_calibration.transact',side_effect=self.reply) as transaction, \
                 patch('robotx_graey_2026.api.navigation.vn_calibration.relative_heading'):
                c.start(object())
                for ypr in ((0,0,0),(0,0,180),(0,90,0),(0,-90,0),(0,0,90),(0,0,-90)):
                    for _ in range(5):
                        now[0] += 1
                        c.sample(ypr)
                c.stop(object())
                self.assertEqual(c.phase,'collected')
                self.assertEqual(c.faces,[5]*6)
                self.assertTrue(any(a.args[1]=='VNWRG,44,0,3,1' for a in transaction.call_args_list))
                self.assertFalse(c.status()['accepted'])

    def test_freeze_failure_cannot_report_success(self):
        c = VNCalibration()
        c.phase='collecting'
        with patch('robotx_graey_2026.api.navigation.vn_calibration.transact',side_effect=OSError('No ACK')):
            with self.assertRaises(OSError):
                c.stop(object())
        self.assertEqual(c.phase,'failed')

    def test_missing_safety_fields_are_not_assumed_safe(self):
        self.assertTrue(maintenance_reason({}))
