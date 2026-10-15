import copy
import tempfile
import unittest
from unittest.mock import patch
from robotx_graey_2026.api.gui.calibration_jobs import CalibrationJobs
from robotx_graey_2026.api.navigation.maintenance_safety import maintenance_reason
from robotx_graey_2026.api.navigation.vn_calibration import VNCalibration


class CalibrationJobsTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(online=True,vehicle_fresh=True,armed=False,sa_active=True,
                      esc_commanded_on=False,mission_claimed=False,autonomous=False)
        self.nav = dict(streams={})
        self.now, self.calls = 0., []
        self.job = CalibrationJobs(lambda:self.c,lambda:self.nav,self.service,lambda:False,
                                   clock=lambda:self.now,sleep=self.sleep)

    def service(self, name, value=None):
        self.calls.append((name,value))
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

    def test_prepare_suspends_then_waits_for_alignment_and_kill(self):
        self.put('supervisor',dict(north_alignment=dict(aligned=False,reason='Need motion')))
        self.job.job = dict(busy=True,cancel=False,action='prepare')
        def sleep(seconds):
            self.now += seconds
            if self.now == 1:
                self.c.update(armed=True,sa_active=False,esc_commanded_on=True)
                self.put('supervisor',dict(north_alignment=dict(aligned=True,reason='Aligned')))
            if self.now == 2:
                self.c.update(armed=False,sa_active=True,esc_commanded_on=False)
        self.job.sleep = sleep
        self.job.run('prepare')
        self.assertEqual(self.calls,[('suspend',None),('pause',None),('prepare',True),('commission',None)])
        self.assertTrue(self.job.waypoint_permission)
        self.assertEqual(self.job.snapshot()['phase'],'ready')

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
