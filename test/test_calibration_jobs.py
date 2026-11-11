import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from robotx_graey_2026.api.gui.calibration_jobs import CalibrationJobs
from robotx_graey_2026.api.navigation.maintenance_safety import maintenance_reason, navigation_start_reason, calibration_reason
from robotx_graey_2026.api.navigation.vn_calibration import VNCalibration


class CalibrationJobsTests(unittest.TestCase):
    def test_calibration_accepts_sa_released_and_link_lost_with_propulsion_off(self):
        self.c.update(sa_active=False,inhibit='RF_LOST',rf_alive=False,tether_alive=False)
        self.assertEqual(calibration_reason(self.c),'')
        with patch('threading.Thread') as thread:
            self.job.request(dict(action='calibrate',confirmed=True))
            thread.return_value.start.assert_called_once()
        samples=iter([dict(log='old'),dict(log='new',phase='collected',saved=True)])
        self.job.sample=lambda name:next(samples)
        self.job.run('calibrate')
        self.assertEqual(self.calls,[('pause',None),('calibrate',None),('resume',None)])
        self.assertEqual(self.job.snapshot()['phase'],'collected')

    def test_calibration_rejects_enabled_propulsion_or_unknown_control(self):
        for key,value in [('armed',True),('esc_commanded_on',True),('vehicle_fresh',False),
                          ('mission_claimed',True),('autonomous',True)]:
            with self.subTest(key=key):
                self.assertTrue(calibration_reason(dict(self.c,**{key:value})))
        self.assertTrue(calibration_reason({}))

    def test_manual_heading_uses_existing_manual_operation_policy_and_service(self):
        with patch('threading.Thread') as thread:
            self.job.request(dict(action='align_heading',true_heading_deg=90.))
            thread.return_value.start.assert_called_once()
        self.job.run('align_heading')
        self.assertEqual(self.calls,[('align_heading',90.)])
        self.assertEqual(self.job.snapshot()['phase'],'aligned')

    def setUp(self):
        self.calls=[]
        self.c=dict(online=True,vehicle_fresh=True,armed=False,mode='MANUAL',sa_active=True,esc_commanded_on=False,mission_claimed=False,autonomous=False)
        self.job=CalibrationJobs(lambda:self.c,lambda:dict(streams={}),lambda n,v=None:self.calls.append((n,v)) or 'started',lambda:False)

    def test_startup_calls_start_only_without_sensor_qualification(self):
        self.job.run('prepare')
        self.assertEqual(self.calls,[('prepare',True)])
        self.assertEqual(self.job.snapshot()['phase'],'ready')

    def test_remote_start_allows_armed_manual_but_not_autonomy(self):
        self.c.update(armed=True,sa_active=False,esc_commanded_on=True)
        self.assertEqual(navigation_start_reason(self.c),'')
        self.assertTrue(maintenance_reason(self.c)) # Calibration still needs disabled propulsion.
        with patch('threading.Thread') as thread:
            self.job.request(dict(action='prepare',surface_confirmed=True))
            thread.return_value.start.assert_called_once()
        for key,value in [('mode','GUIDED'),('mission_claimed',True),('vehicle_fresh',False)]:
            self.assertTrue(navigation_start_reason(dict(self.c,**{key:value})))

    def test_retired_actions_rejected(self):
        for action in ('align_land','align_auto','recover_dvl','compass_start'):
            with self.assertRaises(ValueError):self.job.request(dict(action=action))

    def test_startup_requires_surface_confirmation_for_pressure_zero(self):
        with self.assertRaisesRegex(ValueError,'surface'):
            self.job.request(dict(action='prepare'))

    def test_startup_failure_does_not_report_ready(self):
        def fail(*args):raise ValueError('Service unavailable')
        self.job.service=fail;self.job.run('prepare')
        self.assertEqual(self.job.snapshot()['phase'],'failed')

    def test_saved_calibration_resumes_navigation_without_pressure_rezero(self):
        samples=iter([dict(log='old'),dict(log='new',phase='collected',saved=True,reason='Saved')])
        self.job.sample=lambda name:next(samples)
        self.job.run('calibrate')
        self.assertEqual(self.calls,[('pause',None),('calibrate',None),('resume',None)])
        self.assertEqual(self.job.snapshot()['phase'],'collected')

    def test_unsaved_calibration_does_not_resume_navigation(self):
        samples=iter([dict(log='old'),dict(log='new',phase='collected',saved=False)])
        self.job.sample=lambda name:next(samples)
        self.job.run('calibrate')
        self.assertNotIn(('resume',None),self.calls)
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
            regs={'44':['0','3','1'],'47':['1','0','0','0','1','0','0','0','1','0','0','0']}
            def reply(port,body,**kwargs):
                parts=body.split(',')
                if parts[0]=='VNWRG':regs[parts[1]]=parts[2:];return parts[2:]
                if body=='VNWNV':return []
                if parts[1] in regs:return regs[parts[1]]
                return self.reply(port,body)
            with patch('robotx_graey_2026.api.navigation.vn_calibration.transact',side_effect=reply) as transaction, \
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
                self.assertTrue(c.status()['accepted'])
                self.assertTrue(c.status()['saved'])
                commands=[a.args[1] for a in transaction.call_args_list]
                self.assertEqual(commands.count('VNWNV'),1)
                c.stop(object())
                self.assertEqual(sum(a.args[1]=='VNWNV' for a in transaction.call_args_list),1)

    def test_freeze_failure_cannot_report_success(self):
        c = VNCalibration()
        c.phase='collecting'
        with patch('robotx_graey_2026.api.navigation.vn_calibration.transact',side_effect=OSError('No ACK')):
            with self.assertRaises(OSError):
                c.stop(object())
        self.assertEqual(c.phase,'failed')

    def test_flash_ack_failure_never_reports_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            c=VNCalibration(folder=folder)
            c.path=Path(folder)/'calibration.jsonl'
            c.phase='collecting';c.faces=[5]*6
            regs={'44':['0','3','1'],'47':['1','0','0','0','1','0','0','0','1','0','0','0']}
            def reply(port,body,**kwargs):
                parts=body.split(',')
                if body=='VNWNV':raise OSError('No flash acknowledgement')
                if parts[0]=='VNWRG':regs[parts[1]]=parts[2:];return parts[2:]
                return regs.get(parts[1],['1','1','1','1'])
            with patch('robotx_graey_2026.api.navigation.vn_calibration.transact',side_effect=reply):
                with self.assertRaisesRegex(OSError,'flash acknowledgement'):c.stop(object())
            self.assertEqual(c.phase,'failed')
            self.assertFalse(c.status()['saved'])

    def test_missing_safety_fields_are_not_assumed_safe(self):
        self.assertTrue(maintenance_reason({}))
