"""GUI jobs call existing node services; never start competing navigation nodes."""
import copy
import math
import queue
import threading
import time
import uuid
from robotx_graey_2026.api.navigation.maintenance_safety import maintenance_reason


class ServiceQueue:
    def __init__(self, node):
        from std_srvs.srv import Trigger, SetBool
        from rcl_interfaces.srv import SetParametersAtomically
        from rcl_interfaces.msg import Parameter, ParameterValue
        self.parameter, self.parameter_value = Parameter, ParameterValue
        self.queue = queue.Queue()
        self.clients = {}
        for name, path, kind in (
            ('pause','/graey/navigation/pause_odometry',Trigger),
            ('prepare','/graey/navigation/prepare',SetBool),
            ('calibrate','/graey/vn100/calibrate_start',Trigger),
            ('cancel','/graey/vn100/calibrate_cancel',Trigger)):
            self.clients[name] = (node.create_client(kind,path),kind)

    def request(self, name, value=None):
        job = dict(name=name,value=value,deadline=time.monotonic()+20,event=threading.Event())
        self.queue.put(job)
        if not job['event'].wait(21):
            raise ValueError('No service acknowledgement; inspect sensor state before retrying')
        if not job.get('success'):
            raise ValueError(job.get('message','Navigation service unavailable'))
        return job.get('message','')

    def pump(self):
        try:
            job = self.queue.get_nowait()
        except queue.Empty:
            return
        def finish(success,message):
            job.update(success=success,message=message)
            job['event'].set()
        if time.monotonic() > job['deadline']:
            finish(False,'Navigation request expired before dispatch')
            return
        client, kind = self.clients[job['name']]
        if not client.service_is_ready():
            finish(False,'Required node service unavailable: '+job['name'])
            return
        request = kind.Request()
        if job['name'] == 'prepare':
            request.data = job['value'] is True
        if job['name'] in ('align_heading','align_land'):
            request.parameters=[self.parameter(name='true_heading_deg',
                value=self.parameter_value(type=3,double_value=float(job['value'])))]
        try:
            future = client.call_async(request)
            def done(result):
                try:
                    response = result.result()
                    if job['name'] in ('align_heading','align_land'):
                        finish(response.result.successful,response.result.reason)
                    else:
                        finish(response.success,response.message)
                except Exception as error:
                    finish(False,str(error))
            future.add_done_callback(done)
        except Exception as error:
            finish(False,str(error))


class CalibrationJobs:
    def __init__(self, controller, navigation, service, mission_running, clock=time.monotonic, sleep=time.sleep, maneuver=None):
        self.controller, self.navigation, self.service = controller, navigation, service
        self.mission_running, self.clock, self.sleep = mission_running, clock, sleep
        self.lock = threading.RLock()
        self.job = dict(phase='idle',message='Read instructions before starting',busy=False)
        self.waypoint_permission = False
        self.maneuver = maneuver

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.job)

    def update(self, **fields):
        with self.lock:
            self.job.update(fields)

    def request(self, body):
        action=body.get('action')
        if action not in ('prepare','calibrate','cancel'):
            raise ValueError('Only VN calibration and navigation startup are supported')
        with self.lock:
            if action=='cancel':
                self.job['cancel']=True
                return self.snapshot()
            reason=maintenance_reason(self.controller())
            if reason or self.mission_running():
                raise ValueError(reason or 'Stop the mission before calibration/startup')
            if self.job.get('busy'):raise ValueError('An operation is already running')
            if action=='calibrate' and body.get('confirmed') is not True:
                raise ValueError('Confirm safe rotation for VN calibration')
            if action=='prepare' and body.get('surface_confirmed') is not True:
                raise ValueError('Confirm floating at surface to initialize pressure zero')
            self.job=dict(action=action,phase='starting',busy=True,cancel=False,message='Starting '+action)
            threading.Thread(target=self.run,args=(action,),daemon=True).start()
            return self.snapshot()

    def sample(self,name):
        item=self.navigation().get('streams',{}).get(name,{})
        return item.get('data',{}) if item.get('fresh') else {}

    def run(self,action):
        try:
            if action=='prepare':
                message=self.service('prepare',True)
                self.waypoint_permission=True
                self.update(phase='ready',busy=False,message=message)
                return
            self.service('pause')
            self.waypoint_permission=False
            previous_log=self.sample('vn_calibration').get('log')
            self.service('calibrate')
            self.update(phase='collecting',message='Rotate all axes slowly; keep kill engaged')
            deadline=self.clock()+145
            while self.clock()<deadline:
                if self.snapshot().get('cancel') or maintenance_reason(self.controller()):
                    self.service('cancel')
                    raise ValueError('Calibration cancelled')
                sensor=self.sample('vn_calibration')
                self.update(sensor=sensor)
                if sensor.get('log') and sensor['log']!=previous_log:
                    if sensor.get('phase')=='collected':
                        self.update(phase='collected',busy=False,message=sensor.get('reason','VN calibration collected'))
                        return
                    if sensor.get('phase')=='failed':raise ValueError(sensor.get('reason','VN calibration failed'))
                self.sleep(.5)
            self.service('cancel')
            raise ValueError('VN calibration timed out')
        except Exception as error:
            self.update(phase='failed',busy=False,message=str(error))
