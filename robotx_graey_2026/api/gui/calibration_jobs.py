"""GUI jobs call existing node services; never start competing navigation nodes."""
import copy
import queue
import threading
import time
import uuid
from robotx_graey_2026.api.navigation.maintenance_safety import maintenance_reason


class ServiceQueue:
    def __init__(self, node):
        from std_srvs.srv import Trigger, SetBool
        self.queue = queue.Queue()
        self.clients = {}
        for name, path, kind in (
            ('suspend','/graey/navigation/suspend',Trigger),
            ('pause','/graey/navigation/pause_odometry',Trigger),
            ('prepare','/graey/navigation/prepare',SetBool),
            ('calibrate','/graey/vn100/calibrate_start',Trigger),
            ('cancel','/graey/vn100/calibrate_cancel',Trigger),
            ('commission','/graey/navigation/commission',Trigger)):
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
        try:
            future = client.call_async(request)
            def done(result):
                try:
                    response = result.result()
                    finish(response.success,response.message)
                except Exception as error:
                    finish(False,str(error))
            future.add_done_callback(done)
        except Exception as error:
            finish(False,str(error))


class CalibrationJobs:
    def __init__(self, controller, navigation, service, mission_running, clock=time.monotonic, sleep=time.sleep):
        self.controller, self.navigation, self.service = controller, navigation, service
        self.mission_running, self.clock, self.sleep = mission_running, clock, sleep
        self.lock = threading.RLock()
        self.job = dict(phase='idle',message='Read instructions before starting',busy=False)
        self.waypoint_permission = False

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.job)

    def update(self, **fields):
        with self.lock:
            self.job.update(fields)

    def request(self, body):
        action = body.get('action')
        if action not in ('calibrate','prepare','cancel'):
            raise ValueError('Unknown calibration action')
        if action == 'cancel':
            with self.lock:
                if not self.job.get('busy'):
                    raise ValueError('No active job to cancel')
                self.job['cancel'] = True
                self.waypoint_permission = False
            return self.snapshot()
        reason = maintenance_reason(self.controller())
        if reason or self.mission_running():
            raise ValueError(reason or 'Stop the mission before calibration or preparation')
        if body.get('confirmed') is not True:
            raise ValueError('Confirm that you read the instructions and maintained kill/disarm')
        if action == 'prepare' and body.get('surface_confirmed') is not True:
            raise ValueError('Confirm floating stationary at the water surface before pressure calibration')
        with self.lock:
            if self.job.get('busy'):
                raise ValueError('A calibration or preparation job is already active')
            self.waypoint_permission = False
            self.job = dict(id=uuid.uuid4().hex,action=action,phase='starting',
                            busy=True,cancel=False,message='Checking node services')
            threading.Thread(target=self.run,args=(action,),daemon=True).start()
        return self.snapshot()

    def sample(self, name):
        value = self.navigation().get('streams',{}).get(name,{})
        return value.get('data',{}) if value.get('fresh') else {}

    def run(self, action):
        try:
            self.service('suspend')
            self.service('pause')
            if self.snapshot().get('cancel'):
                raise ValueError('Preparation cancelled; navigation remains suspended')
            if action == 'calibrate':
                previous_log = self.sample('vn_calibration').get('log')
                self.service('calibrate')
                self.update(phase='collecting',message='Rotate all axes slowly; keep kill engaged throughout')
                deadline = self.clock()+145
                while self.clock() < deadline:
                    if self.snapshot().get('cancel'):
                        self.service('cancel')
                        raise ValueError('Calibration cancelled; repeat before accepting it')
                    c = self.sample('vn_calibration')
                    self.update(sensor=c)
                    current_run = c.get('log') and c.get('log') != previous_log
                    if current_run and c.get('phase') == 'collected':
                        self.update(phase='collected',busy=False,message=c['reason'])
                        return
                    if current_run and c.get('phase') == 'failed':
                        raise ValueError(c.get('reason','VN calibration failed'))
                    if maintenance_reason(self.controller()):
                        self.service('cancel')
                        raise ValueError('Safety state changed; calibration stopped')
                    self.sleep(.5)
                self.service('cancel')
                raise ValueError('Calibration timed out; inspect sensor status')
            self.update(phase='preparing',message=self.service('prepare',True))
            # Service ACK precedes the next supervisor publication. Cached status
            # can still contain the maintenance fault set by pause_odometry.
            prepared_at = self.clock()
            deadline = self.clock()+300
            while self.clock() < deadline:
                if self.snapshot().get('cancel'):
                    # Do not reset or change flight modes during manual movement.
                    raise ValueError('Preparation cancelled; waypoint permission remains off')
                s = self.sample('supervisor')
                stamp = s.get('published_monotonic')
                if not isinstance(stamp,(int,float)) or not prepared_at <= stamp <= self.clock():
                    if self.clock()-prepared_at >= 10:
                        raise ValueError('No fresh supervisor acknowledgement after preparation')
                    self.update(message='Surface baseline accepted; waiting for updated supervisor status')
                    self.sleep(.2)
                    continue
                a = s.get('north_alignment') or {}
                self.update(north=a,message=a.get('reason','Waiting for north-alignment telemetry'))
                if s.get('state') == 'Fault':
                    raise ValueError(s.get('reason','Navigation fault'))
                if a.get('aligned'):
                    self.update(phase='verify',message='North aligned. Stop, disarm and engage kill; checking Cube fusion.')
                    if not maintenance_reason(self.controller()):
                        try:
                            message = self.service('commission')
                        except ValueError as error:
                            self.update(message=str(error))
                        else:
                            with self.lock:
                                if self.job.get('cancel'):
                                    raise ValueError('Preparation cancelled; waypoint permission remains off')
                                self.waypoint_permission = True
                                self.job.update(phase='ready',busy=False,message=message)
                            return
                self.sleep(1.)
            raise ValueError('Preparation timed out; no waypoint permission granted')
        except Exception as error:
            self.update(phase='failed',busy=False,message=str(error))
