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
            ('suspend','/graey/navigation/suspend',Trigger),
            ('pause','/graey/navigation/pause_odometry',Trigger),
            ('prepare','/graey/navigation/prepare',SetBool),
            ('recover_dvl','/graey/navigation/recover_dvl',Trigger),
            ('align_heading','/graey/navigation/align_heading',SetParametersAtomically),
            ('align_land','/graey/navigation/align_land',SetParametersAtomically),
            ('forget_land','/graey/navigation/forget_land',Trigger),
            ('begin_motion_north','/graey/navigation/begin_motion_north',Trigger),
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
        action = body.get('action')
        if action == 'align_auto':
            with self.lock:
                if not self.maneuver or self.job.get('busy') or self.mission_running():
                    raise ValueError('Finish or cancel the current operation before north alignment')
                if body.get('clearance_confirmed') is not True:
                    raise ValueError('Confirm clear surface travel and stopping clearance')
                b=self.sample('bridge')
                if (not b or b.get('fault') or not b.get('gps_eligible') or not b.get('inputs',{}).get('dvl')
                        or not b.get('inputs',{}).get('vn') or not (b.get('gps_admission') or {}).get('qualified')
                        or (b.get('north_alignment') or {}).get('aligned')):
                    raise ValueError('Wait for qualified surface GPS, DVL and VN before starting movement')
                if body.get('max_distance_m') != 5:
                    raise ValueError('This alignment run requires 5 m plus stopping clearance')
                self.service('begin_motion_north')
                self.maneuver('start',5.)
                self.waypoint_permission = False
                self.job=dict(action='align_auto',busy=True,cancel=False,auto_started=True,
                    auto_started_at=self.clock(),phase='aligning',message='Automatic surface alignment started; keep clear and supervise Abort')
                threading.Thread(target=self.monitor_alignment,daemon=True).start()
                return self.snapshot()
        if action not in ('calibrate','align_land','prepare','recover_dvl','cancel'):
            raise ValueError('Unknown calibration action')
        if action == 'cancel':
            with self.lock:
                if not self.job.get('busy'):
                    raise ValueError('No active job to cancel')
                self.job['cancel'] = True
                self.waypoint_permission = False
                if self.maneuver and self.job.get('auto_started'):
                    self.maneuver('abort',None)
            return self.snapshot()
        reason = maintenance_reason(self.controller())
        if reason or self.mission_running():
            raise ValueError(reason or 'Stop the mission before calibration or preparation')
        if body.get('confirmed') is not True:
            raise ValueError('Confirm that you read the instructions and maintained kill/disarm')
        if action in ('prepare','recover_dvl') and body.get('surface_confirmed') is not True:
            raise ValueError('Confirm floating stationary at the water surface before pressure calibration')
        mode=body.get('alignment_mode','gps')
        heading=body.get('true_heading_deg')
        if action in ('prepare','align_land'):
            if mode not in ('gps','known','retained'):
                raise ValueError('Unknown north alignment method')
            if (mode=='known' or action=='align_land') and (body.get('heading_confirmed') is not True
                    or isinstance(heading,bool) or not isinstance(heading,(int,float))
                    or not math.isfinite(heading) or not 0 <= heading < 360):
                raise ValueError('Confirm an independently measured true heading between 0 and 360 degrees')
            if action=='prepare' and mode=='retained' and not self.sample('bridge').get('land_north_reference'):
                raise ValueError('Capture land north first and keep VN continuously powered; otherwise select surface movement alignment')
        with self.lock:
            if self.job.get('busy'):
                raise ValueError('A calibration or preparation job is already active')
            self.waypoint_permission = False
            self.job = dict(id=uuid.uuid4().hex,action=action,phase='starting',
                            busy=True,cancel=False,message='Checking node services',
                            alignment_mode=mode,true_heading_deg=heading)
            threading.Thread(target=self.run,args=(action,),daemon=True).start()
        return self.snapshot()

    def sample(self, name):
        value = self.navigation().get('streams',{}).get(name,{})
        return value.get('data',{}) if value.get('fresh') else {}

    def monitor_alignment(self):
        """Alignment ends independently; it never commissions or resets navigation."""
        try:
            started=self.snapshot()['auto_started_at']
            seen=False
            while self.clock()-started < 70:
                if self.snapshot().get('cancel'):
                    raise ValueError('Alignment cancelled')
                m=self.sample('north_maneuver')
                stamp=m.get('published_monotonic',-math.inf)
                fresh=started<=stamp<=self.clock() and self.clock()-stamp<=.7
                if not fresh:
                    if seen or self.clock()-started>10:
                        raise ValueError('Automatic alignment status missing or stale')
                else:
                    seen=True
                    self.update(message=m.get('reason','Aligning'))
                    if m.get('stage')=='ABORTED':
                        raise ValueError(m.get('reason','Alignment aborted'))
                    if m.get('stage')=='COMPLETE' and self.sample('bridge').get('north_alignment',{}).get('aligned'):
                        self.update(phase='aligned',busy=False,auto_verified=True,
                            message='North aligned. Disarm and engage kill, then Prepare Navigation to verify Cube fusion.')
                        return
                self.sleep(.2)
            raise ValueError('Alignment timed out')
        except Exception as error:
            self.maneuver('abort',None)
            self.update(phase='failed',busy=False,message=str(error))

    def run(self, action):
        try:
            self.service('suspend')
            if action != 'recover_dvl':
                self.service('pause')
            if self.snapshot().get('cancel'):
                raise ValueError('Preparation cancelled; navigation remains suspended')
            if action=='align_land':
                self.update(phase='aligning',message='Keep level, stationary and killed; capturing known true bearing')
                self.sleep(2.2)
                if self.snapshot().get('cancel') or maintenance_reason(self.controller()):
                    raise ValueError('Land north capture cancelled or safety state changed')
                message=self.service('align_land',self.snapshot()['true_heading_deg'])
                self.update(phase='aligned_land',busy=False,message=message)
                return
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
            self.update(phase='preparing',message=(self.service('recover_dvl') if action=='recover_dvl' else self.service('prepare',True)))
            # While the first suspend/pause/reset was in flight, supervisor can
            # legitimately latch the OLD bridge fault. Wait for post-reset clean
            # bridge status, then explicitly suspend once more while maintained.
            reset_ack = self.clock()
            while True:
                b=self.sample('bridge')
                stamp=b.get('published_monotonic')
                if (isinstance(stamp,(int,float)) and reset_ack <= stamp <= self.clock()
                        and not b.get('fault')):
                    if action=='prepare' and self.snapshot().get('alignment_mode')=='retained' and not b.get('land_north_reference'):
                        raise ValueError('Land north continuity expired during Prepare; capture north again')
                    break
                if self.snapshot().get('cancel') or self.clock()-reset_ack>=10:
                    raise ValueError('No clean bridge status after reset; preparation stopped')
                self.sleep(.1)
            self.sleep(.6)  # Old bridge publications are outside the 0.5s gate.
            if self.snapshot().get('cancel') or maintenance_reason(self.controller()):
                raise ValueError('Preparation cancelled or safety state changed')
            self.service('suspend')
            if action!='recover_dvl' and self.snapshot().get('alignment_mode')=='known':
                self.update(message='Hold level and stationary with kill engaged; capturing measured true heading')
                self.sleep(2.2)
                if self.snapshot().get('cancel') or maintenance_reason(self.controller()):
                    raise ValueError('Heading capture cancelled or safety state changed')
                self.service('align_heading',self.snapshot()['true_heading_deg'])
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
                message = a.get('reason','Waiting for north-alignment telemetry')
                gps = s.get('gps_admission') or {}
                if message == 'GPS qualifying':
                    message = (f"GPS qualifying {gps.get('qualifying_s',0.):.1f}/"
                               f"{gps.get('required_dwell_s',3.):.1f} s")
                    if gps.get('last_rejection'):
                        message += '; previous reset: '+gps['last_rejection']
                    message += '. Stay stationary until GPS is qualified.'
                self.update(north=a,message=message)
                if action=='prepare' and not a.get('aligned'):
                    if s.get('state')=='Fault':
                        raise ValueError(s.get('reason','Navigation fault'))
                    self.update(phase='prepared',busy=False,
                        message='Surface sensors prepared. North alignment is a separate operation; waypoint permission remains off until north and Cube fusion are verified.')
                    return
                if s.get('state') == 'Fault':
                    raise ValueError(s.get('reason','Navigation fault'))
                if a.get('aligned'):
                    self.update(phase='verify',message='North aligned. Stay stopped, disarmed and killed; checking Cube fusion.')
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
            if self.maneuver and self.snapshot().get('auto_started'):
                self.maneuver('abort',None)
            self.update(phase='failed',busy=False,message=str(error))
