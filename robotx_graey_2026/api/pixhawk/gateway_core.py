"""Central FCU admission rules. No ROS or I/O; ArduSub-specific command ownership."""
import math

# 47 MISSION_ACK and 51 MISSION_REQUEST_INT (Ruth, Nov 10): with 43 they read the QGC plan stored on the Cube
# (tools/survey_run.py). Uploading still cannot pass: 44 MISSION_COUNT and 73 MISSION_ITEM_INT are not admitted.
READ_MESSAGES = {20,21,43,47,51,66,111,117,119,122,321}
MOTION_MESSAGES = {69,70,82,84,86}
COMPONENTS = {191,193,194,195,197,198,201}
RADIO_COMPONENT = 201
RADIO_SYSTEM = 43
RADIO_DESTINATIONS = {(250,190),(254,191)}
RADIO_PAYLOAD_TYPES = {0x9001,0x8010,0x8038}


class GatewayPolicy:
    def __init__(self):
        self.control = {}
        self.control_at = -math.inf

    def update_control(self, data, now):
        if not isinstance(data,dict):raise ValueError('controller status must be an object')
        stamp=data.get('published_monotonic')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp>now:
            raise ValueError('controller timestamp missing or invalid')
        self.control,self.control_at = data,stamp

    def permitted(self, now):
        c=self.control
        return (0<=now-self.control_at<.7 and c.get('vehicle_fresh') is True
                and c.get('sa_active') is False and c.get('esc_commanded_on') is True
                and not c.get('inhibit',True))

    def admit(self, message, component, now):
        if component not in COMPONENTS or message.get_srcComponent()!=component:
            return False,'unregistered component'
        if message.get_msgId()==385:
            target=(getattr(message,'target_system',0),getattr(message,'target_component',0))
            length=getattr(message,'payload_length',-1)
            ptype=getattr(message,'payload_type',-1)
            destination_ok=(target in RADIO_DESTINATIONS or
                            (target==(0,0) and ptype in (0x9001,0x8010)))
            allowed=(component==RADIO_COMPONENT and message.get_srcSystem()==RADIO_SYSTEM
                    and destination_ok and ptype in RADIO_PAYLOAD_TYPES
                    and type(length) is int and 0<length<=128)
            return allowed, None if allowed else 'invalid companion radio tunnel'
        if getattr(message,'target_system',1) not in (0,1) or getattr(message,'target_component',1) not in (0,1):
            return False,'FCU target must be the configured Cube'
        mid=message.get_msgId()
        if mid==0:return False,'gateway owns component heartbeat'
        if mid in READ_MESSAGES:return True,None
        if mid==331:return (component==197,'odometry belongs to navigation')
        if mid in MOTION_MESSAGES:
            # Releasing overrides is always permitted, including after kill.
            if mid==70 and all(getattr(message,'chan%d_raw'%i,65535) in (0,65535,65534) for i in range(1,9)):
                return component==198,'release requires mission component'
            return component==198 and self.permitted(now) and self.control.get('armed') is True,'motion requires fresh control permission'
        if mid==76:
            command=message.command
            if command in (511,512):return True,None
            if command==400 and message.param1==0:
                return component in (194,195,198),'disarm owner'
            if command==176 and message.param2==19:
                return component in (194,198),'manual mode owner'
            if command==183:return component==194,'ESC relay belongs to controller'
            if command==400 and (message.param1!=1 or getattr(message,'param2',0)!=0):
                return False,'force-arm is not an admitted operation'
            if command in (176,400):
                return component in (194,198) and self.permitted(now),'mode/arm requires control permission'
        return False,'message not admitted by FCU gateway'
