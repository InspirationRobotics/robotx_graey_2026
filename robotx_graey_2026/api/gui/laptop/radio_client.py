"""Laptop mission client for fleet_link's GRAEY RFD output."""
import secrets
import threading
import time

from robotx_graey_2026.api.comms.mission_radio import (
    Assembler, KIND_REQUEST, KIND_RESPONSE, PAYLOAD_TYPE, encode)
from robotx_graey_2026.api.comms.radio_mission_node import (
    COMPANION_COMPONENT, COMPANION_SYSTEM, LAPTOP_COMPONENT, LAPTOP_SYSTEM)


class RadioMissionClient:
    def __init__(self, port=14572, clock=time.monotonic, start=True):
        self.port, self.clock = port, clock
        self.condition = threading.Condition()
        self.responses = {}
        self.assembler = Assembler(clock=clock)
        self.last_seen = None
        self.last_status = None
        self.last_error = None
        self.closed = False
        self.connection = None
        self.thread = threading.Thread(target=self._run, name='graey-radio-client', daemon=True) if start else None
        if self.thread is not None:self.thread.start()

    def _run(self):
        try:
            from pymavlink import mavutil
            self.connection = mavutil.mavlink_connection(
                f'udpin:127.0.0.1:{self.port}', source_system=LAPTOP_SYSTEM,
                source_component=LAPTOP_COMPONENT, autoreconnect=True)
            while not self.closed:
                msg = self.connection.recv_match(type='TUNNEL', blocking=True, timeout=.2)
                if msg is None:continue
                if ((msg.get_srcSystem(),msg.get_srcComponent()) !=
                        (COMPANION_SYSTEM,COMPANION_COMPONENT)
                        or getattr(msg,'target_system',0) not in (0,LAPTOP_SYSTEM)
                        or getattr(msg,'target_component',0) not in (0,LAPTOP_COMPONENT)):
                    continue
                # Standard RXL heartbeat traffic proves the companion route.  The
                # vehicle need not duplicate a private JSON status frame at 2 Hz.
                with self.condition:
                    self.last_seen=self.clock()
                    self.condition.notify_all()
                if int(getattr(msg,'payload_type',-1)) != PAYLOAD_TYPE:
                    continue
                payload=bytes(msg.payload)[:int(msg.payload_length)]
                try:complete=self.assembler.add(payload)
                except ValueError as exc:
                    self.last_error=str(exc);continue
                if complete is None:continue
                kind,request_id,value=complete
                if kind!=KIND_RESPONSE:continue
                with self.condition:
                    if value.get('event')=='radio_status':self.last_status=value
                    else:self.responses[request_id]=value
                    self.condition.notify_all()
        except Exception as exc:
            self.last_error=str(exc)
        finally:
            with self.condition:self.condition.notify_all()

    def status(self):
        age=None if self.last_seen is None else max(0.,self.clock()-self.last_seen)
        return dict(connected=age is not None and age<2.0, age_s=age, port=self.port,
                    vehicle=self.last_status, error=self.last_error)

    def request(self, body, timeout=8.0):
        if not isinstance(body,dict):raise ValueError('radio request must be an object')
        request_id=secrets.randbits(32) or 1
        chunks=encode(body,request_id,KIND_REQUEST)
        deadline=self.clock()+timeout
        next_send=-1.
        while self.clock()<deadline:
            with self.condition:
                if request_id in self.responses:
                    result=self.responses.pop(request_id)
                    if result.get('ok') is not True:raise ValueError(result.get('msg','GRAEY refused radio request'))
                    return result
                status=self.status()
                if not status['connected']:
                    self.condition.wait(min(.25,max(0.,deadline-self.clock())))
                    continue
            if self.connection is None:
                time.sleep(.05);continue
            now=self.clock()
            if now>=next_send:
                for payload in chunks:
                    self.connection.mav.tunnel_send(COMPANION_SYSTEM,COMPANION_COMPONENT,
                        PAYLOAD_TYPE,len(payload),payload+bytes(128-len(payload)))
                next_send=now+.75
            with self.condition:self.condition.wait(min(.2,max(0.,deadline-self.clock())))
        raise ValueError('No acknowledged GRAEY radio reply; mission was not confirmed')

    def close(self):
        self.closed=True
        if self.connection is not None:
            try:self.connection.close()
            except Exception:pass
        if self.thread is not None:self.thread.join(timeout=2.)
