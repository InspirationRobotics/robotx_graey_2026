"""Manual MAVLink mesh endpoint through GRAEY's existing Link/MAVProxy path.

No serial device is opened here. Every parsed incoming message is exposed,
including messages from other vehicles and our own echoed traffic. TUNNEL
payloads remain bytes: there is no boat codec, mission logic, peer allowlist,
application acknowledgement, or automatic forwarding in this module.

    python3 -m robotx_graey_2026.api.comms.mesh_link --seconds 10
    python3 -m robotx_graey_2026.api.comms.mesh_link --send-text 'hello from graey'

The matching local MAVProxy output must exist first. Running this module sends
component heartbeats; --send-text/--send-hex additionally submit one TUNNEL.
"""
import argparse
import json
import math
import time

DEFAULT_ENDPOINT = 'udpout:127.0.0.1:14601'
DEFAULT_SYSTEM = 43
DEFAULT_COMPONENT = 191
TEST_PAYLOAD_TYPE = 33022  # Existing team TEST text; no vehicle action.
MAX_PAYLOAD = 128


def _integer(name, value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in {low}..{high}')
    return value


def tunnel_args(payload_type, payload, target_system=0, target_component=0):
    """Build standard TUNNEL arguments; broadcast is the default destination."""
    _integer('payload_type', payload_type, 0, 65535)
    _integer('target_system', target_system, 0, 255)
    _integer('target_component', target_component, 0, 255)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError('payload must be bytes, bytearray, or memoryview')
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('TUNNEL payload exceeds 128 bytes')
    return (target_system, target_component, payload_type, len(payload),
            payload.ljust(MAX_PAYLOAD, b'\0'))


def received_record(kind, message, now):
    """Transport metadata for every parsed frame, without recipient filtering."""
    record = dict(event='rx', received_monotonic=now, message=kind,
                  source_system=message.get_srcSystem(),
                  source_component=message.get_srcComponent(),
                  message_id=message.get_msgId())
    raw = getattr(message, 'get_msgbuf', lambda: None)()
    if raw is not None:
        record['frame_hex'] = bytes(raw).hex()
    if kind == 'TUNNEL':
        length = int(message.payload_length)
        data = bytes(message.payload)
        if not 0 <= length <= MAX_PAYLOAD or length > len(data):
            record['error'] = 'invalid TUNNEL payload_length'
        else:
            body = data[:length]
            record.update(target_system=int(message.target_system),
                          target_component=int(message.target_component),
                          payload_type=int(message.payload_type),
                          payload_length=length, payload_hex=body.hex())
    return record


class MeshLink:
    """Small adapter over the same Link helper used by existing GRAEY clients.

    An injected link is useful for offline verification. Use this from one
    thread, just like the manual diagnostic below.
    """

    def __init__(self, endpoint=DEFAULT_ENDPOINT, system=DEFAULT_SYSTEM,
                 component=DEFAULT_COMPONENT, *, link=None,
                 clock=time.monotonic, node=None):
        _integer('system', system, 1, 254)
        _integer('component', component, 1, 255)
        if not endpoint.startswith('udpout:'):
            raise ValueError('use a udpout endpoint to MAVProxy, never a serial device')
        if link is None:
            # Import before any pymavlink use: Link selects MAVLink 2 and the
            # ardupilotmega dialect. It registers its UDP client by heartbeat.
            from robotx_graey_2026.api.pixhawk.mavlink import Link
            link = Link(endpoint, component, system=system, node=node)
        self.link = link
        self.clock = clock

    def send_tunnel(self, payload_type, payload, target_system=0, target_component=0):
        """Offer a frame to Link, not a remote-delivery acknowledgement.

        Link guards/reopens failed UDP sockets. Its send method does not return
        delivery status, so only a receive record at another endpoint proves
        that this payload crossed a transport.
        """
        args = tunnel_args(payload_type, payload, target_system, target_component)
        self.link.send('tunnel_send', *args)
        return dict(event='tx_attempt', payload_type=payload_type,
                    target_system=target_system, target_component=target_component,
                    payload_length=args[3], payload_hex=bytes(payload).hex())

    def poll(self):
        """Return records for every frame yielded by Link's bounded drain."""
        records = []
        self.drain(lambda kind, msg: records.append(
            received_record(kind, msg, self.clock())))
        return records

    def drain(self, handler):
        """Pass every original (kind, pymavlink message) to a caller's handler."""
        self.link.drain(handler)

    def heartbeat(self):
        self.link.heartbeat()

    def close(self):
        self.link.mav.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--endpoint', default=DEFAULT_ENDPOINT)
    parser.add_argument('--system', type=int, default=DEFAULT_SYSTEM)
    parser.add_argument('--component', type=int, default=DEFAULT_COMPONENT)
    parser.add_argument('--seconds', type=float, default=10.0,
                        help='listen duration; 0 listens until Ctrl-C (default 10)')
    send = parser.add_mutually_exclusive_group()
    send.add_argument('--send-text', help='send one existing team TEST text TUNNEL')
    send.add_argument('--send-hex', help='send raw application bytes as hexadecimal')
    parser.add_argument('--payload-type', type=int,
                        help='required with --send-hex; --send-text uses 33022')
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.seconds) or args.seconds < 0:
            raise ValueError('--seconds must be finite and nonnegative')
        if args.send_hex is not None and args.payload_type is None:
            raise ValueError('--send-hex requires --payload-type')
        if args.send_hex is None and args.payload_type is not None:
            raise ValueError('--payload-type applies only to --send-hex')
        payload = None
        if args.send_text is not None:
            payload, ptype = args.send_text.encode('utf-8'), TEST_PAYLOAD_TYPE
        elif args.send_hex is not None:
            payload, ptype = bytes.fromhex(args.send_hex), args.payload_type
        if payload is not None:
            tunnel_args(ptype, payload)  # Validate before opening a connection.
        mesh = MeshLink(args.endpoint, args.system, args.component)
    except (ValueError, TypeError, ImportError) as exc:
        parser.error(str(exc))

    print(json.dumps(dict(event='open', endpoint=args.endpoint,
                          source_system=args.system, source_component=args.component)),
          flush=True)
    try:
        if payload is not None:
            print(json.dumps(mesh.send_tunnel(ptype, payload)), flush=True)
        started = time.monotonic()
        next_heartbeat = started + 1.0
        while args.seconds == 0 or time.monotonic() - started < args.seconds:
            if time.monotonic() >= next_heartbeat:
                mesh.heartbeat()
                next_heartbeat = time.monotonic() + 1.0
            for record in mesh.poll():
                print(json.dumps(record), flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        mesh.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
