"""Offline radio adapter tests, including real MAVLink bytes over loopback UDP."""
import socket
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robotx_graey_2026.api.comms.mesh_link import (
    DEFAULT_COMPONENT, DEFAULT_SYSTEM, MeshLink, received_record, tunnel_args,
)


def incoming(system=200, component=191, target=99, payload=b'\x00\x01\x00'):
    return SimpleNamespace(
        get_srcSystem=lambda: system, get_srcComponent=lambda: component,
        get_msgId=lambda: 385, target_system=target, target_component=0,
        payload_type=33022, payload_length=len(payload),
        payload=payload.ljust(128, b'\0'))


class MeshAdapterTests(unittest.TestCase):
    def test_binary_payload_and_trailing_zeros_are_preserved(self):
        args = tunnel_args(33022, b'\x00\x01\x00')
        self.assertEqual(args[:4], (0, 0, 33022, 3))
        self.assertEqual(len(args[4]), 128)
        self.assertEqual(args[4][:3], b'\x00\x01\x00')
        record = received_record('TUNNEL', incoming(), 4.0)
        self.assertEqual(record['payload_hex'], '000100')
        self.assertEqual(record['payload_length'], 3)

    def test_empty_and_full_payloads(self):
        self.assertEqual(tunnel_args(32768, b'')[3], 0)
        self.assertEqual(tunnel_args(65535, bytes(range(128)))[3], 128)
        with self.assertRaises(ValueError):
            tunnel_args(1, bytes(129))

    def test_invalid_wire_arguments_are_not_sent(self):
        for kwargs in (dict(payload_type=-1), dict(payload_type=65536),
                       dict(target_system=256), dict(target_component=-1),
                       dict(payload_type=True)):
            args = dict(payload_type=33022, payload=b'ok')
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                tunnel_args(**args)
        with self.assertRaises(TypeError):
            tunnel_args(33022, 'text needs explicit encoding')

    def test_every_peer_and_destination_is_exposed(self):
        link = Mock()
        messages = [('TUNNEL', incoming(system=s, target=99))
                    for s in (DEFAULT_SYSTEM, 200, 42, 254)]
        hb = SimpleNamespace(get_srcSystem=lambda: 1,
                             get_srcComponent=lambda: 1, get_msgId=lambda: 0)
        messages.append(('HEARTBEAT', hb))
        link.drain.side_effect = lambda handler: [handler(*m) for m in messages]
        mesh = MeshLink(link=link, clock=lambda: 3.5)
        records = mesh.poll()
        self.assertEqual([r['source_system'] for r in records], [43, 200, 42, 254, 1])
        self.assertTrue(all(r['target_system'] == 99 for r in records[:-1]))
        self.assertEqual(records[-1]['message'], 'HEARTBEAT')

    def test_send_is_explicit_and_never_retransmits_received_traffic(self):
        link = Mock()
        link.drain.side_effect = lambda handler: handler('TUNNEL', incoming())
        mesh = MeshLink(link=link)
        mesh.poll()
        link.send.assert_not_called()
        report = mesh.send_tunnel(33022, b'hello')
        self.assertEqual(report['event'], 'tx_attempt')
        link.send.assert_called_once_with('tunnel_send', *tunnel_args(33022, b'hello'))

    def test_raw_callback_retains_the_original_message(self):
        link = Mock()
        message = incoming()
        link.drain.side_effect = lambda handler: handler('TUNNEL', message)
        handler = Mock()
        MeshLink(link=link).drain(handler)
        handler.assert_called_once_with('TUNNEL', message)

    def test_serial_device_is_rejected(self):
        with self.assertRaises(ValueError):
            MeshLink('/dev/ttyACM0', link=Mock())

    def test_invalid_received_length_is_visible_as_an_error(self):
        msg = incoming()
        msg.payload_length = 129
        record = received_record('TUNNEL', msg, 1.0)
        self.assertIn('error', record)
        self.assertEqual(record['source_system'], 200)


class MeshWireTests(unittest.TestCase):
    def setUp(self):
        try:
            # Same import order as production; the shared helper selects v2.
            from robotx_graey_2026.api.pixhawk.mavlink import mavutil
        except ImportError:
            self.skipTest('pymavlink is required for real-wire tests')
        self.mavutil = mavutil
        self.router = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.router.close)
        self.router.bind(('127.0.0.1', 0))
        self.router.settimeout(2)
        endpoint = 'udpout:127.0.0.1:%d' % self.router.getsockname()[1]
        # pymavlink's Windows UDP client binds the destination port locally,
        # colliding with this same-host fake router. Model the Jetson/Linux
        # client behavior and bind its test socket explicitly to loopback.
        original = mavutil.mavlink_connection

        def loopback_connection(*args, **kwargs):
            with patch.object(mavutil.platform, 'system', return_value='Linux'):
                conn = original(*args, **kwargs)
            conn.port.bind(('127.0.0.1', 0))
            return conn

        with patch.object(mavutil, 'mavlink_connection', loopback_connection):
            self.mesh = MeshLink(endpoint)
        self.addCleanup(self.mesh.close)
        _, self.client = self.router.recvfrom(4096)  # Registration heartbeat.

    def parse(self, raw):
        parser = self.mavutil.mavlink.MAVLink(None)
        return parser.parse_buffer(raw)

    def test_one_transmission_decodes_identically_at_two_listeners(self):
        self.mesh.send_tunnel(33022, b'graey\0binary\0')
        raw, _ = self.router.recvfrom(4096)
        self.assertEqual(raw[0], 0xFD)  # MAVLink 2 on the wire.
        listeners = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
        try:
            for listener in listeners:
                listener.bind(('127.0.0.1', 0))
                listener.settimeout(2)
                self.router.sendto(raw, listener.getsockname())
            for listener in listeners:
                received, _ = listener.recvfrom(4096)
                frame = self.parse(received)[0]
                self.assertEqual(frame.get_type(), 'TUNNEL')
                self.assertEqual((frame.get_srcSystem(), frame.get_srcComponent()),
                                 (DEFAULT_SYSTEM, DEFAULT_COMPONENT))
                self.assertEqual(frame.target_system, 0)
                self.assertEqual(frame.payload_type, 33022)
                self.assertEqual(bytes(frame.payload)[:frame.payload_length], b'graey\0binary\0')
        finally:
            for listener in listeners:
                listener.close()

    def test_reverse_frames_from_all_peers_reach_the_adapter(self):
        for src in (200, 42, 254, DEFAULT_SYSTEM):
            encoder = self.mavutil.mavlink.MAVLink(None, srcSystem=src, srcComponent=191)
            msg = encoder.tunnel_encode(99, 0, 33022, 3, b'hi\0'.ljust(128, b'\0'))
            self.router.sendto(msg.pack(encoder), self.client)
        records = []
        deadline = time.monotonic() + 2
        while len(records) < 4 and time.monotonic() < deadline:
            records.extend(self.mesh.poll())
            time.sleep(.01)
        self.assertEqual([r['source_system'] for r in records], [200, 42, 254, 43])
        self.assertTrue(all(r['target_system'] == 99 for r in records))
        self.assertTrue(all(r['payload_hex'] == '686900' for r in records))


if __name__ == '__main__':
    unittest.main()
