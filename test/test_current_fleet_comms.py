"""Current upstream protocol contracts and submarine-specific adaptations."""
import json
import socket
import threading
import unittest
from unittest.mock import Mock, patch

from robotx_graey_2026.api.comms.competition_wifi import CompetitionSender, WifiLink
from robotx_graey_2026.api.comms.ocs_discovery import TargetChooser, BeaconListener, check_fallback_hosts
from robotx_graey_2026.api.comms.run_gate import RunGate
from robotx_graey_2026.api.gui.gcs_transport import LaptopLink, target


class TestCurrentRun(unittest.TestCase):
    def setUp(self):
        self.sender=CompetitionSender(':memory:')
        self.addCleanup(self.sender.close)

    def run_snapshot(self,**fields):
        self.sender.receive({'ocs_run':dict(state='RUNNING',run_id=70000,**fields)},0)

    def test_native_snapshot_lives_until_disconnect_not_five_second_lease(self):
        self.run_snapshot(graey_report_v1=True)
        self.sender.survey('one',1,2,[1])
        self.sender.set_surface(True,100)
        self.assertTrue(self.sender.status(100)['run_confirmed'])
        self.assertEqual(len(self.sender.due(100)),1)
        self.sender.disconnected()
        self.assertFalse(self.sender.status(100)['run_confirmed'])
        self.assertEqual(self.sender.due(100),[])

    def test_stock_native_ocs_does_not_receive_unknown_report_envelopes(self):
        self.run_snapshot()
        self.sender.survey('one',1,2,[1]);self.sender.set_surface(True,0)
        self.assertEqual(self.sender.due(0),[])
        self.assertFalse(self.sender.status(0)['report_transport_ready'])
        self.assertEqual(self.sender.status(0)['pending'],1)

    def test_all_native_pre_run_states_accepted_without_start(self):
        for state in ('DISCONNECTED','CONNECTED','COURSE_RX','DECLARED','ENDED'):
            self.sender.receive({'ocs_run':dict(state=state,run_id=None)},0)
            self.assertFalse(self.sender.status(0)['run_confirmed'])

    def test_new_declaration_revokes_admitted_run(self):
        gate=RunGate(True)
        gate.update(dict(run_id=7,remote_state='RUNNING',run_confirmed=True),0)
        self.assertTrue(gate.admit(0,True))
        gate.update(dict(remote_run_id=7,remote_state='DECLARED',run_confirmed=False),1)
        self.assertFalse(gate.admit(1,True))
        self.assertTrue(RunGate(False).admit(1,False))

    def test_end_directive_is_metadata_not_a_vehicle_action(self):
        self.run_snapshot()
        self.sender.receive({'ocs_directive':dict(action='run_end',run_id=70000)},1)
        self.assertEqual(self.sender.remote_state,'ENDED')
        self.assertFalse(self.sender.status(1)['run_confirmed'])

    def test_tiers_are_complete_and_never_queued_disconnected(self):
        link=WifiLink('127.0.0.1');self.addCleanup(link.close)
        with self.assertRaises(ValueError):link.send_tiers({'task2':'TIER_CORE'})
        tiers={f'task{i}':'TIER_NONE' for i in range(1,5)}
        with self.assertRaisesRegex(ValueError,'not queued'):link.send_tiers(tiers)
        link.sock=Mock()
        link.send_tiers(tiers,'survey')
        frame=link.sock.sendall.call_args.args[0]
        self.assertEqual(json.loads(frame[4:])['ocs_tiers']['vehicle_id'],'UUV1')

    def test_discovery_team_course_guard_freshness_and_fallback(self):
        clock=[0.];chooser=TargetChooser(['10.0.0.2','10.0.0.3'],37564,clock=lambda:clock[0])
        listener=BeaconListener(37565,'ASTA',chooser,log=lambda _:None)
        data=json.dumps(dict(ocs_beacon=dict(team='ASTA',v=1,port=37564))).encode()
        self.assertFalse(listener.handle(data,'192.168.65.2'))
        self.assertTrue(listener.handle(data,'10.0.0.9'))
        self.assertEqual(chooser.choose()[:2],('10.0.0.9',37564))
        clock[0]=6
        self.assertEqual(chooser.choose()[0],'10.0.0.2')
        self.assertEqual(chooser.choose()[0],'10.0.0.3')
        self.assertEqual(check_fallback_hosts(['192.168.68.1'])[0],[])

    def test_course_hostname_resolution_is_rejected_before_dial(self):
        with patch('socket.getaddrinfo',return_value=[(2,1,6,'',('192.168.67.1',37564))]),patch('socket.create_connection') as dial:
            with self.assertRaises(OSError):WifiLink._connect('ocs.example',37564)
            dial.assert_not_called()


class TestLaptopTransport(unittest.TestCase):
    def test_only_local_known_services(self):
        self.assertEqual(target('/vehicle/8096/status'),(8096,'/status'))
        for path in ('http://evil/','//evil/','/vehicle/22/status','/api/a\r\nx:y','/\\evil'):
            with self.assertRaises(ValueError):target(path)

    def test_laptop_link_does_not_replay_mutation_after_lost_reply(self):
        sock=Mock();sock.recv.return_value=b''
        link=LaptopLink('127.0.0.1');self.addCleanup(link.close)
        with patch('socket.create_connection',return_value=sock) as dial:
            with self.assertRaises(ConnectionError):link.request(dict(op='http',method='POST',path='/api/waypoint'))
            self.assertEqual(dial.call_count,1)
            self.assertEqual(sock.sendall.call_count,1)
            self.assertFalse(link.status()['connected'])

    def test_laptop_freshness_uses_its_own_clock(self):
        link=LaptopLink('127.0.0.1',clock=lambda:100.)
        link.sock=Mock();self.addCleanup(link.close);link.last_received=99.
        self.assertTrue(link.status()['connected'])
        link.last_received=90.
        self.assertFalse(link.status()['connected'])


class TestRealGcsChain(unittest.TestCase):
    def test_browser_http_over_one_tcp_to_vehicle_backend(self):
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        from robotx_graey_2026.api.gui.gcs_transport import BridgeServer, LaptopHandler
        import http.client
        seen=[]
        class Backend(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                data=b'<html><head></head><body>GRAEY</body></html>' if self.path=='/' else b'{"fresh":true}'
                self.send_response(200);self.send_header('Content-Type','text/html' if self.path=='/' else 'application/json');self.end_headers();self.wfile.write(data)
            def do_POST(self):
                seen.append(self.rfile.read(int(self.headers.get('Content-Length','0'))))
                self.send_response(202);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(b'{"ok":true}')
        backend=ThreadingHTTPServer(('127.0.0.1',0),Backend)
        bridge=BridgeServer(('127.0.0.1',0),backend.server_port)
        link=LaptopLink('127.0.0.1',bridge.server_address[1])
        laptop=ThreadingHTTPServer(('127.0.0.1',0),LaptopHandler);laptop.link=link
        servers=[backend,bridge,laptop]
        threads=[threading.Thread(target=server.serve_forever,daemon=True) for server in servers]
        for thread in threads:thread.start()
        try:
            browser=http.client.HTTPConnection('127.0.0.1',laptop.server_port,timeout=5)
            browser.request('GET','/');response=browser.getresponse();page=response.read()
            self.assertEqual(response.status,200);self.assertIn(b'GRAEY_LAPTOP',page)
            sock=link.sock
            browser.request('GET','/api/communications');response=browser.getresponse();self.assertEqual(json.loads(response.read()),{'fresh':True})
            self.assertIs(link.sock,sock)
            browser.request('POST','/api/waypoint',b'{"action":"start"}',{'Content-Type':'application/json'})
            response=browser.getresponse();response.read();self.assertEqual(response.status,202);self.assertEqual(len(seen),1)
            browser.request('POST','/api/waypoint',b'{}',{'Origin':'https://unrelated.example','Content-Type':'application/json'})
            response=browser.getresponse();response.read();self.assertEqual(response.status,403);self.assertEqual(len(seen),1)
            browser.close()
        finally:
            link.close()
            for server in reversed(servers):server.shutdown();server.server_close()
            for thread in threads:thread.join(2)


class TestVehicleHttpGuards(unittest.TestCase):
    def setUp(self):
        import ast,os,queue,time,tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from urllib.parse import urlparse,parse_qs
        source=Path(__file__).parents[1]/'robotx_graey_2026/api/gui/gui_node.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        handler=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Handler')
        methods=[n for n in handler.body if isinstance(n,ast.FunctionDef) and n.name in ('do_GET','do_POST')]
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.marker=Path(self.folder.name)/'request'
        self.control=dict(online=True,vehicle_fresh=True,armed=False)
        self.comm=dict(fresh=True,connection='connected',data=dict(ocs_run={'tiers':{}},remote_state='ENDED'))
        self.ns=dict(json=json,os=os,time=time,queue=queue,urlparse=urlparse,parse_qs=parse_qs,
            control_bridge=SimpleNamespace(snapshot=lambda:self.control),any_mission_running=lambda:False,
            REQUESTS={'shutdown':str(self.marker)},communications=SimpleNamespace(snapshot=lambda:self.comm),
            tiers_requests=queue.Queue(maxsize=1))
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),self.ns)

    def test_power_refused_armed_stale_unknown_or_mission_running(self):
        for sample in (dict(online=True,vehicle_fresh=True,armed=True),dict(online=False,armed=False),dict(online=True,vehicle_fresh=True)):
            self.control=sample
            handler=Mock(path='/api/power?do=shutdown',headers={})
            self.ns['do_GET'](handler)
            self.assertEqual(handler.reply_json.call_args.args[0],409)
            self.assertFalse(self.marker.exists())
        self.control=dict(online=True,vehicle_fresh=True,armed=False)
        self.ns['any_mission_running']=lambda:True
        handler=Mock(path='/api/power?do=shutdown',headers={})
        self.ns['do_GET'](handler)
        self.assertFalse(self.marker.exists())

    def test_tier_http_cross_origin_and_active_run_are_refused(self):
        import io
        from email.message import Message
        headers=Message();headers['Host']='graey.local:8090';headers['Origin']='https://unrelated.example';headers['Content-Type']='application/json'
        handler=Mock(path='/api/communications/tiers',headers=headers)
        self.ns['do_POST'](handler)
        self.assertEqual(handler.reply_json.call_args.args[0],403)
        del headers['Origin'];headers['Origin']='http://graey.local:8090'
        body=json.dumps(dict(tiers={f'task{i}':'TIER_CORE' for i in range(1,5)})).encode()
        headers['Content-Length']=str(len(body));handler.rfile=io.BytesIO(body)
        self.comm['data']['remote_state']='RUNNING'
        self.ns['do_POST'](handler)
        self.assertEqual(handler.reply_json.call_args.args[0],409)
        self.assertTrue(self.ns['tiers_requests'].empty())
