import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from mavproxy_mission_gate import hook_source,install_continuous_broadcast
from mission_input_gate import MissionInputGate

class NativeGotoTests(unittest.TestCase):
    def test_broadcast_destination_survives_client_reply(self):
        import socket
        from pymavlink import mavutil
        link=SimpleNamespace(destination_addr=('192.168.2.255',14570),port=Mock())
        original=Mock(return_value=link)
        with patch.object(mavutil,'mavlink_connection',original):
            install_continuous_broadcast()
            connection=mavutil.mavlink_connection
            self.assertIs(connection('udpall:192.168.2.255:14570'),link)
            original.assert_called_once_with('udpbcast:192.168.2.255:14570')
            link.destination_addr=('192.168.2.7',14570)
            link.write(b'telemetry')
            link.port.sendto.assert_called_with(b'telemetry',('192.168.2.255',14570))
            link.port.sendto.side_effect=socket.error('temporary network loss')
            link.write(b'telemetry')
            connection('udpin:127.0.0.1:14551')
            original.assert_called_with('udpin:127.0.0.1:14551')

    def test_native_commands_forward_without_router_or_mode_rewrite(self):
        source="def process():\n    for m in msgs:\n        if True:\n            output.write(m.get_msgbuf())\n"
        built=hook_source(source)
        self.assertNotIn('_graey_qgc_goto',built)
        gate=MissionInputGate()
        class Output:
            def __init__(self):self.sent=[]
            def write(self,data):self.sent.append(data)
        output=Output()
        msgs=[SimpleNamespace(get_type=lambda k=k:k,get_srcSystem=lambda:255,get_srcComponent=lambda:190,
            command=c,get_msgbuf=lambda:b'original') for k,c in [('COMMAND_INT',192),('MISSION_ITEM_INT',16)]]
        # Call compiled function with the same namespace.
        ns=dict(msgs=msgs,output=output,_graey_input_gate=gate)
        exec(compile(built,'native-goto','exec'),ns);ns['process']()
        self.assertEqual(output.sent,[b'original',b'original'])

if __name__=='__main__':unittest.main()
