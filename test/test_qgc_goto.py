import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from mavproxy_mission_gate import hook_source
from mission_input_gate import MissionInputGate

class NativeGotoTests(unittest.TestCase):
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
