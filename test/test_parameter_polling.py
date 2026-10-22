"""Exercise production polling against a deliberately bounded reply queue."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from robotx_graey_2026.api.navigation.odometry_profile import PROFILE
COMPASS_PARAMS=tuple(PROFILE)+('COMPASS_LEARN',)


class ParameterPollingTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/fused_nav_bridge.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FusedNavBridge')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='request')
        module=ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[]))
        namespace=dict(PROFILE=PROFILE,COMPASS_PARAMS=COMPASS_PARAMS)
        exec(compile(module,str(path),'exec'),namespace)
        self.poll=namespace['request']
        self.node=NS(output_permitted=True,settings={},link=Mock(),sync_clock=Mock())
        self.received={};self.round_names=[];self.now=0.
        def send(kind,*args):
            if kind!='param_request_read_send':return
            name=args[2].decode();self.round_names.append(name)
            # Later replies disappear in a fixed-size queue. The old burst
            # restarts at PROFILE's first field every round and never recovers.
            if len(self.round_names)<=16:self.received[name]=self.now
        self.node.link.send.side_effect=send

    def tick(self,now):
        self.now=now;self.round_names=[];self.poll(self.node)
        self.assertLessEqual(len(self.round_names),16)
        self.assertEqual(len(self.round_names),len(set(self.round_names)))
        return list(self.round_names)

    def test_all_navigation_and_compass_fields_refresh_under_bounded_reply_queue(self):
        for compass in (False,True):
            self.node.output_permitted=not compass;self.received.clear()
            required=set(COMPASS_PARAMS)
            for t in range(10):
                self.tick(float(t))
                if t>=1:
                    self.assertTrue(required.issubset(self.received))
                    self.assertTrue(all(0<=t-self.received[k]<3 for k in required))

    def test_output_permission_does_not_reset_polling_cursor(self):
        self.tick(0.);self.node.output_permitted=False
        self.assertEqual(self.tick(1.),list(COMPASS_PARAMS)[16:])
        self.assertTrue(set(COMPASS_PARAMS).issubset(self.received))

    def test_polling_never_writes_sources_or_commands_motion(self):

        self.tick(0.);self.tick(1.)
        self.assertTrue(set(COMPASS_PARAMS).issubset(self.received))
        self.assertTrue(all(c.args[0]=='param_request_read_send' for c in self.node.link.send.call_args_list))
        self.node.link.set_mode.assert_not_called()
        self.node.link.command.assert_any_call(511,24,100000)
        self.node.sync_clock.assert_called()
