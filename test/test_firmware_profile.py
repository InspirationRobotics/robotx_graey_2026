import json
import os
import subprocess
import sys
import unittest
import ast
from pathlib import Path
from types import SimpleNamespace


class FirmwareProfileTests(unittest.TestCase):
    def test_parameter_poll_spreads_complete_profile_without_tail_starvation(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/fused_nav_bridge.py'
        tree=ast.parse(path.read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='request_parameters')
        profile={f'PARAM{i}':i for i in range(25)}
        ns={'PROFILE':profile}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),ns)
        sent=[];node=SimpleNamespace(link=SimpleNamespace(send=lambda *args:sent.append(args[3].decode())))
        for _ in range(7):
            before=len(sent);ns['request_parameters'](node)
            self.assertLessEqual(len(sent)-before,4)
        self.assertEqual(sent,list(profile)+['COMPASS_LEARN'])
        self.assertEqual(node.parameter_cursor,0)

    def load(self, **settings):
        env=os.environ.copy()
        for key in ('GRAEY_FIRMWARE_PROFILE','GRAEY_RUNTIME_SCOPE','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY'):
            env.pop(key,None)
        env.update(settings)
        return subprocess.run([sys.executable,'-c',
            'import json; from robotx_graey_2026.api.navigation.odometry_profile import PROFILE,SHALLOW_LIMITS; print(json.dumps([PROFILE,SHALLOW_LIMITS]))'],
            env=env,text=True,capture_output=True)

    def test_default_preserves_live_45(self):
        p=self.load();self.assertEqual(p.returncode,0,p.stderr)
        profile,limits=json.loads(p.stdout)
        self.assertEqual(profile['GPS_TYPE'],1)
        self.assertEqual(limits['WPNAV_SPEED_DN'],10.)

    def test_explicit_47_profile_converts_units(self):
        p=self.load(GRAEY_FIRMWARE_PROFILE='4.7')
        self.assertEqual(p.returncode,0,p.stderr)
        profile,limits=json.loads(p.stdout)
        self.assertEqual(profile['GPS1_TYPE'],1)
        self.assertNotIn('GPS_TYPE',profile)
        self.assertEqual(limits,dict(WP_SPD_DN=.10,WP_SPD_UP=.15,WP_ACC_Z=.50))

