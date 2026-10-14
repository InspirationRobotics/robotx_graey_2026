"""Continuous-only migration contracts and mission permission failure paths."""
import ast
from pathlib import Path
import unittest
from robotx_graey_2026.api.navigation.continuous_supervisor import ContinuousSupervisor, Observation

ROOT = Path(__file__).parents[1]

class ContinuousOnlyTests(unittest.TestCase):
    def test_no_source_command_or_legacy_bridge_entrypoint(self):
        nav = ROOT/'robotx_graey_2026/api/navigation'
        self.assertFalse((nav/'supervisor_logic.py').exists())
        self.assertFalse((nav/'nav_ekf_bridge.py').exists())
        for name in ('navigation_supervisor.py', 'continuous_supervisor.py', 'fused_nav_bridge.py'):
            text = (nav/name).read_text()
            self.assertNotIn('42007', text)
            self.assertNotIn('select_source', text)
            self.assertNotIn('NAV_SRC', text)
        self.assertNotIn('navigation.nav_ekf_bridge:main', (ROOT/'setup.py').read_text())

    def test_launch_has_one_unconditional_bridge(self):
        for name in ('core.launch.py', 'sitl.launch.py'):
            tree = ast.parse((ROOT/'launch'/name).read_text())
            bridges = []
            for n in ast.walk(tree):
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'Node':
                    kw = {k.arg:k.value for k in n.keywords}
                    exe = kw.get('executable')
                    if isinstance(exe, ast.Constant) and exe.value in ('nav_ekf_bridge', 'fused_nav_bridge'):
                        bridges.append(exe.value)
                        self.assertNotIn('condition', kw)
            self.assertEqual(bridges, ['fused_nav_bridge'])

    def test_heading_required_before_capture_and_new_run(self):
        s = ContinuousSupervisor(active=True)
        o = Observation(healthy=True, depth=0., depth_valid=True,
            surface_authoritative=True, surface_observed=True,
            configuration_verified=True, gps_good=True, intent='DIVE',
            intent_fresh=True, run_id='first')
        self.assertEqual(s.step(0., o), [])
        self.assertFalse(s.allowed)
        o.heading_allowed = True
        self.assertEqual(s.step(.1, o), [('capture_reference', None)])
        o.reference_ready = o.reference_saved = True
        o.surface_observed = False
        o.gps_good = False
        self.assertEqual(s.step(.2, o), [])
        self.assertTrue(s.allowed)
        o.run_id = 'second'
        s.step(.3, o)
        self.assertFalse(s.allowed)

    def test_health_depth_intent_and_configuration_gate(self):
        for field, value in (('healthy',False),('depth_valid',False),
                             ('intent_fresh',False),('configuration_verified',False)):
            s = ContinuousSupervisor(active=True)
            o = Observation(healthy=True, depth=0., depth_valid=True,
                surface_authoritative=True, surface_observed=True,
                configuration_verified=True, gps_good=True, intent='SURFACE',
                intent_fresh=True, run_id='one')
            s.step(0.,o); self.assertTrue(s.allowed)
            setattr(o,field,value)
            self.assertEqual(s.step(.1,o),[])
            self.assertFalse(s.allowed)
