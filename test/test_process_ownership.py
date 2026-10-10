"""Real OS contention/crash tests plus GUI handler and launch policy checks."""
import ast
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from urllib.parse import urlparse, parse_qs

from robotx_graey_2026.api.process_ownership import ProcessOwnership, executable_pids
from robotx_graey_2026.api.navigation.bridge_identity import BridgeIdentity

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(sys.platform == 'linux', 'Production locks require Linux flock')
class LockTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.env = dict(os.environ, GRAEY_LOCK_DIR=self.directory.name,
                        GRAEY_RUNTIME_SCOPE='test', PYTHONPATH=str(ROOT))

    def contender(self):
        script = '''from robotx_graey_2026.api.process_ownership import ProcessOwnership
with ProcessOwnership('bridge'):
    print('ACQUIRED', flush=True)
'''
        return subprocess.run([sys.executable, '-c', script], env=self.env,
                              capture_output=True, text=True, timeout=10)

    def test_twenty_simultaneous_starts_cannot_displace_owner(self):
        with ProcessOwnership('bridge', self.directory.name, 'test'):
            with ThreadPoolExecutor(max_workers=10) as pool:
                results = list(pool.map(lambda _: self.contender(), range(20)))
            self.assertTrue(all(r.returncode != 0 for r in results))
            self.assertTrue(all('duplicate start rejected' in r.stderr for r in results))
            self.assertTrue(all('ACQUIRED' not in r.stdout for r in results))
        self.assertEqual(self.contender().returncode, 0)

    def test_crash_releases_lock_without_deleting_file(self):
        script = '''from robotx_graey_2026.api.process_ownership import ProcessOwnership
import time
with ProcessOwnership('bridge'):
    print('READY', flush=True)
    time.sleep(30)
'''
        child = subprocess.Popen([sys.executable, '-c', script], env=self.env,
                                 stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'READY')
            self.assertNotEqual(self.contender().returncode, 0)
        finally:
            child.kill(); child.wait(timeout=5); child.stdout.close()
        self.assertEqual(self.contender().returncode, 0)
        self.assertEqual(len(list(Path(self.directory.name).glob('*.lock'))), 1)

    def test_scopes_isolate_simulation_without_bypassing_same_scope(self):
        with ProcessOwnership('bridge', self.directory.name, 'vehicle'):
            self.assertEqual(self.contender().returncode, 0)


class ProcessDisplayTests(unittest.TestCase):
    def test_gui_spawn_cannot_bypass_core_ownership(self):
        tree = ast.parse((ROOT / 'robotx_graey_2026/api/gui/gui_node.py').read_text())
        names = {'GROUPS', 'CAMERA', 'CORE_MANAGED_GROUPS', 'CORE_MANAGED_EXECUTABLES'}
        nodes = [n for n in tree.body if
                 (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
                  t.id in names for t in n.targets)) or
                 (isinstance(n, ast.FunctionDef) and n.name == 'spawn')]
        process = Mock(DEVNULL=-3, STDOUT=-2)
        ns = dict(os=os, subprocess=process)
        exec(compile(ast.Module(body=nodes, type_ignores=[]), 'gui_node.py', 'exec'), ns)
        for name in ('dvl_node', 'vn100_node', 'nav_ekf_bridge', 'navigation_supervisor'):
            for argv in (['ros2', 'run', 'robotx_graey_2026', name],
                         ['/usr/bin/python3', '/install/lib/robotx_graey_2026/'+name]):
                with self.subTest(argv=argv), self.assertRaises(ValueError):
                    ns['spawn'](argv)
        process.Popen.assert_not_called()
        ns['spawn'](['ros2', 'run', 'robotx_graey_2026', 'pole_tracker'])
        process.Popen.assert_called_once()

    def test_wrappers_and_shell_strings_not_counted_as_nodes(self):
        with tempfile.TemporaryDirectory() as root:
            rows = {
                10001: ['/usr/bin/python3', '/opt/ros/bin/ros2', 'run', 'pkg', 'nav_ekf_bridge'],
                10002: ['/usr/bin/python3', '/install/lib/pkg/nav_ekf_bridge'],
                10003: ['/usr/bin/python3', '/install/lib/pkg/nav_ekf_bridge', '--ros-args'],
                10004: ['/bin/bash', '-c', 'nav_ekf_bridge'],
            }
            for pid, args in rows.items():
                d = Path(root) / str(pid); d.mkdir()
                (d / 'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
            self.assertEqual(executable_pids('nav_ekf_bridge', root), [10002, 10003])

    def test_gui_backend_rejects_start_and_stop_before_process_actions(self):
        tree = ast.parse((ROOT / 'robotx_graey_2026/api/gui/gui_node.py').read_text())
        # Execute the actual handler; no ROS, HTTP server or vehicle connection.
        handler = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                       and any(isinstance(f, ast.FunctionDef) and f.name == 'do_GET' for f in n.body))
        ns = dict(BaseHTTPRequestHandler=object, urlparse=urlparse, parse_qs=parse_qs,
                  CORE_MANAGED_GROUPS={'nav'}, spawn=Mock(), pids_for=Mock())
        exec(compile(ast.Module(body=[handler], type_ignores=[]), 'gui_node.py', 'exec'), ns)
        for action in ('start', 'stop'):
            h = ns[handler.name]()
            h.path = '/api/'+action+'?group=nav'
            h.reply_json = Mock()
            h.do_GET()
            self.assertEqual(h.reply_json.call_args.args[0], 409)
        ns['spawn'].assert_not_called(); ns['pids_for'].assert_not_called()

    def test_launch_does_not_respawn_stateful_bridge(self):
        tree = ast.parse((ROOT / 'launch/core.launch.py').read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'Node']
        bridge = next(n for n in calls if any(k.arg == 'executable'
                      and isinstance(k.value, ast.Constant) and k.value.value == 'nav_ekf_bridge'
                      for k in n.keywords))
        self.assertFalse(any(k.arg is None for k in bridge.keywords))
        self.assertFalse(next(k.value.value for k in bridge.keywords if k.arg == 'respawn'))


class IdentityTests(unittest.TestCase):
    def test_alternating_publishers_detected_without_claiming_a_crash(self):
        tracker = BridgeIdentity()
        self.assertEqual(tracker.observe('A', 0), '')
        self.assertIn('Multiple', tracker.observe('B', .1))
        self.assertIn('Multiple', tracker.observe('A', .2))

    def test_sequential_restart_and_missing_identity(self):
        tracker = BridgeIdentity()
        tracker.observe('A', 0)
        self.assertIn('restarted', tracker.observe('B', 1))
        self.assertEqual(tracker.observe('B', 1.1), '')
        self.assertIn('invalid', tracker.observe(None, 2))


if __name__ == '__main__':
    unittest.main()
