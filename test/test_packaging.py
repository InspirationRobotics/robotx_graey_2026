"""Installed assets and component identity regressions."""
import ast
from pathlib import Path
import unittest
from unittest.mock import patch
import runpy
import os

ROOT = Path(__file__).parents[1]


class PackagingTests(unittest.TestCase):
    def test_offline_map_dependencies_are_installed(self):
        previous = Path.cwd()
        try:
            os.chdir(ROOT)
            with patch('setuptools.setup') as setup:
                runpy.run_path('setup.py', run_name='__test__')
            installed = {Path(p).as_posix() for _, paths in setup.call_args.kwargs['data_files'] for p in paths}
            self.assertIn('tools/vendor/leaflet/leaflet.js', installed)
            self.assertIn('tools/vendor/leaflet/leaflet.css', installed)
            self.assertIn('tools/vendor/leaflet/LICENSE', installed)
        finally:
            os.chdir(previous)

    def test_core_mavlink_components_are_unique(self):
        ids = []
        for path in ('led/pixhawk_led_node.py', 'pixhawk/kill_switch.py',
                     'navigation/nav_ekf_bridge.py', 'navigation/navigation_supervisor.py',
                     'gui/gui_node.py', 'navigation/pos_server.py'):
            tree = ast.parse((ROOT/'robotx_graey_2026/api'/path).read_text())
            calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Name) and n.func.id == 'Link']
            self.assertEqual(len(calls), 1)
            ids.append(ast.literal_eval(calls[0].args[1]))
        self.assertEqual(len(ids), len(set(ids)))
