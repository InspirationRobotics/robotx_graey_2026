"""An endless MAVLink producer must yield to mission/health timers."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class ReceiveBudgetTests(unittest.TestCase):
    def test_continuous_producer_yields(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/pixhawk/mavlink.py'
        tree=ast.parse(path.read_text())
        tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Link']
        ns=dict(time=SimpleNamespace(monotonic=lambda:0.))
        exec(compile(tree,str(path),'exec'),ns)
        link=ns['Link'].__new__(ns['Link'])
        link.mav=SimpleNamespace(recv_match=Mock(return_value=SimpleNamespace(get_type=lambda:'HEARTBEAT')))
        callback=Mock()
        link.drain(callback)
        self.assertEqual(callback.call_count,256)
