"""Unexpected callback exceptions must not become clean process exits."""
import ast
import contextlib
import io
from pathlib import Path
from types import SimpleNamespace
import traceback
import unittest
from unittest.mock import Mock


class NodeFailureTests(unittest.TestCase):
    def test_callback_failure_is_logged_and_exits_nonzero(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/node_util.py'
        tree=ast.parse(path.read_text())
        tree.body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_run']
        node=Mock()
        def exit_process(code): raise SystemExit(code)
        ns=dict(rclpy=SimpleNamespace(init=Mock(),spin=Mock(side_effect=RuntimeError('callback failed')),
                                    ok=lambda:True,shutdown=Mock()),
                traceback=traceback,sys=SimpleNamespace(stdout=io.StringIO(),stderr=io.StringIO()),
                os=SimpleNamespace(_exit=exit_process))
        exec(compile(tree,str(path),'exec'),ns)
        error=io.StringIO()
        with contextlib.redirect_stderr(error),self.assertRaises(SystemExit) as caught:
            ns['_run'](lambda:node)
        self.assertEqual(caught.exception.code,1)
        self.assertIn('callback failed',error.getvalue())
        node.destroy_node.assert_called_once()
