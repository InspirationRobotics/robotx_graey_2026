"""Verify timer-clock selection without requiring ROS on the test host."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class SteadyNodeTest(unittest.TestCase):
    def test_defaults_to_steady_but_preserves_explicit_clock(self):
        class FakeNode:
            def __init__(self,*args,**kwargs): pass
            def create_timer(self,*args,**kwargs): return kwargs['clock']
        tree=ast.parse((Path(__file__).parents[1]/'robotx_graey_2026/api/steady_node.py').read_text())
        tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        namespace=dict(Node=FakeNode, Clock=lambda **kw:kw['clock_type'],
                       ClockType=SimpleNamespace(STEADY_TIME='steady'))
        exec(compile(tree,'steady_node.py','exec'),namespace)
        node=namespace['SteadyNode']('test')
        self.assertEqual(node.create_timer(.1,lambda:None),'steady')
        self.assertEqual(node.create_timer(.1,lambda:None,clock='explicit'),'explicit')
