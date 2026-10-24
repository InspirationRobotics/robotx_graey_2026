import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.relative_estimator import RelativeEstimator


class SegmentedBridgeTests(unittest.TestCase):
    def test_correction_requires_authorized_stopped_fresh_check(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/fused_nav_bridge.py'
        tree=ast.parse(path.read_text())
        method=next(n for c in tree.body if isinstance(c,ast.ClassDef)
                    for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='mission_alignment_status')
        module=ast.Module(body=[method],type_ignores=[])
        ns=dict(json=json,math=math,time=NS(monotonic=lambda:10.),RelativeEstimator=RelativeEstimator)
        exec(compile(module,str(path),'exec'),ns)
        e=RelativeEstimator();e.motion_at=10.;e.motion_valid=True
        e.status=Mock(return_value=dict(healthy=True,gps_admission=dict(qualified=True)))
        e.apply_segment_correction=Mock()
        node=NS(est=e,control_received=10.,segmented_course_run='r',control_state=dict(
            mission_claimed=True,mode='STABILIZE',armed=True,sa_active=False,esc_commanded_on=True))
        message=dict(published_monotonic=10.,run_id='r',stage='TRANSIT_CHECK',segment_checks=2,
                     plan=dict(lat=33.,lon=-117.))
        def send():ns['mission_alignment_status'](node,NS(data=json.dumps(message)))
        e.raw_velocity=(.2,0.,0.);send();e.apply_segment_correction.assert_not_called()
        e.raw_velocity=(0.,0.,0.);message['stage']='TRANSIT';send();e.apply_segment_correction.assert_not_called()
        message['stage']='TRANSIT_CHECK';node.control_state['sa_active']=True
        send();e.apply_segment_correction.assert_not_called()
        node.control_state['sa_active']=False;e.motion_at=0.
        send();e.apply_segment_correction.assert_not_called()
        e.motion_at=10.;send();e.apply_segment_correction.assert_called_once_with(10.,('r',2))
