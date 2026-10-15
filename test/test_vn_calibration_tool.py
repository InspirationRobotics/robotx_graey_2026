import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import MagicMock, patch


ROOT = Path(__file__).parents[1]


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CalibrationToolTests(unittest.TestCase):
    def test_interrupt_stops_learning_and_restores_serial_ownership(self):
        from pymavlink import mavutil
        import serial
        tool = load_tool('vn_calibrate')
        owner, port, link = MagicMock(), MagicMock(), MagicMock()
        heartbeat = SimpleNamespace(base_mode=0, get_srcSystem=lambda:1, get_srcComponent=lambda:1)
        link.recv_match.side_effect = [heartbeat, None]
        def command(unused, body):
            if body == 'VNRRG,08':
                raise KeyboardInterrupt()
            return ['1','1','1','1']
        with tempfile.TemporaryDirectory() as folder:
            args = ['vn_calibrate.py','--calibrate','--killed-disarmed','--output',str(Path(folder)/'log.jsonl')]
            with patch('sys.argv', args), patch.object(tool, 'ProcessOwnership', return_value=owner), \
                 patch.object(serial, 'Serial', return_value=port), \
                 patch.object(mavutil, 'mavlink_connection', return_value=link), \
                 patch.object(tool, 'relative_heading'), \
                 patch.object(tool, 'transact', side_effect=command) as transactions:
                with self.assertRaises(KeyboardInterrupt):
                    tool.main()
                commands = [call.args[1] for call in transactions.call_args_list]
                self.assertEqual(commands[-1], 'VNWRG,44,0,3,1')
                self.assertIn('VNWRG,44,2,3,1', commands)
                self.assertFalse(any('VNWNV' in body for body in commands))
                owner.__exit__.assert_called_once()
                port.__exit__.assert_called_once()
                link.close.assert_called_once()

    def test_coverage_requires_distinct_gravity_faces(self):
        tool = load_tool('vn_calibrate')
        faces = [tool.gravity_face(p) for p in ((0,0,0),(0,0,180),(0,90,0),
                    (0,-90,0),(0,0,90),(0,0,-90))]
        self.assertEqual(set(faces), set(range(6)))

    def test_watcher_does_not_call_unaligned_or_uncommissioned_ready(self):
        tool = load_tool('prepare_navigation')
        state = dict(north_alignment=dict(aligned=False,reason='Need movement'))
        nav = dict(streams=dict(supervisor=dict(fresh=True,age=0.,data=state)))
        self.assertFalse(tool.assess(nav)[0])
        state['north_alignment']['aligned'] = True
        nav['cube_estimate'] = dict(valid=True)
        nav['streams']['attitude'] = dict(fresh=True)
        state.update(surfaced=True, state='GPS aiding admitted')
        self.assertFalse(tool.assess(nav)[0])
        state.update(configuration_verified=True, mode='Active',dive_heading_check=dict(allowed=True))
        self.assertTrue(tool.assess(nav)[0])
