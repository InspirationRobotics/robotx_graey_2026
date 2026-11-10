import tempfile
import unittest
from pathlib import Path
from robotx_graey_2026.api.gui.log_reader import read_log
from robotx_graey_2026.api.gui.system_info import SystemInfo


class TestGroundstationServices(unittest.TestCase):
    def test_log_append_is_incremental_and_truncation_resets(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'log';path.write_text('first\n')
            first=read_log(path)
            with path.open('a') as f:f.write('WARN second\n')
            second=read_log(path,first['cursor'],first['generation'])
            self.assertEqual(second['lines'],['WARN second'])
            self.assertFalse(second['reset'])
            path.write_text('reset\n')
            self.assertTrue(read_log(path,second['cursor'],second['generation'])['reset'])

    def test_cursor_moves_even_when_filter_removes_lines(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'log';path.write_text('info\nWARN attention\n')
            result=read_log(path,errors=True)
            self.assertEqual(result['lines'],['WARN attention'])
            self.assertEqual(read_log(path,result['cursor'],result['generation'])['lines'],[])

    def test_missing_system_measurements_remain_unknown(self):
        with tempfile.TemporaryDirectory() as folder:
            result=SystemInfo(proc=folder,thermal=folder,disk=folder).snapshot()
            self.assertIsNone(result['cpu_percent'])
            self.assertIsNone(result['memory_used_bytes'])
            self.assertIsNone(result['temperature_c'])
            self.assertGreater(result['disk_total_bytes'],0)
