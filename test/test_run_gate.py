import unittest
from robotx_graey_2026.api.comms.run_gate import RunGate


class TestRunGate(unittest.TestCase):
    def test_local_missions_independent(self):
        self.assertTrue(RunGate().admit(0,False))

    def test_unknown_or_stale_run_does_not_start(self):
        gate=RunGate(True)
        self.assertFalse(gate.admit(0,True))
        gate.update(dict(run_confirmed=True,remote_state='RUNNING',run_id=7),0)
        self.assertFalse(gate.admit(3,True))

    def test_only_fresh_verified_run_and_ready_can_start(self):
        gate=RunGate(True)
        gate.update(dict(run_confirmed=True,remote_state='RUNNING',run_id=70000),0)
        self.assertFalse(gate.admit(0,False))
        self.assertTrue(gate.admit(0,True))
        self.assertEqual(gate.run_id,70000)
        self.assertTrue(gate.admit(1000,True))  # submerged disconnection

    def test_ended_or_different_run_revokes(self):
        for update in (dict(remote_state='ENDED',run_id=7),dict(run_confirmed=True,remote_state='RUNNING',run_id=8)):
            gate=RunGate(True)
            gate.update(dict(run_confirmed=True,remote_state='RUNNING',run_id=7),0)
            self.assertTrue(gate.admit(0,True))
            gate.update(update,1)
            self.assertFalse(gate.admit(1,True))
