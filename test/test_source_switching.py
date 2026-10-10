"""Source handshake fault injection against production state-machine logic."""
import unittest
from robotx_graey_2026.api.navigation.supervisor_logic import Supervisor, Observation


class SourceSwitchTests(unittest.TestCase):
    def setUp(self):
        self.s = Supervisor(active=True)
        self.s.pending = (2, 10.)
        self.o = Observation(healthy=True, depth_valid=True, depth=.1,
                             intent_fresh=True, source=1)

    def step(self, t, **values):
        for k, v in values.items():
            setattr(self.o, k, v)
        self.s.step(t, self.o)

    def test_ack_then_source(self):
        self.step(10.1, ack='accepted', ack_updated=10.1)
        self.assertIsNotNone(self.s.pending)
        self.step(10.3, source=2, source_updated=10.3)
        self.assertIsNone(self.s.pending)
        self.assertFalse(self.s.fault)

    def test_source_then_ack(self):
        self.step(10.1, source=2, source_updated=10.1)
        self.assertIsNotNone(self.s.pending)
        self.step(10.3, ack='accepted', ack_updated=10.3)
        self.assertIsNone(self.s.pending)

    def test_late_confirmation_cannot_bypass_deadline(self):
        self.step(13.1, source=2, source_updated=13.1, ack='accepted', ack_updated=13.1)
        self.assertIn('timeout', self.s.fault)
        self.assertFalse(self.s.allowed)

    def test_missing_ack_explained(self):
        self.step(13.1, source=2, source_updated=12.9)
        self.assertIn('missing accepted command ACK', self.s.fault)
        self.assertNotIn('fresh Surface', self.s.fault)

    def test_later_source_reports_do_not_erase_timely_evidence(self):
        self.step(10.2, source=2, source_updated=10.2)
        self.step(13.1, source_updated=13.1)
        self.assertIn('missing accepted command ACK', self.s.fault)
        self.assertNotIn('source feedback', self.s.fault)

    def test_missing_feedback_explained(self):
        self.step(13.1, ack='accepted', ack_updated=12.9)
        self.assertIn('fresh Surface GPS source feedback', self.s.fault)
        self.assertNotIn('missing accepted command ACK', self.s.fault)

    def test_pre_request_ack_cannot_confirm(self):
        self.step(13.1, source=2, source_updated=12.9, ack='accepted', ack_updated=9.9)
        self.assertIn('accepted command ACK', self.s.fault)

    def test_future_timestamps_cannot_confirm(self):
        self.step(10.2, source=2, source_updated=11., ack='accepted', ack_updated=11.)
        self.assertIsNotNone(self.s.pending)

    def test_rejected_and_discontinuous_transitions_latch(self):
        self.step(10.2, ack='rejected', ack_updated=10.1)
        self.assertIn('rejected', self.s.fault)
        self.step(10.4, source=2, source_updated=10.3, ack='accepted', ack_updated=10.3)
        self.assertFalse(self.s.allowed)

    def test_continuity_failure(self):
        self.step(10.2, source=2, source_updated=10.1, ack='accepted', ack_updated=10.1,
                  continuity_ok=False)
        self.assertIn('discontinuity', self.s.fault)

    def test_clock_rollback_latches_fault(self):
        self.step(10.2)
        self.step(9.)
        self.assertIn('clock moved backwards', self.s.fault)
        self.assertFalse(self.s.allowed)
