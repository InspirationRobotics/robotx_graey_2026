import random
import unittest
from dataclasses import replace
from robotx_graey_2026.api.navigation.supervisor_logic import Observation, Supervisor


class Rig:
    def __init__(self, active=True):
        self.s = Supervisor(active=active)
        self.o = Observation(healthy=True, depth=.1, depth_valid=True, intent='SURFACE',
            intent_fresh=True, run_id='dive1', gps_good=True, source=1,
            configuration_verified=True)
        self.t = 0.
        self.commands = []
        self.auto = True

    def run(self, seconds):
        for _ in range(round(seconds*10)):
            self.t = round(self.t+.1, 3)
            self.o.gps_updated = self.t if self.o.gps_good else self.o.gps_updated
            self.o.source_updated = self.t
            actions = self.s.step(self.t, self.o)
            for kind, value in actions:
                if kind == 'select_source':
                    self.commands.append(value)
                    if self.auto:
                        self.o.source = value
                        self.o.ack, self.o.ack_updated = 'accepted', self.t+.001
                elif kind == 'align_external_position' and self.auto:
                    self.o.alignment_id = value
                elif kind == 'capture_reference':
                    self.o.reference_ready = self.o.reference_saved = True
        return self


class SupervisorTests(unittest.TestCase):
    def test_surface_dive_capture_align_switch_sequence(self):
        r = Rig().run(5)
        self.assertTrue(r.s.allowed)
        r.o.intent = 'DIVE'; r.run(.1)
        self.assertFalse(r.s.allowed)
        r.run(2)
        self.assertEqual(r.commands, [2, 1])
        self.assertTrue(r.o.reference_saved)
        self.assertTrue(r.o.alignment_id)
        self.assertTrue(r.s.allowed)

    def test_observation_sends_no_source_commands(self):
        r = Rig(active=False).run(20)
        self.assertEqual(r.commands, [])
        self.assertFalse(r.s.allowed)
        self.assertEqual(r.s.snapshot(r.o)['requested_source'], 'Surface GPS')

    def test_brief_gps_loss_keeps_surface_source(self):
        r = Rig().run(5); r.o.gps_good = False; r.run(1)
        self.assertTrue(r.s.allowed)
        self.assertEqual(r.commands, [2])
        self.assertEqual(r.s.state, 'Surface GPS degraded')

    def test_long_gps_loss_blocks_transit_and_leaves_gps_source(self):
        r = Rig().run(5); r.o.gps_good = False; r.run(5)
        self.assertFalse(r.s.allowed)
        self.assertEqual(r.commands, [2, 1])
        r.run(20)
        self.assertEqual(r.commands, [2, 1])

    def test_suspect_fix_has_no_dropout_grace(self):
        r = Rig().run(5)
        r.o.gps_good = False
        r.o.gps_rejected = True
        r.run(.1)
        self.assertFalse(r.s.allowed)
        r.run(1)
        self.assertEqual(r.commands, [2, 1])

    def test_bad_gps_cannot_force_switch_with_unhealthy_external_navigation(self):
        r = Rig().run(5)
        r.o.gps_good = False; r.o.gps_rejected = True; r.o.healthy = False
        r.run(10)
        self.assertEqual(r.commands, [2])
        self.assertFalse(r.s.allowed)

    def test_recovery_requires_qualification_again(self):
        r = Rig().run(5); r.o.gps_good = False; r.run(5)
        r.o.gps_good = True; r.run(1)
        # Recovery must not refresh dropout permission before qualification.
        self.assertFalse(r.s.allowed)
        r.run(4); self.assertTrue(r.s.allowed)
        self.assertEqual(r.commands, [2, 1, 2])

    def test_waves_and_sustained_depth(self):
        r = Rig().run(5)
        for _ in range(30):
            r.o.depth = .5; r.run(.2)
            r.o.depth = .1; r.run(.2)
        self.assertEqual(r.commands, [2])
        r.o.depth = .6; r.run(3)
        self.assertEqual(r.commands, [2, 1])

    def test_bad_gps_does_not_enable_surface_source(self):
        r = Rig(); r.o.gps_good = False; r.run(10)
        self.assertEqual(r.commands, [])

    def test_unqualified_depth_is_unknown_and_does_not_request_underwater(self):
        r = Rig(); r.o.depth_valid = False; r.run(10)
        snapshot = r.s.snapshot(r.o)
        self.assertEqual(snapshot['surface_state'], 'Unknown')
        self.assertIsNone(snapshot['surfaced'])
        self.assertEqual(snapshot['requested_source'], 'Unknown')
        self.assertEqual(r.commands, [])

    def test_surface_state_requires_hysteresis_dwell(self):
        r = Rig(); r.o.depth_valid = False; r.run(1)
        r.o.depth_valid = True; r.o.depth = .1; r.run(1.9)
        self.assertEqual(r.s.snapshot(r.o)['surface_state'], 'Unknown')
        r.run(.2)
        self.assertEqual(r.s.snapshot(r.o)['surface_state'], 'Surfaced')

    def test_stale_health_intent_depth_blocks_changes(self):
        for k in ('healthy', 'depth_valid', 'intent_fresh'):
            r = Rig(); setattr(r.o, k, False); r.run(10)
            self.assertEqual(r.commands, [])
            self.assertFalse(r.s.allowed)

    def test_configuration_not_validated_blocks_active(self):
        r = Rig(); r.o.configuration_verified = False; r.run(10)
        self.assertEqual(r.commands, [])

    def test_ack_alone_does_not_confirm(self):
        r = Rig(); r.auto = False; r.run(4)
        r.o.ack = 'accepted'; r.o.ack_updated = r.t
        r.run(4)
        self.assertEqual(r.s.state, 'Fault')
        self.assertEqual(r.commands, [2])

    def test_source_report_alone_does_not_confirm(self):
        r = Rig(); r.auto = False; r.run(4)
        r.o.source = 2; r.run(4)
        self.assertEqual(r.s.state, 'Fault')

    def test_rejected_command_latches(self):
        r = Rig(); r.auto = False; r.run(4)
        r.o.ack = 'rejected'; r.o.ack_updated = r.t; r.run(.2)
        self.assertEqual(r.s.state, 'Fault')
        r.o.ack = 'accepted'; r.run(5)
        self.assertFalse(r.s.allowed)

    def test_alignment_must_complete_before_underwater_command(self):
        r = Rig().run(5); r.auto = False
        r.o.intent = 'DIVE'; r.run(1)
        self.assertEqual(r.commands, [2])
        r.run(4); self.assertEqual(r.s.state, 'Fault')

    def test_position_jump_blocks_permission(self):
        r = Rig().run(5); r.o.intent = 'DIVE'; r.o.continuity_ok = False
        r.run(1); self.assertEqual(r.s.state, 'Fault')
        self.assertFalse(r.s.allowed)

    def test_randomized_health_and_observation_invariants(self):
        rng = random.Random(20261002)
        for active in (True, False):
            for _ in range(100):
                r = Rig(active)
                for _ in range(100):
                    r.o.gps_good = rng.random() > .2
                    r.o.healthy = rng.random() > .05
                    r.o.intent_fresh = rng.random() > .02
                    r.run(.1)
                    if not active or not r.o.healthy or not r.o.intent_fresh:
                        self.assertFalse(r.s.allowed)
                if not active:
                    self.assertEqual(r.commands, [])


if __name__ == '__main__':
    unittest.main()
