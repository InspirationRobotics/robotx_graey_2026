"""Submarine pack/unpack and producer behavior; all observations below are fixtures."""
import unittest
from pathlib import Path
import tempfile
from unittest.mock import patch
from robotx_graey_2026.api.comms import sub_link as sl


def heartbeat(task='TASK_INFRA_SURVEY_REPAIR'):
    return dict(position=dict(latitude=32.9234567, longitude=-117.0386123),
                state='STATE_AUTO', current_task=task, spd_mps=.4, heading_deg=123.,
                altitude_hae_m=-25., depth_m=1.5, roll_deg=2., pitch_deg=-3.)


class SubCodecTests(unittest.TestCase):
    ids = dict(run_id=7, session_id=15, sequence=3)

    def test_sub_status_is_uuv_and_has_task_start_and_depth(self):
        raw = sl.pack_sub(heartbeat(), **self.ids)
        self.assertEqual(len(raw), 47)
        packet = sl.unpack_sub(raw)
        hb = packet['heartbeat']
        self.assertEqual(hb['current_task'], 'TASK_INFRA_SURVEY_REPAIR')
        self.assertEqual(hb['vehicle_type'], 'TYPE_UUV')
        self.assertEqual(hb['depth_m'], 1.5)
        self.assertEqual(hb['position'], heartbeat()['position'])
        self.assertEqual(hb['flight_phase'], 'FLIGHT_PHASE_UNKNOWN')

    def test_missing_attitude_is_not_invented(self):
        hb = heartbeat()
        hb.pop('roll_deg'); hb.pop('pitch_deg')
        back = sl.unpack_sub(sl.pack_sub(hb, **self.ids))['heartbeat']
        self.assertNotIn('roll_deg', back)
        self.assertNotIn('pitch_deg', back)

    def test_original_segment_order_survives_and_is_independent_of_repair(self):
        segments = ['PIPELINE_SEGMENT_INTACT', 'PIPELINE_SEGMENT_DAMAGED', 'PIPELINE_SEGMENT_INTACT']
        raw = sl.pack_pipeline_survey(heartbeat()['position'], segments, **self.ids)
        back = sl.unpack_pipeline_survey(raw)['pipeline_survey']
        self.assertEqual(back['segments'], segments)
        self.assertEqual(back['active_buoy_position'], heartbeat()['position'])

    def test_disruptive_colors_keep_resource_then_circle_order(self):
        raw = sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED', **self.ids)
        self.assertEqual(len(raw), 15)
        request = sl.unpack_resource_request(raw)['resource_delivery']
        self.assertEqual(request, dict(task='TASK_INFRA_SURVEY_REPAIR',
                                       resource_color='COLOR_BLUE', delivery_circle_color='COLOR_RED'))

    def test_advanced_uses_any_tin_but_specific_circle(self):
        raw = sl.pack_resource_request('COLOR_ANY', 'COLOR_GREEN', **self.ids)
        self.assertEqual(sl.unpack_resource_request(raw)['resource_delivery']['resource_color'], 'COLOR_ANY')
        with self.assertRaises(ValueError):
            sl.pack_resource_request('COLOR_RED', 'COLOR_ANY', **self.ids)

    def test_empty_producer_sends_no_fake_reports(self):
        sender = sl.Sender(session_id=15)
        self.assertEqual(sender.due(0), [])
        self.assertEqual(sender.due(100), [])
        with self.assertRaises(ValueError):
            sender.resource_observed('COLOR_BLUE', 'COLOR_RED')

    def test_status_stops_when_producer_goes_stale(self):
        sender = sl.Sender(run_id=7, session_id=15)
        sender.update_status(heartbeat(), 10)
        first = sender.due(10)
        self.assertEqual([p for p, _ in first], [sl.PAYLOAD_SUB])
        self.assertEqual(sender.due(10.1), [])
        first_again = sender.due(10.5)
        self.assertEqual(len(first_again), 1)
        self.assertNotEqual(sl.unpack_sub(first_again[0][1])['sequence'],
                            sl.unpack_sub(first[0][1])['sequence'])
        self.assertEqual(sender.due(11.1), [])

    def test_real_events_repeat_identically_and_new_run_clears_them(self):
        sender = sl.Sender(run_id=7, session_id=15)
        sender.resource_observed('COLOR_BLUE', 'COLOR_RED')
        first = sender.due(10)
        self.assertEqual(sender.due(10.1), [])
        self.assertEqual(sender.due(13), first)
        sender.start_run(8)
        self.assertEqual(sender.due(14), [])

    def test_receiver_receipt_does_not_claim_delivery_begun(self):
        receiver = sl.Receiver()
        raw = sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED', **self.ids)
        packet = receiver.hear(sl.PAYLOAD_RESOURCE_REQUEST, raw)
        self.assertEqual(sl.ocs_envelope(packet)['report']['vehicle_id'], 'UUV1')
        echo = receiver.delivery_begun(**self.ids)
        self.assertEqual(echo['report']['vehicle_id'], 'UAV1')
        self.assertEqual(echo['report']['resource_delivery'], packet['resource_delivery'])
        with self.assertRaises(ValueError):
            receiver.delivery_begun(run_id=7, session_id=15, sequence=99)

    def test_malformed_frames_and_unknown_values_are_rejected(self):
        raw = sl.pack_resource_request('COLOR_RED', 'COLOR_BLUE', **self.ids)
        for bad in (raw[:-1], raw+b'\0', b'\x02'+raw[1:], raw[:-1]+b'\x09'):
            with self.subTest(raw=bad), self.assertRaises(ValueError):
                sl.unpack_resource_request(bad)
        with self.assertRaises(ValueError):
            sl.pack_pipeline_survey(heartbeat()['position'], [], **self.ids)
        with self.assertRaises(ValueError):
            sl.pack_sub(dict(heartbeat(), heading_deg=float('nan')), **self.ids)
        self.assertIsNone(sl.decode(32768, b'old boat bytes'))

    def test_submerged_collects_products_and_surfaces_with_latest_status_only(self):
        sender = sl.Sender(run_id=7, session_id=15)
        sender.set_submerged(True)
        sender.update_status(heartbeat(), 10)
        sender.survey_complete(heartbeat()['position'], ['PIPELINE_SEGMENT_DAMAGED'])
        sender.resource_observed('COLOR_BLUE', 'COLOR_RED')
        self.assertEqual(sender.due(10), [])
        self.assertEqual(sender.due(500), [])
        sender.set_submerged(False)
        due = sender.due(500)
        self.assertEqual([ptype for ptype, _ in due], [sl.PAYLOAD_PIPELINE_SURVEY, sl.PAYLOAD_RESOURCE_REQUEST])
        sender.update_status(heartbeat('TASK_NONE'), 500.1)
        due = sender.due(500.1)
        self.assertEqual([ptype for ptype, _ in due], [sl.PAYLOAD_SUB])
        self.assertEqual(sl.unpack_sub(due[0][1])['heartbeat']['current_task'], 'TASK_NONE')

    def test_outbox_restores_mission_products_and_phase_but_not_heartbeat(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'outbox.json'
            sender = sl.Sender(run_id=7, session_id=15, outbox_path=path)
            sender.set_submerged(True)
            sender.update_status(heartbeat(), 10)
            sender.survey_complete(heartbeat()['position'], ['PIPELINE_SEGMENT_DAMAGED'])
            sender.resource_observed('COLOR_BLUE', 'COLOR_RED')
            expected = dict(sender._packets)
            restored = sl.Sender(session_id=99, outbox_path=path)
            self.assertEqual(restored.run_id, 7)
            self.assertTrue(restored.submerged)
            self.assertEqual(restored.due(500), [])
            restored.set_submerged(False)
            due = restored.due(500)
            self.assertEqual(dict(due), {k: v for k, v in expected.items() if k != sl.PAYLOAD_SUB})
            self.assertTrue(all(sl.decode(ptype, raw)['session_id'] == 15 for ptype, raw in due))

    def test_explicit_new_run_clears_persisted_previous_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'outbox.json'
            sender = sl.Sender(run_id=7, session_id=15, outbox_path=path)
            sender.resource_observed('COLOR_BLUE', 'COLOR_RED')
            restored = sl.Sender(run_id=8, session_id=99, outbox_path=path)
            self.assertEqual(restored.due(0), [])
            self.assertEqual(sl.Sender(outbox_path=path).run_id, 8)

    def test_failed_outbox_write_does_not_claim_a_queued_product(self):
        sender = sl.Sender(run_id=7, session_id=15)
        with patch.object(sender, '_save', side_effect=OSError('fixture full disk')):
            with self.assertRaises(OSError):
                sender.resource_observed('COLOR_BLUE', 'COLOR_RED')
        self.assertEqual(sender.due(0), [])

    def test_corrupt_outbox_is_visible_instead_of_silently_discarded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'outbox.json'
            path.write_text('{broken', encoding='utf-8')
            with self.assertRaises(ValueError):
                sl.Sender(outbox_path=path)


if __name__ == '__main__':
    unittest.main()
