import unittest
from robotx_graey_2026.api.gui.communications_view import CommunicationsView


class TestCommunicationsView(unittest.TestCase):
    def test_unknown_before_first_sample(self):
        self.assertEqual(CommunicationsView().snapshot()['connection'],'unknown')

    def test_stale_snapshot_does_not_claim_live_connection_or_verified_run(self):
        now=[0.]
        view=CommunicationsView(clock=lambda:now[0])
        view.update(dict(wifi=dict(connected=True),run_confirmed=True,published_monotonic=0.))
        self.assertTrue(view.snapshot()['run_verified'])
        now[0]=2.
        self.assertEqual(view.snapshot()['connection'],'unknown')
        self.assertFalse(view.snapshot()['run_verified'])

    def test_snapshot_cannot_mutate_cache(self):
        view=CommunicationsView(clock=lambda:0.)
        view.update(dict(wifi=dict(connected=True),published_monotonic=0.))
        view.snapshot()['data']['wifi']['connected']=False
        self.assertEqual(view.snapshot()['connection'],'connected')
