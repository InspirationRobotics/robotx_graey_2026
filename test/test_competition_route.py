import math
import unittest
from robotx_graey_2026.api.navigation.competition_route import CompetitionRouteRun
from robotx_graey_2026.api.navigation.route_plan import validate_route
from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame
from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo


def point(kind, north, east=0):
    lat, lon = ne_to_geo(north, east, (33., -117.))
    return dict(kind=kind, lat=lat, lon=lon)


class RouteTests(unittest.TestCase):
    def make(self):
        plan = dict(points=[point('waypoint', 2), point('pipeline_start', 4),
                            point('pipeline_buoy', 4, 2), point('pipeline_end', 9), point('waypoint', 12)],
                    pipeline_end_action='surface', recording_failure_action='finish_segment')
        run = CompetitionRouteRun(plan, 'test', 0)
        frame = GeographicFrame()
        frame.align(33., -117., (0., 0.), 0.)
        o = dict(fresh=True, armed=True, mode=4, local=(0., 0., -.3), yaw=0., speed=0., depth=0.,
                 bridge=dict(enabled=True, configuration_verified=True, map_transform=frame.snapshot(),
                             estimator_health=dict(usable=True)), recording=dict(ready=True))
        return run, o

    def test_pairing(self):
        with self.assertRaisesRegex(ValueError, 'missing'):
            validate_route(dict(points=[point('pipeline_start', 0)]))
        with self.assertRaisesRegex(ValueError, 'preceding'):
            validate_route(dict(points=[point('pipeline_end', 0)]))
        with self.assertRaisesRegex(ValueError, 'followed'):
            validate_route(dict(points=[point('pipeline_start', 0), point('waypoint', 4)]))
        with self.assertRaises(ValueError):
            validate_route(dict(points=[point('pipeline_buoy', 1)]))

    def test_full_route_without_client_or_rf_inputs(self):
        run, o = self.make()
        # Failed recorders must not prevent descent, traversal, ascent, or completion.
        o['recording'] = dict(ready=False, reason='Injected camera and sonar failure')
        stages = set()
        disarms = 0
        for i in range(1, 2500):
            actions = run.step(i*.1, o)
            stages.add(run.stage)
            for action, target in actions:
                if action in ('guided_hold', 'guided_target', 'guided_orient'):
                    old = o['local']
                    # Simple ideal plant, not a physical ArduSub validation.
                    o['local'] = tuple(v+max(-.04, min(.04, t-v)) for v, t in zip(old, target[:3]))
                    o['depth'] = o['local'][2]+.3
                    o['speed'] = math.dist(old[:2], o['local'][:2])/.1
                    if action == 'guided_orient':
                        o['yaw'] = target[3]
                if action == 'disarm':
                    self.assertEqual(run.point_index, len(run.points))
                    self.assertLess(o['depth'], .2)
                    o['armed'] = False
                    disarms += 1
            if run.stage in run.terminal:
                break
        self.assertEqual(run.stage, 'COMPLETE', run.reason)
        self.assertTrue({'PIPELINE_DIVE', 'PIPELINE', 'PIPELINE_SURFACE'}.issubset(stages))
        self.assertEqual(disarms, 1)
        self.assertFalse(run.recording_requested)

    def test_recording_failure_continues_and_pilot_still_overrides(self):
        run, o = self.make()
        run.stage = 'PIPELINE'
        run.surface_down = 3.7
        run.pipeline_hold = (4., 0.)
        run.pipeline_end = (9., 0.)
        run.pipeline_heading = 0.
        run.depth_target = 4.
        o.update(local=(5., 0., 3.7), depth=4., recording=dict(ready=False))
        actions = run.step(.1, o)
        self.assertEqual(run.stage, 'PIPELINE')
        self.assertAlmostEqual(actions[0][1][2], 3.7)
        self.assertEqual(actions[0][1][:2], (9., 0.))
        self.assertTrue(run.recording_warning)
        o['mode'] = 0
        self.assertEqual(run.step(.2, o), [])
        self.assertEqual(run.stage, 'ABORTED')
        self.assertTrue(run.pilot_takeover)

    def test_recording_does_not_block_but_pressure_defines_descent(self):
        run, o = self.make()
        run.stage = 'RECORD'; run.pipeline_hold = (0., 0.); run.surface_down = -.3
        run.pipeline_end = (8., 0.)
        o['depth'] = None
        o['recording'] = dict(ready=False, reason='Camera offline')
        self.assertEqual(run.step(.1, o)[0][1][2], -.3)
        self.assertEqual(run.stage, 'PIPELINE_ORIENT')
        o['depth'] = None
        self.assertEqual(run.step(.4, o)[0][1][2], -.3)
        self.assertEqual(run.stage, 'PIPELINE_ORIENT')

    def test_all_sensor_loss_aborts_but_gps_loss_does_not(self):
        run, o = self.make()
        run.stage = 'PIPELINE'; run.surface_down = 3.7
        run.pipeline_end = (9., 0.); run.pipeline_heading = 0.
        o.update(local=(4., 0., 3.7), depth=4.)
        o['bridge']['estimator_health']['sensors'] = dict(gps=False, dvl=True, vn=True)
        self.assertEqual(run.step(.1, o)[0][0], 'guided_orient')
        o['bridge']['estimator_health']['all_sensors_lost'] = True
        run.step(.2, o)
        self.assertEqual(run.stage, 'ABORTED')

    def test_pressure_dropout_uses_cube_prediction_and_keeps_travelling(self):
        run, o = self.make()
        run.stage = 'PIPELINE'; run.surface_down = 3.7
        run.pipeline_end = (9., 0.); run.pipeline_heading = 0.
        o.update(local=(4., 0., 3.7), depth=4.)
        run.step(.1, o)
        o['depth'] = None
        action, target = run.step(.3, o)[0]
        self.assertEqual(action, 'guided_orient')
        self.assertEqual(target[:2], (9., 0.))
        self.assertAlmostEqual(target[2], 3.7)
        self.assertIn('prediction', run.depth_source)
