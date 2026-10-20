import unittest
from robotx_graey_2026.api.navigation.cube_north import CubeNorth,COMPASS_PARAMS,YAW_PARAMS


class CubeNorthTests(unittest.TestCase):
    def test_cancel_after_valid_capture_preserves_north_through_restoration(self):
        for already_restoring in (False,True):
            n=CubeNorth();n.start(0,self.params());self.completed(n)
            for i in range(31):
                t=1+i*.1;n.observe(t,self.params(t,1),90,30,True)
            self.assertEqual(n.phase,'captured')
            if already_restoring:n.capture(4,self.params(4,1))
            changes=n.cancel(4,self.params(4,1))
            self.assertAlmostEqual(n.reference['offset_deg'],60)
            self.assertEqual(changes['COMPASS_LEARN'],0)
            n.observe(4.1,self.params(4.1),0,0,False)
            self.assertEqual(n.phase,'ready')

    def completed(self,n):
        n.observe(.5,self.params(.5,1,3),90,30,True)
        n.message(1,'CompassLearn: Finished')

    def params(self,now=0.,yaw=6,learn=0):
        values=dict(COMPASS_ENABLE=1,COMPASS_USE=1,COMPASS_DEV_ID=123,COMPASS_LEARN=learn)
        values.update({k:yaw for k in YAW_PARAMS})
        return {k:(now,v) for k,v in values.items()}

    def test_no_circular_externalnav_reference_or_completion_by_parameter_alone(self):
        n=CubeNorth();changes=n.start(0,self.params())
        self.assertEqual(changes['COMPASS_LEARN'],3)
        for k in YAW_PARAMS:self.assertEqual(changes[k],1)
        for i in range(41):
            t=i*.1;n.observe(t,self.params(t,yaw=1),90,30,True)
        with self.assertRaises(ValueError):n.capture(4,self.params(4,yaw=1))
        n.message(4,'CompassLearn: Finished')
        for i in range(41):
            t=4+i*.1;n.observe(t,self.params(t,yaw=6),90,30,True)
        with self.assertRaises(ValueError):n.capture(8,self.params(8,yaw=6))

    def test_paired_offset_capture_and_externalnav_restoration_ack(self):
        n=CubeNorth();n.start(0,self.params());self.completed(n)
        for i in range(31):
            t=1+i*.1;n.observe(t,self.params(t,yaw=1),359+i*.1,29+i*.1,True)
        self.assertEqual(n.phase,'captured')
        self.assertAlmostEqual(n.reference['offset_deg'],-30)
        reference,changes=n.capture(4,self.params(4,yaw=1))
        self.assertAlmostEqual(reference['offset_deg'],-30)
        self.assertEqual(n.phase,'restoring')
        n.observe(4.1,self.params(4.1,yaw=1),0,0,True)
        self.assertEqual(n.phase,'restoring')
        n.observe(4.2,self.params(4.2),0,0,False)
        self.assertEqual(n.phase,'ready')
        self.assertEqual(changes,{**{k:6 for k in YAW_PARAMS},'COMPASS_LEARN':0})

    def test_unidentified_disabled_stale_and_bad_compass_rejected(self):
        for key in ('COMPASS_ENABLE','COMPASS_USE','COMPASS_DEV_ID'):
            p=self.params();p[key]=(0,0)
            with self.assertRaises(ValueError):CubeNorth().start(0,p)
        with self.assertRaises(ValueError):CubeNorth().start(4,self.params())
        n=CubeNorth();n.start(0,self.params());self.completed(n)
        for i in range(31):
            t=1+i*.1;n.observe(t,self.params(t,1),90,30,False)
        with self.assertRaises(ValueError):n.capture(4,self.params(4,1))
        n.cancel();n.observe(4.1,self.params(4.1),0,0,False)
        self.assertEqual(n.phase,'cancelled');self.assertIsNone(n.reference)

    def test_unstable_offset_rejected(self):
        n=CubeNorth();n.start(0,self.params());self.completed(n)
        for i in range(31):
            t=1+i*.1;n.observe(t,self.params(t,1),90+(10 if i%2 else -10),30,True)
        with self.assertRaisesRegex(ValueError,'disagreement'):n.capture(4,self.params(4,1))

    def test_restart_recovery_restores_yaw_but_never_claims_north(self):
        n=CubeNorth();changes=n.cancel(1,self.params(1,1,3))
        self.assertEqual(changes['COMPASS_LEARN'],0)
        n.observe(2,self.params(2),0,0,False)
        self.assertEqual(n.phase,'cancelled');self.assertIsNone(n.reference)
        with self.assertRaises(ValueError):CubeNorth().cancel(2,self.params(2))

    def test_vn_loss_never_accepts_old_learning_completion(self):
        n=CubeNorth();n.start(0,self.params());self.completed(n)
        n.invalidate_vn()
        for i in range(31):
            t=1+i*.1;n.observe(t,self.params(t,1),90,30,True)
        with self.assertRaises(ValueError):n.capture(4,self.params(4,1))
