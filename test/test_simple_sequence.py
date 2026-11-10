import math
import unittest
from robotx_graey_2026.api.navigation.simple_sequence import SimpleSequenceRun

class SequenceTests(unittest.TestCase):
    def test_complete_without_gps_or_prepare(self):
        m=SimpleSequenceRun({}, 'test', 0)
        o=dict(fresh=True, local=(10.,20.,3.), yaw=math.pi/2, speed=0., mode=4, armed=True)
        m.step(.1,o); m.step(.2,o)
        self.assertEqual(m.stage,'FORWARD_1')
        expected=[('FORWARD_1',(10.,25.,3.)),('DESCEND',(10.,25.,5.)),('FORWARD_2',(10.,30.,5.)),('SURFACE',(10.,30.,3.))]
        t=.3
        for stage,target in expected:
            self.assertEqual(m.stage,stage)
            for a,b in zip(m.targets[stage],target):self.assertAlmostEqual(a,b)
            o['local']=target
            for _ in range(30):
                t+=.1;m.step(t,o)
                if m.stage!=stage:break
            self.assertNotEqual(m.stage,stage)
        m.step(t+.1,o)
        self.assertEqual(m.stage,'DISARM')
        self.assertEqual(m.step(t+2,o)[0][0],'disarm')
        o['armed']=False;m.step(t+3,o)
        self.assertEqual(m.stage,'COMPLETE')

    def test_takeover(self):
        m=SimpleSequenceRun({},'test',0)
        o=dict(fresh=True,local=(0.,0.,0.),yaw=0.,speed=0.,mode=4,armed=True)
        m.step(.1,o);m.step(.2,o);o['mode']=19
        self.assertEqual(m.step(.3,o),[])
        self.assertEqual(m.stage,'ABORTED')
