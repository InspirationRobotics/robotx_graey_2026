import math
import unittest
from robotx_graey_2026.api.navigation.course_heading import CourseHeading


class CourseHeadingTests(unittest.TestCase):
    def test_segment_stop_preserves_long_baseline_but_turning_discards_it(self):
        a=CourseHeading();a.preserve_stationary_anchor=True
        a.anchor=(0.,(33.,-117.),(0.,0.),0.,1.2)
        anchor=a.anchor
        self.assertEqual(a.observe(2.,self.fix(),self.sample(),(0.,0.,0.)),0.)
        self.assertEqual(a.anchor,anchor)
        a.observe(3.,self.fix(),self.sample(rates=(0.,0.,.2)),(0.,0.,0.))
        self.assertIsNone(a.anchor)

    def sample(self,velocity=(.2,0.,0.),rates=(0.,0.,0.)):
        return dict(raw_velocity=velocity,rates=rates)

    def fix(self,**changes):
        return dict(dict(speed_mps=.2,course_deg=10.,speed_accuracy_mps=.02,
                         accuracy=1.,lat=32.,lon=-117.),**changes)

    def test_slow_precise_velocity_corrects_only_after_sustained_motion(self):
        a=CourseHeading();steps=[]
        for i in range(41):steps.append(a.observe(i*.1,self.fix(),self.sample(),(.2,0.,0.)))
        self.assertEqual(sum(steps[:30]),0.)
        self.assertEqual(sum(steps),.5)
        self.assertEqual(a.updates,1)

    def test_lateral_motion_is_not_mistaken_for_nose_heading(self):
        for world,course in (((0.,.2,0.),90.),((-.2,0.,0.),180.)):
            a=CourseHeading()
            for i in range(41):step=a.observe(i*.1,self.fix(course_deg=course),self.sample(world),world)
            self.assertEqual(a.last_error,0.)

    def test_uncertain_stationary_turning_and_large_disagreement_never_correct(self):
        cases=[(self.fix(speed_accuracy_mps=.4),self.sample(),(.2,0.,0.)),
               (self.fix(speed_mps=.01),self.sample(),(.01,0.,0.)),
               (self.fix(),self.sample(rates=(0.,0.,.2)),(.2,0.,0.)),
               (self.fix(course_deg=120.),self.sample(),(.2,0.,0.))]
        for fix,sample,world in cases:
            a=CourseHeading()
            self.assertEqual(sum(a.observe(i*.1,fix,sample,world) for i in range(41)),0.)
            self.assertEqual(a.updates,0)

    def test_bounded_step_interval_and_wrapped_course(self):
        a=CourseHeading()
        for i in range(100):a.observe(i*.1,self.fix(course_deg=359.),self.sample(),(.2,0.,0.))
        self.assertAlmostEqual(a.last_error,-1.)
        self.assertLessEqual(a.updates,2)

    def test_rejection_or_dropout_does_not_reuse_old_window(self):
        a=CourseHeading()
        for i in range(29):a.observe(i*.1,self.fix(),self.sample(),(.2,0.,0.))
        a.reset_window('GPS rejected')
        self.assertEqual(a.observe(3,self.fix(),self.sample(),(.2,0.,0.)),0.)
        self.assertEqual(len(a.velocity_rows),1)
        a.motion(3,(.2,0,0),(0,0,0),False)
        self.assertEqual(len(a.velocity_rows),0)

    def test_long_position_baseline_supports_uncertain_doppler(self):
        a=CourseHeading();step=0.
        for i in range(101):
            t=i*.2;distance=.2*t
            fix=self.fix(speed_accuracy_mps=.5,accuracy=.2,lat=32.+distance/111320.)
            sample=self.sample();sample.update(course_motion_ne=(distance*math.cos(math.radians(10)),
                distance*math.sin(math.radians(10))),course_distance_m=distance)
            step+=a.observe(t,fix,sample,(.2,0,0))
        self.assertLess(step,0)
        self.assertIn('position baseline',a.method)

    def test_short_noisy_position_baseline_cannot_produce_heading(self):
        a=CourseHeading()
        for i in range(81):
            t=i*.1;distance=.2*t;sample=self.sample()
            sample.update(course_motion_ne=(distance,0),course_distance_m=distance)
            a.observe(t,self.fix(speed_accuracy_mps=.5,accuracy=2.,lat=32.+distance/111320.),sample,(.2,0,0))
        self.assertEqual(a.updates,0)

    def test_repeated_slow_motion_corrections_reduce_error_without_chasing_a_jump(self):
        a=CourseHeading();bias=10.
        for i in range(1201):
            t=i*.1
            if 30<=t<34:
                a.reset_window('GPS jumped and was rejected')
                continue
            world=(.2*math.cos(math.radians(bias)),.2*math.sin(math.radians(bias)),0.)
            bias+=a.observe(t,self.fix(course_deg=0.,speed_accuracy_mps=.01),self.sample(),world)
        self.assertLess(abs(bias),2.)
        self.assertGreater(bias,-.1)
