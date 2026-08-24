import math
import unittest

from semantic_perception.slam_drift_metrics import (
    DriftMonitor,
    Pose2D,
    shortest_angle,
)


class TestSlamDriftMetrics(unittest.TestCase):
    """Verify correction-jump metrics independently of ROS TF."""

    def setUp(self):
        self.monitor = DriftMonitor(
            translation_jump_threshold_m=0.15,
            yaw_jump_threshold_rad=math.radians(10.0),
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        self.monitor.update(origin, origin, origin)

    def test_small_correction_is_not_an_event(self):
        sample = self.monitor.update(
            Pose2D(0.05, 0.0, math.radians(2.0)),
            Pose2D(0.25, 0.0, math.radians(2.0)),
            Pose2D(0.20, 0.0, 0.0),
        )

        self.assertFalse(sample.correction_event)
        self.assertAlmostEqual(sample.correction_translation_m, 0.05)
        self.assertEqual(self.monitor.correction_event_count, 0)

    def test_translation_correction_crossing_threshold_is_event(self):
        sample = self.monitor.update(
            Pose2D(0.16, 0.0, 0.0),
            Pose2D(0.36, 0.0, 0.0),
            Pose2D(0.20, 0.0, 0.0),
        )

        self.assertTrue(sample.correction_event)
        self.assertEqual(self.monitor.correction_event_count, 1)

    def test_yaw_wrap_does_not_create_false_jump(self):
        near_positive_pi = Pose2D(0.0, 0.0, math.radians(179.0))
        near_negative_pi = Pose2D(0.0, 0.0, math.radians(-179.0))
        monitor = DriftMonitor(0.15, math.radians(10.0))
        monitor.update(near_positive_pi, near_positive_pi, near_positive_pi)

        sample = monitor.update(
            near_negative_pi,
            near_negative_pi,
            near_negative_pi,
        )

        self.assertAlmostEqual(
            math.degrees(sample.correction_yaw_rad),
            2.0,
        )
        self.assertFalse(sample.correction_event)

    def test_shortest_angle_wraps_to_pi_interval(self):
        self.assertAlmostEqual(
            math.degrees(shortest_angle(math.radians(358.0))),
            -2.0,
        )


if __name__ == '__main__':
    unittest.main()
