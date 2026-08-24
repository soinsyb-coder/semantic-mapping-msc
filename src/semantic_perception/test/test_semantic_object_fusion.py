import unittest

from semantic_perception.semantic_object_fusion import (
    MapDetection,
    SemanticObjectFusion,
)


def detection(
    class_name='chair',
    confidence=0.8,
    x=1.0,
    y=2.0,
    z=0.5,
):
    """Create a concise map-frame detection for fusion tests."""
    return MapDetection(
        class_name=class_name,
        confidence=confidence,
        x=x,
        y=y,
        z=z,
    )


def fusion(**overrides):
    """Create a fusion instance with representative defaults."""
    parameters = {
        'association_radius': 0.6,
        'confirmation_hits': 3,
        'confirmation_window': 5,
        'max_missed_frames': 5,
    }
    parameters.update(overrides)
    return SemanticObjectFusion(**parameters)


class TestSemanticObjectFusion(unittest.TestCase):
    """Verify filtering-independent spatial fusion behaviour."""

    def test_repeated_detections_confirm_one_stable_object(self):
        object_fusion = fusion()

        self.assertEqual(
            object_fusion.update([detection(x=1.0)]),
            [],
        )
        self.assertEqual(
            object_fusion.update([detection(x=1.1)]),
            [],
        )
        confirmed = object_fusion.update([detection(x=1.2)])

        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].object_id, 0)
        self.assertEqual(confirmed[0].class_name, 'chair')
        self.assertEqual(confirmed[0].observation_count, 3)
        self.assertAlmostEqual(confirmed[0].x, 1.1)
        stats = object_fusion.last_update_stats
        self.assertEqual(stats.new_objects, 0)
        self.assertEqual(stats.matched_candidates, 1)
        self.assertEqual(stats.newly_confirmed, 1)

    def test_different_classes_do_not_merge(self):
        object_fusion = fusion(confirmation_hits=1)

        confirmed = object_fusion.update([
            detection(class_name='chair'),
            detection(class_name='couch'),
        ])

        self.assertEqual(len(confirmed), 2)
        self.assertEqual(
            {item.class_name for item in confirmed},
            {'chair', 'couch'},
        )
        self.assertEqual(
            {item.object_id for item in confirmed},
            {0, 1},
        )

    def test_spatially_separate_same_class_objects_do_not_merge(self):
        object_fusion = fusion(confirmation_hits=1)

        confirmed = object_fusion.update([
            detection(x=1.0),
            detection(x=2.0),
        ])

        self.assertEqual(len(confirmed), 2)
        self.assertNotEqual(
            confirmed[0].object_id,
            confirmed[1].object_id,
        )

    def test_unconfirmed_transient_detection_expires(self):
        object_fusion = fusion(max_missed_frames=2)
        object_fusion.update([detection()])

        object_fusion.update([])
        object_fusion.update([])
        object_fusion.update([])

        self.assertEqual(object_fusion.objects, [])
        self.assertEqual(object_fusion.candidate_count(), 0)
        self.assertEqual(
            object_fusion.last_update_stats.expired_candidates,
            1,
        )

    def test_confirmed_object_persists_without_new_detections(self):
        object_fusion = fusion(confirmation_hits=1)
        first = object_fusion.update([detection()])

        for _ in range(20):
            confirmed = object_fusion.update([])

        self.assertEqual(len(confirmed), 1)
        self.assertEqual(
            confirmed[0].object_id,
            first[0].object_id,
        )

    def test_association_stats_account_for_accepted_detections(self):
        object_fusion = fusion(confirmation_hits=1)
        object_fusion.update([detection(x=1.0)])

        object_fusion.update([
            detection(x=1.1),
            detection(x=3.0),
        ])

        stats = object_fusion.last_update_stats
        self.assertEqual(stats.new_objects, 1)
        self.assertEqual(stats.matched_confirmed, 1)
        self.assertEqual(stats.newly_confirmed, 1)
        self.assertEqual(
            stats.new_objects
            + stats.matched_candidates
            + stats.matched_confirmed,
            2,
        )

    def test_position_uses_confidence_weighted_average(self):
        object_fusion = fusion(confirmation_hits=1)
        object_fusion.update([
            detection(confidence=0.5, x=0.0),
        ])

        confirmed = object_fusion.update([
            detection(confidence=1.0, x=0.3),
        ])

        self.assertAlmostEqual(confirmed[0].x, 0.2)
        self.assertAlmostEqual(
            confirmed[0].mean_confidence,
            0.75,
        )

    def test_evidence_records_match_distance_before_position_update(self):
        object_fusion = fusion(confirmation_hits=1)
        object_fusion.update([detection(x=1.0)])

        object_fusion.update([detection(x=1.4)])

        evidence = object_fusion.last_association_evidence[0]
        self.assertEqual(evidence.action, 'matched_confirmed')
        self.assertEqual(evidence.object_id, 0)
        self.assertAlmostEqual(evidence.distance_m, 0.4)
        self.assertAlmostEqual(evidence.object_x, 1.0)

    def test_evidence_records_nearest_object_outside_radius(self):
        object_fusion = fusion(confirmation_hits=1)
        object_fusion.update([detection(x=1.0)])

        object_fusion.update([detection(x=1.7)])

        evidence = object_fusion.last_association_evidence[0]
        self.assertEqual(evidence.action, 'new_confirmed')
        self.assertEqual(evidence.object_id, 1)
        self.assertEqual(evidence.nearest_object_id, 0)
        self.assertAlmostEqual(evidence.nearest_distance_m, 0.7)

    def test_evidence_marks_candidate_confirmation_event(self):
        object_fusion = fusion(
            confirmation_hits=3,
            confirmation_window=10,
        )
        object_fusion.update([detection(x=1.0)])
        object_fusion.update([detection(x=1.1)])

        object_fusion.update([detection(x=1.2)])

        evidence = object_fusion.last_association_evidence[0]
        self.assertEqual(evidence.action, 'candidate_confirmed')
        self.assertEqual(evidence.object_id, 0)

    def test_evidence_exposes_same_frame_one_to_one_block(self):
        object_fusion = fusion(confirmation_hits=1)
        object_fusion.update([detection(x=1.0)])

        object_fusion.update([
            detection(confidence=0.9, x=1.0),
            detection(confidence=0.8, x=1.1),
        ])

        evidence = object_fusion.last_association_evidence[1]
        self.assertEqual(evidence.action, 'new_confirmed')
        self.assertEqual(evidence.object_id, 1)
        self.assertEqual(evidence.nearest_object_id, 0)
        self.assertTrue(evidence.nearest_matched_this_frame)


if __name__ == '__main__':
    unittest.main()
