import unittest

from semantic_perception.fusion_diagnostics import (
    ACCEPTED,
    CLASS_NOT_ALLOWED,
    FusionGate,
    LOW_CONFIDENCE,
    LOW_DEPTH_RATIO,
    NO_MAP_TF,
    NO_VALID_DEPTH,
    TOO_FAR,
)


class TestFusionGate(unittest.TestCase):
    """Verify exclusive, explainable fusion-gate outcomes."""

    def setUp(self):
        self.gate = FusionGate(
            minimum_confidence=0.5,
            minimum_valid_ratio=0.5,
            maximum_distance_m=6.0,
            allowed_classes=frozenset({'chair', 'bed'}),
        )

    def classify(self, **overrides):
        """Classify a representative detection with optional changes."""
        values = {
            'class_name': 'chair',
            'confidence': 0.8,
            'valid_ratio': 0.8,
            'distance_m': 2.0,
        }
        values.update(overrides)
        return self.gate.classify(**values)

    def test_accepts_detection_that_passes_all_gates(self):
        self.assertEqual(self.classify(), ACCEPTED)

    def test_reports_missing_depth_before_other_reasons(self):
        self.assertEqual(
            self.classify(
                class_name='person',
                has_valid_depth=False,
            ),
            NO_VALID_DEPTH,
        )

    def test_reports_missing_map_transform(self):
        self.assertEqual(
            self.classify(has_map_transform=False),
            NO_MAP_TF,
        )

    def test_reports_disallowed_dynamic_class(self):
        self.assertEqual(
            self.classify(class_name='person'),
            CLASS_NOT_ALLOWED,
        )

    def test_reports_low_confidence(self):
        self.assertEqual(
            self.classify(confidence=0.49),
            LOW_CONFIDENCE,
        )

    def test_reports_low_depth_ratio(self):
        self.assertEqual(
            self.classify(valid_ratio=0.49),
            LOW_DEPTH_RATIO,
        )

    def test_reports_excessive_distance(self):
        self.assertEqual(
            self.classify(distance_m=6.01),
            TOO_FAR,
        )


if __name__ == '__main__':
    unittest.main()
