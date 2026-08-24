from dataclasses import dataclass


ACCEPTED = 'accepted'
NO_VALID_DEPTH = 'no_valid_depth'
NO_MAP_TF = 'no_map_tf'
CLASS_NOT_ALLOWED = 'class_not_allowed'
LOW_CONFIDENCE = 'low_confidence'
LOW_DEPTH_RATIO = 'low_depth_ratio'
TOO_FAR = 'too_far'

OUTCOME_ORDER = (
    ACCEPTED,
    NO_VALID_DEPTH,
    NO_MAP_TF,
    CLASS_NOT_ALLOWED,
    LOW_CONFIDENCE,
    LOW_DEPTH_RATIO,
    TOO_FAR,
)


@dataclass(frozen=True)
class FusionGate:
    """Classify each raw detection by one exclusive fusion outcome."""

    minimum_confidence: float
    minimum_valid_ratio: float
    maximum_distance_m: float
    allowed_classes: frozenset

    def classify(
        self,
        class_name,
        confidence,
        valid_ratio,
        distance_m,
        has_valid_depth=True,
        has_map_transform=True,
    ):
        """Return the first applicable outcome for one detection."""
        if not has_valid_depth:
            return NO_VALID_DEPTH
        if not has_map_transform:
            return NO_MAP_TF
        if class_name not in self.allowed_classes:
            return CLASS_NOT_ALLOWED
        if confidence < self.minimum_confidence:
            return LOW_CONFIDENCE
        if valid_ratio < self.minimum_valid_ratio:
            return LOW_DEPTH_RATIO
        if distance_m > self.maximum_distance_m:
            return TOO_FAR
        return ACCEPTED


def format_outcomes(counter):
    """Format known non-zero outcomes in a stable order."""
    parts = [
        f'{name}={counter[name]}'
        for name in OUTCOME_ORDER
        if counter[name]
    ]
    return ', '.join(parts) if parts else 'none'
