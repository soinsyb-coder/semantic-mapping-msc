from collections import deque
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class MapDetection:
    """Represent one filtered object detection in the map frame."""

    class_name: str
    confidence: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class FusionUpdateStats:
    """Describe how one frame of accepted detections was associated."""

    new_objects: int = 0
    matched_candidates: int = 0
    matched_confirmed: int = 0
    newly_confirmed: int = 0
    expired_candidates: int = 0


@dataclass(frozen=True)
class AssociationEvidence:
    """Describe how one accepted detection was associated."""

    class_name: str
    confidence: float
    detection_x: float
    detection_y: float
    detection_z: float
    action: str
    object_id: int
    distance_m: float | None = None
    object_x: float | None = None
    object_y: float | None = None
    object_z: float | None = None
    nearest_object_id: int | None = None
    nearest_distance_m: float | None = None
    nearest_matched_this_frame: bool = False


@dataclass
class SemanticObject:
    """Store a spatially fused semantic object hypothesis."""

    object_id: int
    class_name: str
    x: float
    y: float
    z: float
    confidence_sum: float
    observation_count: int = 1
    missed_frames: int = 0
    confirmed: bool = False
    hit_history: deque = field(default_factory=deque, repr=False)

    @property
    def mean_confidence(self):
        """Return mean confidence over all associated observations."""
        return self.confidence_sum / self.observation_count

    def distance_to(self, detection):
        """Return Euclidean distance to a map-frame detection."""
        return math.sqrt(
            (self.x - detection.x) ** 2
            + (self.y - detection.y) ** 2
            + (self.z - detection.z) ** 2
        )

    def add_observation(self, detection):
        """Update position using confidence-weighted averaging."""
        previous_weight = self.confidence_sum
        new_weight = max(float(detection.confidence), 1.0e-6)
        total_weight = previous_weight + new_weight
        self.x = (
            self.x * previous_weight + detection.x * new_weight
        ) / total_weight
        self.y = (
            self.y * previous_weight + detection.y * new_weight
        ) / total_weight
        self.z = (
            self.z * previous_weight + detection.z * new_weight
        ) / total_weight
        self.confidence_sum = total_weight
        self.observation_count += 1
        self.missed_frames = 0


class SemanticObjectFusion:
    """Fuse repeated same-class detections into stable map objects."""

    def __init__(
        self,
        association_radius,
        confirmation_hits,
        confirmation_window,
        max_missed_frames,
    ):
        if association_radius <= 0.0:
            raise ValueError('association_radius must be positive')
        if confirmation_window < 1:
            raise ValueError('confirmation_window must be positive')
        if not 1 <= confirmation_hits <= confirmation_window:
            raise ValueError(
                'confirmation_hits must be in [1, confirmation_window]'
            )
        if max_missed_frames < 0:
            raise ValueError('max_missed_frames must be non-negative')

        self.association_radius = float(association_radius)
        self.confirmation_hits = int(confirmation_hits)
        self.confirmation_window = int(confirmation_window)
        self.max_missed_frames = int(max_missed_frames)
        self.objects = []
        self.next_object_id = 0
        self.last_update_stats = FusionUpdateStats()
        self.last_association_evidence = []

    def _new_object(self, detection):
        hit_history = deque(
            [True],
            maxlen=self.confirmation_window,
        )
        semantic_object = SemanticObject(
            object_id=self.next_object_id,
            class_name=detection.class_name,
            x=float(detection.x),
            y=float(detection.y),
            z=float(detection.z),
            confidence_sum=max(float(detection.confidence), 1.0e-6),
            confirmed=self.confirmation_hits == 1,
            hit_history=hit_history,
        )
        self.next_object_id += 1
        self.objects.append(semantic_object)
        return semantic_object

    def _nearest_unmatched_object(
        self,
        detection,
        matched_object_ids,
    ):
        candidates = []
        for semantic_object in self.objects:
            if semantic_object.object_id in matched_object_ids:
                continue
            if semantic_object.class_name != detection.class_name:
                continue
            distance = semantic_object.distance_to(detection)
            if distance <= self.association_radius:
                candidates.append((distance, semantic_object.object_id))

        if not candidates:
            return None

        _, object_id = min(candidates)
        return next(
            semantic_object
            for semantic_object in self.objects
            if semantic_object.object_id == object_id
        )

    def _nearest_same_class_object(
        self,
        detection,
        excluded_object_ids=frozenset(),
    ):
        """Return nearest non-excluded same-class object and distance."""
        candidates = []
        for semantic_object in self.objects:
            if semantic_object.object_id in excluded_object_ids:
                continue
            if semantic_object.class_name != detection.class_name:
                continue
            candidates.append(
                (
                    semantic_object.distance_to(detection),
                    semantic_object.object_id,
                    semantic_object,
                )
            )

        if not candidates:
            return None, None

        distance, _, semantic_object = min(
            candidates,
            key=lambda item: (item[0], item[1]),
        )
        return semantic_object, distance

    def update(self, detections):
        """Associate one frame of detections and return confirmed objects."""
        matched_object_ids = set()
        new_objects = 0
        matched_candidates = 0
        matched_confirmed = 0
        newly_confirmed = 0
        association_evidence = []

        for semantic_object in self.objects:
            semantic_object.hit_history.append(False)
            semantic_object.missed_frames += 1

        ordered_detections = sorted(
            detections,
            key=lambda item: item.confidence,
            reverse=True,
        )
        for detection in ordered_detections:
            nearest_object, nearest_distance = (
                self._nearest_same_class_object(
                    detection,
                )
            )
            semantic_object = self._nearest_unmatched_object(
                detection,
                matched_object_ids,
            )
            if semantic_object is None:
                semantic_object = self._new_object(detection)
                association_evidence.append(
                    AssociationEvidence(
                        class_name=detection.class_name,
                        confidence=float(detection.confidence),
                        detection_x=float(detection.x),
                        detection_y=float(detection.y),
                        detection_z=float(detection.z),
                        action='new_confirmed'
                        if semantic_object.confirmed
                        else 'new_candidate',
                        object_id=semantic_object.object_id,
                        nearest_object_id=(
                            None
                            if nearest_object is None
                            else nearest_object.object_id
                        ),
                        nearest_distance_m=nearest_distance,
                        nearest_matched_this_frame=(
                            nearest_object is not None
                            and nearest_object.object_id
                            in matched_object_ids
                        ),
                    )
                )
                new_objects += 1
                if semantic_object.confirmed:
                    newly_confirmed += 1
                matched_object_ids.add(semantic_object.object_id)
                continue

            was_confirmed = semantic_object.confirmed
            distance_m = semantic_object.distance_to(detection)
            object_x = semantic_object.x
            object_y = semantic_object.y
            object_z = semantic_object.z
            if was_confirmed:
                matched_confirmed += 1
            else:
                matched_candidates += 1
            semantic_object.add_observation(detection)
            semantic_object.hit_history[-1] = True
            if (
                not semantic_object.confirmed
                and sum(semantic_object.hit_history)
                >= self.confirmation_hits
            ):
                semantic_object.confirmed = True
                newly_confirmed += 1
            if not was_confirmed and semantic_object.confirmed:
                action = 'candidate_confirmed'
            elif was_confirmed:
                action = 'matched_confirmed'
            else:
                action = 'matched_candidate'
            association_evidence.append(
                AssociationEvidence(
                    class_name=detection.class_name,
                    confidence=float(detection.confidence),
                    detection_x=float(detection.x),
                    detection_y=float(detection.y),
                    detection_z=float(detection.z),
                    action=action,
                    object_id=semantic_object.object_id,
                    distance_m=distance_m,
                    object_x=object_x,
                    object_y=object_y,
                    object_z=object_z,
                    nearest_object_id=(
                        None
                        if nearest_object is None
                        else nearest_object.object_id
                    ),
                    nearest_distance_m=nearest_distance,
                    nearest_matched_this_frame=(
                        nearest_object is not None
                        and nearest_object.object_id
                        in matched_object_ids
                    ),
                )
            )
            matched_object_ids.add(semantic_object.object_id)

        expired_candidates = sum(
            not semantic_object.confirmed
            and semantic_object.missed_frames
            > self.max_missed_frames
            for semantic_object in self.objects
        )
        self.objects = [
            semantic_object
            for semantic_object in self.objects
            if (
                semantic_object.confirmed
                or semantic_object.missed_frames
                <= self.max_missed_frames
            )
        ]
        self.last_update_stats = FusionUpdateStats(
            new_objects=new_objects,
            matched_candidates=matched_candidates,
            matched_confirmed=matched_confirmed,
            newly_confirmed=newly_confirmed,
            expired_candidates=expired_candidates,
        )
        self.last_association_evidence = association_evidence
        return self.confirmed_objects()

    def confirmed_objects(self):
        """Return confirmed objects ordered by their stable IDs."""
        return sorted(
            (
                semantic_object
                for semantic_object in self.objects
                if semantic_object.confirmed
            ),
            key=lambda item: item.object_id,
        )

    def candidate_count(self):
        """Return the number of hypotheses awaiting confirmation."""
        return sum(
            not semantic_object.confirmed
            for semantic_object in self.objects
        )
