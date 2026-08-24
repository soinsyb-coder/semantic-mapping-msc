from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Represent a planar transform for drift diagnostics."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DriftSample:
    """Store inter-sample motion and correction magnitudes."""

    correction_translation_m: float
    correction_yaw_rad: float
    map_motion_m: float
    map_yaw_rad: float
    odom_motion_m: float
    odom_yaw_rad: float
    correction_event: bool


def shortest_angle(angle):
    """Wrap an angular difference to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_delta(current, previous):
    """Return planar translation and absolute wrapped yaw change."""
    translation = math.hypot(
        current.x - previous.x,
        current.y - previous.y,
    )
    yaw = abs(shortest_angle(current.yaw - previous.yaw))
    return translation, yaw


class DriftMonitor:
    """Accumulate map-to-odom corrections without changing SLAM."""

    def __init__(
        self,
        translation_jump_threshold_m,
        yaw_jump_threshold_rad,
    ):
        self.translation_jump_threshold_m = float(
            translation_jump_threshold_m
        )
        self.yaw_jump_threshold_rad = float(yaw_jump_threshold_rad)
        self.previous_map_odom = None
        self.previous_map_base = None
        self.previous_odom_base = None
        self.sample_count = 0
        self.correction_event_count = 0
        self.cumulative_correction_translation_m = 0.0
        self.cumulative_correction_yaw_rad = 0.0
        self.maximum_correction_translation_m = 0.0
        self.maximum_correction_yaw_rad = 0.0

    def update(self, map_odom, map_base, odom_base):
        """Add synchronous TF poses and return their inter-sample changes."""
        if self.previous_map_odom is None:
            correction_translation = 0.0
            correction_yaw = 0.0
            map_motion = 0.0
            map_yaw = 0.0
            odom_motion = 0.0
            odom_yaw = 0.0
        else:
            correction_translation, correction_yaw = pose_delta(
                map_odom,
                self.previous_map_odom,
            )
            map_motion, map_yaw = pose_delta(
                map_base,
                self.previous_map_base,
            )
            odom_motion, odom_yaw = pose_delta(
                odom_base,
                self.previous_odom_base,
            )

        correction_event = (
            correction_translation
            >= self.translation_jump_threshold_m
            or correction_yaw >= self.yaw_jump_threshold_rad
        )
        if correction_event:
            self.correction_event_count += 1

        self.sample_count += 1
        self.cumulative_correction_translation_m += (
            correction_translation
        )
        self.cumulative_correction_yaw_rad += correction_yaw
        self.maximum_correction_translation_m = max(
            self.maximum_correction_translation_m,
            correction_translation,
        )
        self.maximum_correction_yaw_rad = max(
            self.maximum_correction_yaw_rad,
            correction_yaw,
        )
        self.previous_map_odom = map_odom
        self.previous_map_base = map_base
        self.previous_odom_base = odom_base

        return DriftSample(
            correction_translation_m=correction_translation,
            correction_yaw_rad=correction_yaw,
            map_motion_m=map_motion,
            map_yaw_rad=map_yaw,
            odom_motion_m=odom_motion,
            odom_yaw_rad=odom_yaw,
            correction_event=correction_event,
        )
