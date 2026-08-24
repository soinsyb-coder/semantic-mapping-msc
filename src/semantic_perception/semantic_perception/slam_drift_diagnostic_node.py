import csv
import math
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from semantic_perception.slam_drift_metrics import (
    DriftMonitor,
    Pose2D,
)
from tf2_ros import Buffer, TransformException, TransformListener


class SlamDriftDiagnosticNode(Node):
    """Observe TF corrections that can invalidate semantic landmarks."""

    def __init__(self):
        super().__init__('slam_drift_diagnostic_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sample_rate_hz', 2.0)
        self.declare_parameter('report_interval_sec', 5.0)
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter(
            'translation_jump_threshold_m',
            0.15,
        )
        self.declare_parameter(
            'yaw_jump_threshold_deg',
            10.0,
        )
        self.declare_parameter('csv_path', '')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        sample_rate_hz = float(
            self.get_parameter('sample_rate_hz').value
        )
        self.report_interval_sec = float(
            self.get_parameter('report_interval_sec').value
        )
        self.tf_timeout_sec = float(
            self.get_parameter('tf_timeout_sec').value
        )
        translation_threshold = float(
            self.get_parameter(
                'translation_jump_threshold_m'
            ).value
        )
        yaw_threshold_deg = float(
            self.get_parameter('yaw_jump_threshold_deg').value
        )
        csv_path = str(self.get_parameter('csv_path').value)

        if sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be positive')
        if self.report_interval_sec <= 0.0:
            raise ValueError('report_interval_sec must be positive')
        if self.tf_timeout_sec < 0.0:
            raise ValueError('tf_timeout_sec must be non-negative')
        if translation_threshold <= 0.0:
            raise ValueError(
                'translation_jump_threshold_m must be positive'
            )
        if yaw_threshold_deg <= 0.0:
            raise ValueError('yaw_jump_threshold_deg must be positive')

        self.monitor = DriftMonitor(
            translation_jump_threshold_m=translation_threshold,
            yaw_jump_threshold_rad=math.radians(yaw_threshold_deg),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_report_time = time.monotonic()
        self.last_tf_warning_time = -float('inf')
        self.csv_file = None
        self.csv_writer = None
        if csv_path:
            self.open_csv(csv_path)

        self.timer = self.create_timer(
            1.0 / sample_rate_hz,
            self.sample_transforms,
        )
        self.get_logger().info(
            'SLAM drift diagnostic ready: '
            f'{self.map_frame}<-{self.odom_frame}, '
            f'base={self.base_frame}, rate={sample_rate_hz:.1f} Hz, '
            f'jump_threshold={translation_threshold:.2f} m/'
            f'{yaw_threshold_deg:.1f} deg'
        )

    def open_csv(self, csv_path):
        """Create a CSV file for offline drift analysis."""
        output_path = Path(csv_path).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = output_path.open(
            'w',
            newline='',
            encoding='utf-8',
        )
        field_names = [
            'ros_time_s',
            'map_odom_x',
            'map_odom_y',
            'map_odom_yaw_deg',
            'correction_step_m',
            'correction_step_deg',
            'map_base_step_m',
            'map_base_step_deg',
            'odom_base_step_m',
            'odom_base_step_deg',
            'correction_event',
        ]
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=field_names,
        )
        self.csv_writer.writeheader()
        self.csv_file.flush()
        self.get_logger().info(f'SLAM_DIAG csv={output_path}')

    @staticmethod
    def pose_from_transform(transform):
        """Convert a TransformStamped to planar position and yaw."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sin_yaw = 2.0 * (
            rotation.w * rotation.z + rotation.x * rotation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z
        )
        return Pose2D(
            x=float(translation.x),
            y=float(translation.y),
            yaw=math.atan2(sin_yaw, cos_yaw),
        )

    def lookup_pose(self, target_frame, source_frame):
        """Look up the latest planar transform between two frames."""
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        return self.pose_from_transform(transform)

    def sample_transforms(self):
        """Sample map correction, map pose and raw odometry pose."""
        try:
            map_odom = self.lookup_pose(
                self.map_frame,
                self.odom_frame,
            )
            map_base = self.lookup_pose(
                self.map_frame,
                self.base_frame,
            )
            odom_base = self.lookup_pose(
                self.odom_frame,
                self.base_frame,
            )
        except TransformException as error:
            now = time.monotonic()
            if now - self.last_tf_warning_time >= 2.0:
                self.get_logger().warning(
                    f'SLAM_DIAG tf_unavailable: {error}'
                )
                self.last_tf_warning_time = now
            return

        sample = self.monitor.update(
            map_odom,
            map_base,
            odom_base,
        )
        correction_deg = math.degrees(sample.correction_yaw_rad)
        map_deg = math.degrees(sample.map_yaw_rad)
        odom_deg = math.degrees(sample.odom_yaw_rad)
        cumulative_yaw_deg = math.degrees(
            self.monitor.cumulative_correction_yaw_rad
        )

        if self.csv_writer is not None:
            self.csv_writer.writerow({
                'ros_time_s': (
                    self.get_clock().now().nanoseconds * 1.0e-9
                ),
                'map_odom_x': map_odom.x,
                'map_odom_y': map_odom.y,
                'map_odom_yaw_deg': math.degrees(map_odom.yaw),
                'correction_step_m': (
                    sample.correction_translation_m
                ),
                'correction_step_deg': correction_deg,
                'map_base_step_m': sample.map_motion_m,
                'map_base_step_deg': map_deg,
                'odom_base_step_m': sample.odom_motion_m,
                'odom_base_step_deg': odom_deg,
                'correction_event': int(sample.correction_event),
            })
            self.csv_file.flush()

        if sample.correction_event:
            self.get_logger().warning(
                'SLAM_EVENT correction_jump: '
                f'step={sample.correction_translation_m:.3f} m/'
                f'{correction_deg:.2f} deg, '
                f'map_motion={sample.map_motion_m:.3f} m/'
                f'{map_deg:.2f} deg, '
                f'odom_motion={sample.odom_motion_m:.3f} m/'
                f'{odom_deg:.2f} deg'
            )

        now = time.monotonic()
        if now - self.last_report_time >= self.report_interval_sec:
            self.get_logger().info(
                'SLAM_DIAG summary: '
                f'samples={self.monitor.sample_count}, '
                f'events={self.monitor.correction_event_count}, '
                f'latest_correction='
                f'{sample.correction_translation_m:.3f} m/'
                f'{correction_deg:.2f} deg, '
                f'max_correction='
                f'{self.monitor.maximum_correction_translation_m:.3f} m/'
                f'{math.degrees(self.monitor.maximum_correction_yaw_rad):.2f} '
                'deg, '
                f'cumulative_correction='
                f'{self.monitor.cumulative_correction_translation_m:.3f} '
                'm/'
                f'{cumulative_yaw_deg:.2f} '
                'deg'
            )
            self.last_report_time = now

    def destroy_node(self):
        """Close the optional CSV before destroying the ROS node."""
        if self.csv_file is not None:
            self.csv_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SlamDriftDiagnosticNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'SLAM drift diagnostic failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
