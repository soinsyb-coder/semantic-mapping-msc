from collections import Counter
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from semantic_perception.fusion_diagnostics import (
    ACCEPTED,
    format_outcomes,
    FusionGate,
)
from semantic_perception.semantic_object_fusion import (
    MapDetection,
    SemanticObjectFusion,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import torch
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray


class RgbdYoloLocalizerNode(Node):
    """Localize RGB-D detections and fuse them in the map frame."""

    def __init__(self):
        super().__init__('rgbd_yolo_localizer_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera_info',
        )
        self.declare_parameter(
            'annotated_topic',
            '/yolo/rgbd_annotated_image',
        )
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('torch_threads', 4)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.10)
        self.declare_parameter('depth_region_scale', 0.50)
        self.declare_parameter('min_valid_ratio', 0.20)
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('log_interval', 10)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter(
            'marker_topic',
            '/semantic_map/markers',
        )
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('marker_scale', 0.25)
        self.declare_parameter('fusion_min_confidence', 0.50)
        self.declare_parameter('fusion_min_valid_ratio', 0.50)
        self.declare_parameter(
            'fusion_allowed_classes',
            [
                'bed',
                'chair',
                'couch',
                'dining table',
                'microwave',
                'oven',
                'potted plant',
                'refrigerator',
                'sink',
                'toilet',
                'tv',
            ],
        )
        self.declare_parameter(
            'fusion_max_detection_distance_m',
            6.0,
        )
        self.declare_parameter('fusion_radius_m', 0.60)
        self.declare_parameter('fusion_confirmation_hits', 3)
        self.declare_parameter('fusion_confirmation_window', 5)
        self.declare_parameter('fusion_max_missed_frames', 5)

        model_path = Path(
            self.get_parameter('model_path').value
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'YOLO model not found: {model_path}'
            )

        self.image_topic = self.get_parameter(
            'image_topic'
        ).value
        self.depth_topic = self.get_parameter(
            'depth_topic'
        ).value
        self.camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).value
        self.annotated_topic = self.get_parameter(
            'annotated_topic'
        ).value
        self.device = self.get_parameter('device').value
        self.image_size = int(
            self.get_parameter('imgsz').value
        )
        self.confidence = float(
            self.get_parameter('confidence').value
        )
        torch_threads = int(
            self.get_parameter('torch_threads').value
        )
        sync_queue_size = int(
            self.get_parameter('sync_queue_size').value
        )
        sync_slop = float(
            self.get_parameter('sync_slop').value
        )
        self.depth_region_scale = float(
            self.get_parameter('depth_region_scale').value
        )
        self.min_valid_ratio = float(
            self.get_parameter('min_valid_ratio').value
        )
        self.min_depth_m = float(
            self.get_parameter('min_depth_m').value
        )
        self.max_depth_m = float(
            self.get_parameter('max_depth_m').value
        )
        self.log_interval = max(
            1,
            int(self.get_parameter('log_interval').value),
        )
        self.map_frame = str(
            self.get_parameter('map_frame').value
        )
        self.marker_topic = str(
            self.get_parameter('marker_topic').value
        )
        self.tf_timeout_sec = float(
            self.get_parameter('tf_timeout_sec').value
        )
        self.marker_scale = float(
            self.get_parameter('marker_scale').value
        )
        self.fusion_min_confidence = float(
            self.get_parameter('fusion_min_confidence').value
        )
        self.fusion_min_valid_ratio = float(
            self.get_parameter('fusion_min_valid_ratio').value
        )
        self.fusion_allowed_classes = {
            str(class_name)
            for class_name in self.get_parameter(
                'fusion_allowed_classes'
            ).value
        }
        self.fusion_max_detection_distance_m = float(
            self.get_parameter(
                'fusion_max_detection_distance_m'
            ).value
        )
        fusion_radius_m = float(
            self.get_parameter('fusion_radius_m').value
        )
        fusion_confirmation_hits = int(
            self.get_parameter('fusion_confirmation_hits').value
        )
        fusion_confirmation_window = int(
            self.get_parameter('fusion_confirmation_window').value
        )
        fusion_max_missed_frames = int(
            self.get_parameter('fusion_max_missed_frames').value
        )

        if not 0.0 < self.depth_region_scale <= 1.0:
            raise ValueError(
                'depth_region_scale must be in (0.0, 1.0]'
            )
        if not 0.0 <= self.min_valid_ratio <= 1.0:
            raise ValueError(
                'min_valid_ratio must be in [0.0, 1.0]'
            )
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError(
                'min_depth_m must be smaller than max_depth_m'
            )
        if not self.map_frame:
            raise ValueError('map_frame must not be empty')
        if self.tf_timeout_sec < 0.0:
            raise ValueError('tf_timeout_sec must be non-negative')
        if not 0.0 <= self.fusion_min_confidence <= 1.0:
            raise ValueError(
                'fusion_min_confidence must be in [0.0, 1.0]'
            )
        if not 0.0 <= self.fusion_min_valid_ratio <= 1.0:
            raise ValueError(
                'fusion_min_valid_ratio must be in [0.0, 1.0]'
            )
        if self.marker_scale <= 0.0:
            raise ValueError('marker_scale must be positive')
        if not self.fusion_allowed_classes:
            raise ValueError(
                'fusion_allowed_classes must not be empty'
            )
        if self.fusion_max_detection_distance_m <= 0.0:
            raise ValueError(
                'fusion_max_detection_distance_m must be positive'
            )

        self.semantic_fusion = SemanticObjectFusion(
            association_radius=fusion_radius_m,
            confirmation_hits=fusion_confirmation_hits,
            confirmation_window=fusion_confirmation_window,
            max_missed_frames=fusion_max_missed_frames,
        )
        self.fusion_gate = FusionGate(
            minimum_confidence=self.fusion_min_confidence,
            minimum_valid_ratio=self.fusion_min_valid_ratio,
            maximum_distance_m=(
                self.fusion_max_detection_distance_m
            ),
            allowed_classes=frozenset(self.fusion_allowed_classes),
        )

        torch.set_num_threads(max(1, torch_threads))
        self.get_logger().info(
            f'Loading YOLO model: {model_path.resolve()}'
        )
        self.model = YOLO(str(model_path))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()
        self.frames_processed = 0
        self.reported_shape_error = False
        self.reported_frame_error = False
        self.last_tf_warning_time = -float('inf')
        self.marker_map_cleared = False
        self.fusion_outcome_totals = Counter()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.image_subscriber = Subscriber(
            self,
            Image,
            self.image_topic,
            qos_profile=sensor_qos,
        )
        self.depth_subscriber = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=sensor_qos,
        )
        self.info_subscriber = Subscriber(
            self,
            CameraInfo,
            self.camera_info_topic,
            qos_profile=sensor_qos,
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [
                self.image_subscriber,
                self.depth_subscriber,
                self.info_subscriber,
            ],
            queue_size=max(2, sync_queue_size),
            slop=max(0.0, sync_slop),
        )
        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        self.image_publisher = self.create_publisher(
            Image,
            self.annotated_topic,
            sensor_qos,
        )

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            marker_qos,
        )

        self.get_logger().info(
            'RGB-D YOLO localizer ready: '
            f'rgb={self.image_topic}, '
            f'depth={self.depth_topic}, '
            f'camera_info={self.camera_info_topic}, '
            f'output={self.annotated_topic}, '
            f'map_frame={self.map_frame}, '
            f'markers={self.marker_topic}, '
            f'fusion_confidence={self.fusion_min_confidence:.2f}, '
            f'fusion_depth_ratio={self.fusion_min_valid_ratio:.2f}, '
            f'fusion_radius={fusion_radius_m:.2f} m, '
            f'confirmation={fusion_confirmation_hits}/'
            f'{fusion_confirmation_window}, '
            f'slop={sync_slop:.3f} s'
        )

    @staticmethod
    def stamp_seconds(message):
        """Convert a ROS message timestamp to floating-point seconds."""
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def depth_in_metres(self, message):
        """Convert a supported ROS depth image to metres."""
        depth = self.bridge.imgmsg_to_cv2(
            message,
            desired_encoding='passthrough',
        )
        depth = np.asarray(depth)

        if message.encoding == '32FC1':
            return depth.astype(np.float32, copy=False)
        if message.encoding in ('16UC1', 'mono16'):
            return depth.astype(np.float32) * 0.001

        raise ValueError(
            f'Unsupported depth encoding: {message.encoding}'
        )

    def sample_depth(self, depth_image, box):
        """Return robust median depth and valid-pixel ratio for a box."""
        image_height, image_width = depth_image.shape[:2]
        x1, y1, x2, y2 = [float(value) for value in box]
        centre_x = 0.5 * (x1 + x2)
        centre_y = 0.5 * (y1 + y2)
        half_width = 0.5 * (x2 - x1) * self.depth_region_scale
        half_height = 0.5 * (y2 - y1) * self.depth_region_scale

        roi_x1 = max(0, int(np.floor(centre_x - half_width)))
        roi_y1 = max(0, int(np.floor(centre_y - half_height)))
        roi_x2 = min(image_width, int(np.ceil(centre_x + half_width)))
        roi_y2 = min(image_height, int(np.ceil(centre_y + half_height)))

        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            return None, 0.0

        region = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
        valid_mask = (
            np.isfinite(region)
            & (region >= self.min_depth_m)
            & (region <= self.max_depth_m)
        )
        valid_ratio = float(np.count_nonzero(valid_mask)) / region.size

        if valid_ratio < self.min_valid_ratio:
            return None, valid_ratio

        return float(np.median(region[valid_mask])), valid_ratio

    @staticmethod
    def transform_xyz(x, y, z, transform):
        """Apply a stamped rigid transform to a Cartesian point."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        doubled_cross_x = 2.0 * (
            rotation.y * z - rotation.z * y
        )
        doubled_cross_y = 2.0 * (
            rotation.z * x - rotation.x * z
        )
        doubled_cross_z = 2.0 * (
            rotation.x * y - rotation.y * x
        )

        rotated_x = (
            x
            + rotation.w * doubled_cross_x
            + rotation.y * doubled_cross_z
            - rotation.z * doubled_cross_y
        )
        rotated_y = (
            y
            + rotation.w * doubled_cross_y
            + rotation.z * doubled_cross_x
            - rotation.x * doubled_cross_z
        )
        rotated_z = (
            z
            + rotation.w * doubled_cross_z
            + rotation.x * doubled_cross_y
            - rotation.y * doubled_cross_x
        )

        return (
            rotated_x + translation.x,
            rotated_y + translation.y,
            rotated_z + translation.z,
        )

    @staticmethod
    def marker_colour(class_name):
        """Return a repeatable RGB colour for a semantic class."""
        palette = (
            (0.20, 0.80, 1.00),
            (1.00, 0.45, 0.20),
            (0.35, 0.90, 0.35),
            (0.95, 0.35, 0.75),
            (0.90, 0.85, 0.20),
            (0.55, 0.40, 1.00),
        )
        colour_index = sum(class_name.encode('utf-8')) % len(palette)
        return palette[colour_index]

    def publish_markers(self, semantic_objects, stamp):
        """Publish confirmed objects as persistent sphere/text pairs."""
        marker_array = MarkerArray()

        if not self.marker_map_cleared:
            clear_marker = Marker()
            clear_marker.header.frame_id = self.map_frame
            clear_marker.header.stamp = stamp
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            self.marker_map_cleared = True

        for semantic_object in semantic_objects:
            marker_id = semantic_object.object_id
            class_name = semantic_object.class_name
            map_x = semantic_object.x
            map_y = semantic_object.y
            map_z = semantic_object.z
            red, green, blue = self.marker_colour(class_name)

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns = 'semantic_objects'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = map_x
            sphere.pose.position.y = map_y
            sphere.pose.position.z = map_z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.marker_scale
            sphere.scale.y = self.marker_scale
            sphere.scale.z = self.marker_scale
            sphere.color.r = red
            sphere.color.g = green
            sphere.color.b = blue
            sphere.color.a = 0.90
            marker_array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'semantic_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = map_x
            label.pose.position.y = map_y
            label.pose.position.z = (
                map_z + 1.5 * self.marker_scale
            )
            label.pose.orientation.w = 1.0
            label.scale.z = 0.8 * self.marker_scale
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = (
                f'{class_name} #{marker_id} '
                f'{semantic_object.mean_confidence:.2f} '
                f'({semantic_object.observation_count} hits)'
            )
            marker_array.markers.append(label)

        self.marker_publisher.publish(marker_array)

    def lookup_map_transform(self, image_message):
        """Look up map-from-camera TF at the image timestamp."""
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                image_message.header.frame_id,
                Time.from_msg(image_message.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as error:
            now = time.monotonic()
            if now - self.last_tf_warning_time >= 2.0:
                self.get_logger().warning(
                    'Map transform unavailable: '
                    f'{self.map_frame} <- '
                    f'{image_message.header.frame_id}: {error}'
                )
                self.last_tf_warning_time = now
            return None

    def synchronized_callback(
        self,
        image_message,
        depth_message,
        camera_info_message,
    ):
        """Run YOLO and project valid detections into camera coordinates."""
        try:
            image = self.bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding='bgr8',
            )
            depth_image = self.depth_in_metres(depth_message)

            if image.shape[:2] != depth_image.shape[:2]:
                if not self.reported_shape_error:
                    self.get_logger().error(
                        'RGB and depth sizes differ: '
                        f'rgb={image.shape[1]}x{image.shape[0]}, '
                        f'depth={depth_image.shape[1]}x'
                        f'{depth_image.shape[0]}'
                    )
                    self.reported_shape_error = True
                return

            if (
                image_message.header.frame_id
                != depth_message.header.frame_id
            ):
                if not self.reported_frame_error:
                    self.get_logger().error(
                        'RGB and depth frame IDs differ: '
                        f'rgb={image_message.header.frame_id}, '
                        f'depth={depth_message.header.frame_id}'
                    )
                    self.reported_frame_error = True
                return

            fx = float(camera_info_message.k[0])
            fy = float(camera_info_message.k[4])
            cx = float(camera_info_message.k[2])
            cy = float(camera_info_message.k[5])
            if fx <= 0.0 or fy <= 0.0:
                raise ValueError('Camera focal lengths must be positive')

            start_time = time.perf_counter()
            result = self.model.predict(
                source=image,
                device=self.device,
                imgsz=self.image_size,
                conf=self.confidence,
                verbose=False,
            )[0]
            annotated_image = result.plot()

            boxes = result.boxes.xyxy.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()
            localized = []
            mapped = []
            fusion_detections = []
            fusion_outcomes = Counter()

            map_transform = None
            if len(boxes) > 0:
                map_transform = self.lookup_map_transform(
                    image_message
                )

            for box, class_id, confidence in zip(
                boxes,
                class_ids,
                confidences,
            ):
                class_name = result.names[int(class_id)]
                depth_m, valid_ratio = self.sample_depth(
                    depth_image,
                    box,
                )
                if depth_m is None:
                    outcome = self.fusion_gate.classify(
                        class_name=class_name,
                        confidence=float(confidence),
                        valid_ratio=valid_ratio,
                        distance_m=float('inf'),
                        has_valid_depth=False,
                    )
                    fusion_outcomes[outcome] += 1
                    continue

                u = 0.5 * (float(box[0]) + float(box[2]))
                v = 0.5 * (float(box[1]) + float(box[3]))
                x = (u - cx) * depth_m / fx
                y = (v - cy) * depth_m / fy
                z = depth_m
                map_xyz = None
                if map_transform is not None:
                    map_xyz = self.transform_xyz(
                        x,
                        y,
                        z,
                        map_transform,
                    )
                    mapped.append(
                        (
                            class_name,
                            float(confidence),
                            *map_xyz,
                        )
                    )
                    outcome = self.fusion_gate.classify(
                        class_name=class_name,
                        confidence=float(confidence),
                        valid_ratio=valid_ratio,
                        distance_m=z,
                    )
                    fusion_outcomes[outcome] += 1
                    if outcome == ACCEPTED:
                        fusion_detections.append(
                            MapDetection(
                                class_name=class_name,
                                confidence=float(confidence),
                                x=map_xyz[0],
                                y=map_xyz[1],
                                z=map_xyz[2],
                            )
                        )
                else:
                    outcome = self.fusion_gate.classify(
                        class_name=class_name,
                        confidence=float(confidence),
                        valid_ratio=valid_ratio,
                        distance_m=z,
                        has_map_transform=False,
                    )
                    fusion_outcomes[outcome] += 1

                localized.append(
                    (
                        class_name,
                        float(confidence),
                        u,
                        v,
                        valid_ratio,
                        x,
                        y,
                        z,
                        map_xyz,
                    )
                )

                label = (
                    f'{class_name}: {z:.2f} m'
                )
                if map_xyz is not None:
                    label += (
                        f' map=({map_xyz[0]:.2f}, '
                        f'{map_xyz[1]:.2f}, {map_xyz[2]:.2f})'
                    )
                text_x = max(0, int(box[0]))
                text_y = min(
                    annotated_image.shape[0] - 8,
                    max(18, int(box[3]) + 18),
                )
                cv2.putText(
                    annotated_image,
                    label,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            semantic_objects = self.semantic_fusion.update(
                fusion_detections
            )
            for evidence in self.semantic_fusion.last_association_evidence:
                message = (
                    'FUSION_EVIDENCE '
                    f'frame={self.frames_processed + 1} '
                    f'class={evidence.class_name!r} '
                    f'confidence={evidence.confidence:.3f} '
                    f'observation=({evidence.detection_x:.3f},'
                    f'{evidence.detection_y:.3f},'
                    f'{evidence.detection_z:.3f}) '
                    f'action={evidence.action} '
                    f'object_id={evidence.object_id}'
                )
                if evidence.distance_m is not None:
                    message += (
                        f' association_distance_m='
                        f'{evidence.distance_m:.3f} '
                        f'object_before=({evidence.object_x:.3f},'
                        f'{evidence.object_y:.3f},'
                        f'{evidence.object_z:.3f})'
                    )
                else:
                    message += ' association_distance_m=none'
                if evidence.nearest_object_id is None:
                    message += ' nearest_same_class=none'
                else:
                    message += (
                        f' nearest_same_class_id='
                        f'{evidence.nearest_object_id} '
                        f'nearest_distance_m='
                        f'{evidence.nearest_distance_m:.3f} '
                        f'nearest_matched_this_frame='
                        f'{int(evidence.nearest_matched_this_frame)}'
                    )
                message += (
                    f' association_radius_m='
                    f'{self.semantic_fusion.association_radius:.3f}'
                )
                self.get_logger().info(message)
            self.fusion_outcome_totals.update(fusion_outcomes)
            self.publish_markers(
                semantic_objects,
                image_message.header.stamp,
            )

            output_message = self.bridge.cv2_to_imgmsg(
                annotated_image,
                encoding='bgr8',
            )
            output_message.header = image_message.header
            self.image_publisher.publish(output_message)

            self.frames_processed += 1
            if (
                self.frames_processed == 1
                or self.frames_processed % self.log_interval == 0
            ):
                class_counts = Counter(
                    item[0] for item in localized
                )
                summary = ', '.join(
                    f'{name}={count}'
                    for name, count in sorted(class_counts.items())
                )
                if not summary:
                    summary = 'none'

                stamps = [
                    self.stamp_seconds(image_message),
                    self.stamp_seconds(depth_message),
                    self.stamp_seconds(camera_info_message),
                ]
                sync_span_ms = (max(stamps) - min(stamps)) * 1000.0
                elapsed_ms = (
                    time.perf_counter() - start_time
                ) * 1000.0
                self.get_logger().info(
                    f'Frame {self.frames_processed}: '
                    f'localized={len(localized)} [{summary}], '
                    f'mapped={len(mapped)}, '
                    f'fusion_input={len(fusion_detections)}, '
                    f'candidates='
                    f'{self.semantic_fusion.candidate_count()}, '
                    f'confirmed={len(semantic_objects)}, '
                    f'sync_span={sync_span_ms:.1f} ms, '
                    f'end_to_end={elapsed_ms:.1f} ms'
                )
                self.get_logger().info(
                    'FUSION_GATE frame: '
                    f'raw={len(boxes)}, '
                    f'outcomes=[{format_outcomes(fusion_outcomes)}]'
                )
                self.get_logger().info(
                    'FUSION_GATE total: '
                    f'raw={sum(self.fusion_outcome_totals.values())}, '
                    f'outcomes=['
                    f'{format_outcomes(self.fusion_outcome_totals)}]'
                )
                association = self.semantic_fusion.last_update_stats
                self.get_logger().info(
                    'FUSION_ASSOC frame: '
                    f'accepted={len(fusion_detections)}, '
                    f'new_objects={association.new_objects}, '
                    f'matched_candidates='
                    f'{association.matched_candidates}, '
                    f'matched_confirmed='
                    f'{association.matched_confirmed}, '
                    f'newly_confirmed={association.newly_confirmed}, '
                    f'expired_candidates='
                    f'{association.expired_candidates}'
                )

                for item in localized:
                    (
                        class_name,
                        confidence,
                        u,
                        v,
                        valid_ratio,
                        x,
                        y,
                        z,
                        map_xyz,
                    ) = item
                    message = (
                        f'{class_name}: '
                        f'confidence={confidence:.2f}, '
                        f'bbox_center=({u:.1f}, {v:.1f}), '
                        f'valid_depth_ratio={valid_ratio:.2f}, '
                        f'depth_m={z:.3f}, '
                        f'camera_xyz=({x:.3f}, {y:.3f}, {z:.3f})'
                    )
                    if map_xyz is None:
                        message += ', map_xyz=unavailable'
                    else:
                        message += (
                            f', map_xyz=({map_xyz[0]:.3f}, '
                            f'{map_xyz[1]:.3f}, '
                            f'{map_xyz[2]:.3f})'
                        )
                    self.get_logger().info(message)

        except Exception as error:
            self.get_logger().error(
                'RGB-D YOLO processing failed: '
                f'{type(error).__name__}: {error}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None

    try:
        node = RgbdYoloLocalizerNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        node.get_logger().info(
            'Using two executor threads so TF can update during lookup'
        )
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'RGB-D YOLO localizer failed to start: {error}')
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
