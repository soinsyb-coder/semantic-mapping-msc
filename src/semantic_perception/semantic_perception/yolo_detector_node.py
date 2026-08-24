from collections import Counter
from pathlib import Path
import time

from cv_bridge import CvBridge
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter(
            'annotated_topic',
            '/yolo/annotated_image',
        )
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('torch_threads', 4)

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

        torch.set_num_threads(max(1, torch_threads))

        self.get_logger().info(
            f'Loading YOLO model: {model_path.resolve()}'
        )
        self.model = YOLO(str(model_path))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()
        self.frames_processed = 0

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos,
        )

        self.image_publisher = self.create_publisher(
            Image,
            self.annotated_topic,
            sensor_qos,
        )

        self.get_logger().info(
            f'YOLO detector ready: '
            f'input={self.image_topic}, '
            f'output={self.annotated_topic}, '
            f'imgsz={self.image_size}, '
            f'confidence={self.confidence}, '
            f'device={self.device}, '
            f'torch_threads={torch.get_num_threads()}'
        )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )

            start_time = time.perf_counter()

            result = self.model.predict(
                source=image,
                device=self.device,
                imgsz=self.image_size,
                conf=self.confidence,
                verbose=False,
            )[0]

            annotated_image = result.plot()

            output_message = self.bridge.cv2_to_imgmsg(
                annotated_image,
                encoding='bgr8',
            )
            output_message.header = message.header
            self.image_publisher.publish(output_message)

            self.frames_processed += 1
            elapsed_ms = (
                time.perf_counter() - start_time
            ) * 1000.0

            if (
                self.frames_processed == 1
                or self.frames_processed % 10 == 0
            ):
                class_ids = result.boxes.cls.cpu().tolist()
                class_counts = Counter(
                    result.names[int(class_id)]
                    for class_id in class_ids
                )

                detections = ', '.join(
                    f'{name}={count}'
                    for name, count in sorted(class_counts.items())
                )

                if not detections:
                    detections = 'none'

                self.get_logger().info(
                    f'Frame {self.frames_processed}: '
                    f'detections={len(result.boxes)} '
                    f'[{detections}], '
                    f'end_to_end={elapsed_ms:.1f} ms, '
                    f'inference={result.speed["inference"]:.1f} ms'
                )

        except Exception as error:
            self.get_logger().error(
                f'YOLO processing failed: '
                f'{type(error).__name__}: {error}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'YOLO detector failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
