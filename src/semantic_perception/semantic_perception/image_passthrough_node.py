import cv2
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


class ImagePassthroughNode(Node):

    def __init__(self):
        super().__init__('image_passthrough_node')

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()
        self.frames_received = 0

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            sensor_qos,
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/yolo/annotated_image',
            sensor_qos,
        )

        self.get_logger().info(
            'Waiting for images on /camera/image'
        )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )

            cv2.rectangle(
                image,
                (8, 8),
                (385, 44),
                (0, 0, 0),
                thickness=-1,
            )

            cv2.putText(
                image,
                'semantic_perception: passthrough',
                (16, 33),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            output_message = self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8',
            )
            output_message.header = message.header

            self.image_publisher.publish(output_message)

            self.frames_received += 1

            if self.frames_received == 1 or self.frames_received % 20 == 0:
                self.get_logger().info(
                    f'Published {self.frames_received} annotated frames '
                    f'from frame {message.header.frame_id}'
                )

        except Exception as error:
            self.get_logger().error(
                f'Image conversion or publication failed: {error}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImagePassthroughNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
