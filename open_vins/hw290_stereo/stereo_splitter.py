#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class SideBySideSplitter(Node):
    def __init__(self):
        super().__init__('hw290_stereo_splitter')
        self.bridge = CvBridge()
        source_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(Image, '/image_raw', self.on_image, source_qos)
        self.left_pub = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/cam1/image_raw', 10)
        self.left_info = self.create_publisher(CameraInfo, '/cam0/camera_info', 10)
        self.right_info = self.create_publisher(CameraInfo, '/cam1/camera_info', 10)
        self.get_logger().info('Splitting /image_raw 3200x1200 into /cam0 and /cam1')
    def info(self, header, right=False):
        c = CameraInfo(); c.header = header; c.width, c.height = 320, 240
        if not right: c.k = [211.9862574,0.0,159.3680959,0.0,212.0303933,123.4780295,0.0,0.0,1.0]; c.d = [-0.4169720272,0.2386866981,0.0001763182,-0.0001649462,-0.0935433880]
        else: c.k = [213.3254585,0.0,161.1292157,0.0,213.3642968,122.0964593,0.0,0.0,1.0]; c.d = [-0.4181365632,0.2317307119,0.0000882348,-0.0001055983,-0.0819414622]
        c.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]; c.p = [c.k[0],0.0,c.k[2],0.0,0.0,c.k[4],c.k[5],0.0,0.0,0.0,1.0,0.0]
        return c
    def on_image(self, msg):
        try:
            # Keep the camera's RGB frame and only split the two views. OpenVINS
            # performs the grayscale conversion once in its image callback.
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if image.shape[1] < 640 or image.shape[0] < 240: return
            left, right = image[:, :320], image[:, 320:640]
            encoding = msg.encoding
            lmsg = self.bridge.cv2_to_imgmsg(left, encoding=encoding); rmsg = self.bridge.cv2_to_imgmsg(right, encoding=encoding)
            lmsg.header = msg.header; rmsg.header = msg.header; lmsg.header.frame_id = 'cam0'; rmsg.header.frame_id = 'cam1'
            self.left_pub.publish(lmsg); self.right_pub.publish(rmsg); self.left_info.publish(self.info(lmsg.header)); self.right_info.publish(self.info(rmsg.header, True))
        except Exception as exc: self.get_logger().error(f'image split failed: {exc}')
def main():
    rclpy.init(); node = SideBySideSplitter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
