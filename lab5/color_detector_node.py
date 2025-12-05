#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        self.last_result = "Unknown"
        self.last_raw = "Unknown"      # 当前帧结果
        self.stable_result = "Unknown" # 稳定判定结果
        self.stable_counter = 0        # 连续帧数计数
        self.stable_threshold = 10     # 连续帧 ÷ 30fps = ~0.33秒


        super().__init__('color_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # adjust this topic name if needed
            self.image_callback,
            1)
        self.publisher_ = self.create_publisher(String, '/answer_result', 10)
        self.get_logger().info('Color Detector Node started with auto calibration.')


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=10)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Central ROI reduce background noise
        h, w, _ = hsv.shape
        # V1 30-70
        # roi = hsv[int(h*0.3):int(h*0.7),int(w*0.3):int(w*0.7)]
        # V2 10-90
        # roi = hsv[int(h*0.1):int(h*0.9), int(w*0.1):int(w*0.9)]
        # V3 100%
        roi = hsv[0:h, 0:w]


        # Define HSV thresholds
        lower_red1 = np.array([0, 120, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 80])
        upper_red2 = np.array([179, 255, 255])
        
        # lower_red1 = np.array([0, 140, 100])

        # lower_red1 = np.array([0, 60, 20])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([170, 60, 20])
        # upper_red2 = np.array([179, 255, 255])

        # upper_red1 = np.array([6, 255, 255])    # H从10→6
        # lower_red2 = np.array([173, 140, 100])  # H从170→173
        # upper_red2 = np.array([179, 255, 255])
        lower_green = np.array([35, 40, 20])
        upper_green = np.array([85, 255, 255])

        # Create masks
        red_mask = cv2.inRange(roi, lower_red1, upper_red1) | cv2.inRange(roi, lower_red2, upper_red2)
        green_mask = cv2.inRange(roi, lower_green, upper_green)

        # Morphological ops
        kernel = np.ones((5,5),np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        red_pixels = np.sum(red_mask > 0)
        green_pixels = np.sum(green_mask > 0)
        roi_area = roi.shape[0] * roi.shape[1]

        red_ratio = red_pixels / roi_area
        green_ratio = green_pixels / roi_area


        # Detection phase
        result = String()

        red_thresh = 0.001
        green_thresh = 0.001



        def largest_area(mask):
            contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return max ((cv2.contourArea(c) for c in contours), default=0)


        red_area = largest_area(red_mask)
        green_area = largest_area(green_mask)
        roi_area = roi.shape[0] * roi.shape[1]

        min_blob_area = 0.005 * roi_area

        # center filtering
        h, w, _ = roi.shape
        cx, cy = w//2, h//2
        def roughly_centered(mask):
            M = cv2.moments(mask)
            if M["m00"] == 0: return False
            mx = int(M['m10'] / M['m00'])
            my = int(M['m01'] / M['m00'])
            return abs(mx - cx) < w*0.35 and abs(my - cy) < h*0.35

        center_red = roughly_centered(red_mask)
        center_green = roughly_centered(green_mask)


        # comprehensive judgment
        if (red_ratio > red_thresh and red_area > min_blob_area and
            red_ratio > green_ratio * 1.2 and center_red):
            result.data = "False"

        elif (green_ratio > green_thresh and green_area > min_blob_area and
              green_ratio > red_ratio * 1.2 and center_green):
            result.data = "True"

        else:
            result.data = "Unknown"

        # 稳定result 0.3s

        raw = result.data

        if raw == self.last_raw:
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        self.last_raw = raw

        # Only publish when stable for >= threshold
        if self.stable_counter >= self.stable_threshold and raw != self.stable_result:
            self.stable_result = raw
            self.publisher_.publish(result)
            self.get_logger().info(f"STABLE DETECTED → {raw}")



def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

