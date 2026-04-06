from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import time


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.color_frame_stamp = None
        self.depth_frame = None
        self.intrinsics = None
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.get_logger().info("Waiting for client's call...")

    def camera_info_callback(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_frame_stamp = str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_color_frame(self):
        return self.color_frame

    def get_color_frame_stamp(self):
        return self.color_frame_stamp

    def get_depth_frame(self):
        return self.depth_frame

    def get_camera_intrinsic(self):
        return self.intrinsics
    
    #############sh################
    def get_center_depth(self):
        """카메라 화면의 중심 픽셀에서의 depth 값을 반환 (단위: mm)"""
        if self.depth_frame is None:
            self.get_logger().warn("Depth frame is not available yet.")
            return None
        if self.intrinsics is None:
            self.get_logger().warn("Intrinsics not available yet.")
            return None
        try:
            cx = int(self.intrinsics["ppx"])  # ppx
            cy = int(self.intrinsics["ppy"])  # ppy
            depth = self.depth_frame[cy, cx]  # depth 값 자체가 mm

            if depth == 0:
                self.get_logger().warn("Depth at center is zero (no valid measurement).")
                return None
            
            return depth  # mm 단위
        
        except Exception as e:
            self.get_logger().warn(f"Error reading center depth: {e}")
            return None
    #############sh################

    ######Seojin##############
    def get_depth_roi(self, center=(320,240), size=10):
        if self.depth_frame is None:
            self.get_logger().warn("Depth frame is not available yet.")
            return None
        if self.intrinsics is None:
            self.get_logger().warn("Intrinsics not available yet.")
            return None
        try:
            x, y = center
            roi = self.depth_frame[y-size//2:y+size//2, x-size//2:x+size//2]
            roi = roi[roi>0]
            if len(roi) ==0 :
                return None
            return np.median(roi)
        
        except Exception as e:
            self.get_logger().warn(f"Error reading center depth: {e}")
            return None
    

    def get_depth_roi_with_offset(self, center=(320, 240), roi_size=10, offset=(0, 0)):
        "중심 좌표에서 ROI 범위만큼 depth를 추출하고, median을 반환"
        cx, cy = center
        dx, dy = offset
        cx += dx
        cy += dy    
        half = roi_size // 2
        x_start = max(0, cx - half)
        x_end = min(self.depth_frame.shape[1], cx + half)
        y_start = max(0, cy - half)
        y_end = min(self.depth_frame.shape[0], cy + half)    
        roi = self.depth_frame[y_start:y_end, x_start:x_end]
        valid_depths = roi[(roi > 100) & (roi < 1500)]  # 10cm~2m 사이만 사용    
        if valid_depths.size == 0:
            return None    
        return int(self.robust_median(valid_depths))
    
    def robust_median(self, data):
        if len(data) < 3:
            return np.median(data)
        q1 = np.percentile(data, 25)
        q3 = np.percentile(data, 75)
        iqr = q3 - q1
        filtered = data[(data > q1 - 1.5 * iqr) & (data < q3 + 1.5 * iqr)]
        return np.median(filtered) if filtered.size > 0 else np.median(data)

    def get_robust_depth(self, attempts=10, delay=0.05, center=(320, 240), roi_size=10, offset=(0, 0)):
        depths = []
        for _ in range(attempts):
            depth = self.get_depth_roi_with_offset(center=center, roi_size=roi_size, offset=offset)
            if depth is not None:
                depths.append(depth)
            time.sleep(delay)
        if not depths:
            return None    
        return int(np.median(depths))




        