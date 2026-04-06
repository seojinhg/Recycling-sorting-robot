import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from object_detection.yolo import YoloModel
from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG
from object_detection.realsense import ImgNode
import threading
from rclpy.executors import MultiThreadedExecutor
import pygame

package_path = get_package_share_directory("dream")
YOLO_MODEL_FILENAME = "garbage_sorting1.pt"
YOLO_MODEL_PATH = os.path.join(package_path, "resource", YOLO_MODEL_FILENAME)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100
# BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -20.0
DEPTH_OFFSET_CAN = -45.0
MIN_DEPTH = 30.0
#LABEL : {0: 'can', 1: 'glass', 2: 'paper', 3: 'plastic'}
LABEL = 0.0
X1, Y1, X2, Y2 = 0.0, 0.0, 0.0, 0.0
#JReady = [5.79, 0.73, 82.54, 0, 96.74, 5.79]
JReady = [-10.50, -7.09, 89.87, 0.02, 97.24, -10.50]
depth_list = []

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()

dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, posx, move_spiral
    
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

####################### TTS ###############################

detect_box = "garbage_box.mp3"
start_sorting = "garbage_sort.mp3"
make_empty = "garbage_box_make_empty.mp3"
No_detection = "No_detection.mp3"
finish_sorting = "finish_sorting.mp3"

#######################TrashBox############################
plastic_box = posx([629.99, 158.54, 194.38, 94.71, -178.79, -176.46])
can_box = posx([629.99, 8.54, 194.38, 94.71, -178.79, -176.46])
paper_box = posx([629.99, -141.46, 194.38, 94.71, -178.79, -176.46])
box_list = [plastic_box, can_box, paper_box]

########### Gripper Setup. Do not modify this area ########
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### Robot Controller ##############################
class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()
        #self.model = YoloModel(YOLO_MODEL_PATH) #################
        self.model = YoloModel()
        self.img_node = ImgNode()
        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()

            target_list = get_keyword_result.message.split()

            for target in target_list:
                print(f"****{target}****")
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn("No target position")
                    self.TTS(No_detection)
                else:
                    self.get_logger().info(f"target position: {target_pos}")
                    self.pick_and_place_target(target_pos)
                    self.init_robot()

        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            global LABEL, X1, X2, Y1, Y2
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")

            # 수정된 부분 #
            bbox, score, label = self.model.get_best_detection(self.img_node, target)
            LABEL = label
            if bbox:
                x1, y1, x2, y2 = map(int, bbox)
                self.get_logger().info(f"[YOLO] Label: {label}, Confidence: {score:.2f}, BBox: [{x1}, {y1}, {x2}, {y2}]\n")
                X1 = x1
                Y1 = y1
                X2 = x2
                Y2 = y2
  
            else:
                self.get_logger().warn("YOLO did not find the target confidently.")
            # ================== #

            if sum(result) == 0:
                print("No target position")
                self.TTS(No_detection)
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                ####################################
                if LABEL == 0:
                    td_coord[2] += DEPTH_OFFSET_CAN

                else:
                    td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos

    def init_robot(self):
        # JReady = [0, 0, 90, 0, 90, 0]

        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()


    def pick_and_place_target(self, target_pos):
        self.TTS(start_sorting)
        print("current pos = ", get_current_posx())
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()

        if X2-X1 > Y2-Y1:
            movel([0,0,0,0,0,-90], vel=VELOCITY, acc=ACC, mod=1)
            mwait
            
        if LABEL == 0:
            movel([0,30,0,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
            mwait()
        print(f"********{LABEL}********")

        gripper.close_gripper()

        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()
        movel([0,0,100,0,0,0], vel=VELOCITY, acc=ACC, mod=1)

        print(f"depth_list:::{depth_list}")
        # move to trashcan
        if LABEL == 0:
            movel(can_box, vel=VELOCITY, acc=ACC)
            mwait()
            retry_cnt = 0 
            #real-time depth
            #depth = self.img_node.get_center_depth()
            depth = self.img_node.get_robust_depth(center=(320,240), roi_size=20, offset=(0,-20))

            while depth is None and retry_cnt <5:
                time.sleep(0.1)
                depth = self.img_node.get_robust_depth(center=(320,240), roi_size=20, offset=(0,-20))
                mwait()
                print(f"can_depth:::{depth}")
                retry_cnt += 1
            if depth is not None:
                print(f"can_depth:::{depth}")
                depth_list[1] = depth
                if depth_list[1] -50 > 325:
                    movel([0,0,-50,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
                else:
                    self.get_logger().warn("Please empty the box")
                    self.TTS(make_empty)
        elif LABEL == 2:
            movel(paper_box, vel=VELOCITY, acc=ACC)
            mwait()
            depth = self.img_node.get_robust_depth(center=(320,240), roi_size=20, offset=(0,-20))
            print(f"paper_depth:::{depth}")
            mwait()
            if depth is not None:
                depth_list[2] = depth
                if depth_list[2] - 10 > 325:
                    movel([0,0,-50,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
                else:
                    self.get_logger().warn("Please empty the box")
                    self.TTS(make_empty)
        elif LABEL == 3:
            movel(plastic_box, vel=VELOCITY, acc=ACC)
            depth = self.img_node.get_robust_depth(center=(320,240), roi_size=20, offset=(0,-20))
            print(f"plastic_depth:::{depth}")
            if depth is not None:
                depth_list[0] = depth
                if depth_list[0] -50  > 325:
                    movel([0,0,-50,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
                else:
                    self.get_logger().warn("Please empty the box")
                    self.TTS(make_empty)

        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
    
    def TTS(self, action):
        try:
            base_path = "/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/dream"
            self.path = os.path.join(base_path, action)

            # Init mixer only once outside this function ideally
            if not pygame.mixer.get_init():
                freq = 24000
                bitsize = -16
                channels = 1
                buffer = 2048
                pygame.mixer.init(freq, bitsize, channels, buffer)

            pygame.mixer.music.load(self.path)
            pygame.mixer.music.play()

            clock = pygame.time.Clock()
            while pygame.mixer.music.get_busy():
                clock.tick(30)  # 30 fps 대기

            # Don't quit mixer here if you want to reuse sound later
            # pygame.mixer.quit()
            mwait()

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")



def main(args=None):
    gripper.open_gripper()
    while gripper.get_status()[0]:
            time.sleep(0.5)

    global depth_list

    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.img_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    node.TTS(detect_box)

    for i in box_list:
        movel(i, vel=VELOCITY, acc=ACC)
        mwait()
        time.sleep(1)
        
        depth = node.img_node.get_center_depth()
        while depth is None:
            time.sleep(0.1)
            depth = node.img_node.get_center_depth()
        print(f"##--##{depth}##--##")###
        depth_list.append(depth)
        if depth < 325:
            node.get_logger().warn("Please empty the box before start")
            node.TTS(make_empty)


    movej(JReady, vel=VELOCITY, acc=ACC)

    while rclpy.ok():
        node.robot_control()
        node.TTS(finish_sorting)
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
