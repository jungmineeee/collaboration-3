from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.srv import DeleteEntity
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# 색상 범위 정의 (HSV)
COLOR_RANGES = {
    "red1": (np.array([0, 100, 100]), np.array([10, 255, 255])),  # 빨간색 범위 1
    "red2": (np.array([160, 100, 100]), np.array([179, 255, 255])),  # 빨간색 범위 2
    "blue": (np.array([100, 150, 0]), np.array([140, 255, 255])),  # 파란색
    "green": (np.array([35, 100, 100]), np.array([85, 255, 255])),  # 연두색/초록색
    "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),  # 노란색
}

COORDINATES = {
    "destination" : [16.2153, -5.5443, 3.850],
    "blue" : [10.4869, -3.8000, 1.57],
    "red" : [7.9315, -3.8000, 1.57],
    "green" : [5.5403, -3.8000, 1.57],
    'yellow' : [2.9746, -3.8000, 1.57],
}

#####################################################yellow
class ControllerNode(Node):
    def __init__(self):
        super().__init__('contorller_node')
        self.callback_group = ReentrantCallbackGroup()
        self.turtle1_init_pose = [0.0, 0.0, 0.0, 1.0]
        self.turtle2_init_pose = [7.7350, -6.9434, 0.7071, 0.7071]
        self.subscription_robotcam = None
        
        self.dest_lever = None
        self.current_target = None
        self.num = 3
        
        # # 퍼블리셔 생성
        # self.publisher = self.create_publisher(
        #     String,
        #     'command_topic',
        #     10, # qos
        #     callback_group=self.callback_group # 콜백 그룹
        # )
        # self.publisher
            
        # def publish_message(self, message):
        #     """Publish a message to the button_click_topic."""
        #     msg = String()
        #     msg.data = message
        #     self.publisher.publish(msg)
        #     self.get_logger().info(f"퍼블리시 (메세지 / 신호): {message}")
        
        
        ######################################################
        # 로봇 동작 액션 (turtle1)
        self.turtlebot1_nav_client = ActionClient(
            self, 
            NavigateToPose, 
            '/turtle1/navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # # 초기 위치 설정 서비스 (클라이언트 -> 서버(nav2)) (turtle1)
        # self.turtlebot1_set_initial_pose_service_client = self.create_client(
        #     SetInitialPose,
        #     '/turtle1/set_initial_pose',
        # )
        
        # # setting 2D position estimate
        # while not self.turtlebot1_set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service /turtle1/set_initial_pose not available, waiting again...')
        ###################################################### 
        
        ######################################################
        # 로봇 동작 액션 (turtle2)
        self.turtlebot2_nav_client = ActionClient(
            self, 
            NavigateToPose, 
            '/turtle2/navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # # 초기 위치 설정 서비스 (클라이언트 -> 서버(nav2)) (turtle2)
        # self.turtlebot2_set_initial_pose_service_client = self.create_client(
        #     SetInitialPose,
        #     '/turtle2/set_initial_pose',
        # )
        
        # # setting 2D position estimate
        # while not self.turtlebot2_set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service /turtle2/set_initial_pose not available, waiting again...')
        
        ######################################################
        
        # self.turtle1_set_initial_pose(*self.turtle1_init_pose)
        # self.turtle2_set_initial_pose(*self.turtle2_init_pose)
        """ 2D Pose Estimate 안해줘도 되네??"""
                
        self.delete_client = self.create_client(
            DeleteEntity, 
            '/delete_entity',
            callback_group=self.callback_group
        )
        
        while not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Delete Service not available, waiting...')
        
        
    def robotcam_subscription(self, topic_name, callback):
        # 로봇캠 이미지를 subscribe 하는 서브스크라이버
        self.subscription_robotcam = self.create_subscription(
            Image,
            topic_name, # 서브스크라이버 생성시 인자
            callback,   # 서브스크라이버 생성시 인자
            10,
            callback_group=self.callback_group
        )
        self.bridge = CvBridge()
        self.get_logger().info(f"Subscribed to topic: {topic_name}")
        
    # 2D position estimation (실행시 호출)
    def turtle1_set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.02]

        future = self.turtlebot1_set_initial_pose_service_client.call_async(req)
    
        # 2D position estimation (실행시 호출)
    
    def turtle2_set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.02]

        future = self.turtlebot2_set_initial_pose_service_client.call_async(req)
    
    #### Action Logic (turtle1) ####
    def turtlebot1_send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.turtlebot1_nav_client.wait_for_server() # 서버 연결 대기
        
        self.get_logger().info(f'Goal을 전송하였습니다.')
        
        self.send_goal_future = self.turtlebot1_nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.turtlebot1_feedback_callback)
        self.send_goal_future.add_done_callback(self.turtlebot1_goal_response_callback)

    def turtlebot1_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 피드백은 터미널에서 Log로만 확인 / 실제 GUI에 띄우지는 않음
        self.get_logger().info(f"로봇의 현재 위치 : {feedback.current_pose}")

    def turtlebot1_goal_response_callback(self, future):
        self.turtle1_goal_handle = future.result()
        
        if not self.turtle1_goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self.action_result_future = self.turtle1_goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.turtlebot1_get_result_callback)
        
    def turtlebot1_get_result_callback(self, future):
        result = future.result().result

        self.get_logger().info(f'목표 지점에 도착하였습니다.')

    #### Action Logic (turtle2) ####
    def turtlebot2_send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.turtlebot2_nav_client.wait_for_server() # 서버 연결 대기
        
        self.get_logger().info(f'Goal을 전송하였습니다.')
        
        self.send_goal_future = self.turtlebot2_nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.turtlebot2_feedback_callback)
        self.send_goal_future.add_done_callback(self.turtlebot2_goal_response_callback)

    def turtlebot2_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 피드백은 터미널에서 Log로만 확인 / 실제 GUI에 띄우지는 않음
        self.get_logger().info(f"로봇의 현재 위치 : {feedback.current_pose}")

    def turtlebot2_goal_response_callback(self, future):
        self.turtle2_goal_handle = future.result()
        
        if not self.turtle2_goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self.action_result_future = self.turtle2_goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.turtlebot2_get_result_callback)
        
    def turtlebot2_get_result_callback(self, future):
        result = future.result().result

        self.get_logger().info(f'목표 지점에 도착하였습니다.')
        
        if self.current_target == self.dest_lever :
            self.delete_box(f"box_{self.current_target}")
            self.num -= 1
        
        else : 
            pass


    #### 로봇 정지 (turtle1) ####
    def turtle1_cancel_goal(self):
        if self.turtle1_goal_handle is not None:
            self.get_logger().info('Cancelling the goal...')
            self.cancel_future = self.turtle1_goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self.turtle1_cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    def turtle1_cancel_done_callback(self, future):
        cancel_result = future.result()
        
        self.get_logger().info('Goal successfully cancelled.')
        
    #### 로봇 정지 (turtle2) ####
    def turtle2_cancel_goal(self):
        if self.turtle2_goal_handle is not None:
            self.get_logger().info('Cancelling the goal...')
            self.cancel_future = self.turtle2_goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self.turtle2_cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    def turtle2_cancel_done_callback(self, future):
        cancel_result = future.result()
        
        self.get_logger().info('Goal successfully cancelled.')
        
    ### 벽 제거 ###
    def delete_box(self, box_name):
        request = DeleteEntity.Request()
        request.name = f'{box_name}_{self.num}'
        future = self.delete_client.call_async(request)
        
        # Add a done callback to handle the response asynchronously
        future.add_done_callback(self.delete_box_callback)

    def delete_box_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f"Wall is deleted successfully")
            else:
                self.get_logger().error("Failed to delete box")
        except Exception as e:
            self.get_logger().error(f"Error while deleting box: {str(e)}")


#####################################################
class Ros2Worker(QThread):
    robotcam_image_received = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.node = None
        self.running = True
        self.topic_name = '/turtle1/camera/image_raw'

    def run(self):
        """Start ROS2 spinning in this thread."""
        rclpy.init()
        self.node = ControllerNode()
        
        self.node.robotcam_subscription(self.topic_name, self.handle_robotcam_image)
        
        # MultiThreadedExecutor 설정
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)

        while self.running:
            self.executor.spin_once(timeout_sec=0.1)
            
        # Clean up
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        """Stop the ROS2 spinning loop."""
        self.running = False
        self.quit()
        self.wait()
        
    def handle_robotcam_image(self, msg):
        try :
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.node.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.robotcam_image_received.emit(cv_image)
        
        except Exception as e :
            self.node.get_logger().error(f"Failed to process image: {e}")
    
    # def publish_message(self, message):
    #     if self.node:
    #         self.node.publish_message(message)


#####################################################
class Controller_GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.detected_color = None
        
        # Initialize ROS2 worker
        self.ros_worker = Ros2Worker()
        self.ros_worker.robotcam_image_received.connect(self.update_robotcam_image)
        self.ros_worker.start()
        
        self.setupUi()
        
        
        # event handler
        self.robot1_goal_btn.clicked.connect(self.set_destination)
        self.robot1_stop_btn.clicked.connect(self.robot1_stop)
        self.blue_btn.clicked.connect(self.go_blue_lever)
        self.red_btn.clicked.connect(self.go_red_lever)
        self.green_btn.clicked.connect(self.go_green_lever)
        self.yellow_btn.clicked.connect(self.go_yellow_lever)

    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(633, 709)
        font = QtGui.QFont()
        font.setPointSize(11)
        self.setFont(font)
        self.setAutoFillBackground(False)
        self.setStyleSheet("")

        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")

        self.robot1_goal_btn = QtWidgets.QPushButton(self.centralwidget)
        self.robot1_goal_btn.setGeometry(QtCore.QRect(30, 520, 221, 71))
        self.robot1_goal_btn.setStyleSheet("QPushButton {\n"
                                           "     background-color: rgb(153, 193, 241);  /* 버튼 배경색 */\n"
                                           "    border-radius: 6px;        /* 둥근 테두리 */\n"
                                           "    border: 1px solid rgb(119, 118, 123); /* 테두리 색상 */\n"
                                           "}\n"
                                           "\n"
                                           "QPushButton:hover {\n"
                                           "    background-color: rgb(164, 202, 247);  /* 마우스 오버 시 색상 */\n"
                                           "}\n"
                                           "\n"
                                           "QPushButton:pressed {\n"
                                           "    background-color: rgb(135, 178, 230);  /* 버튼 클릭 시 색상 */\n"
                                           "}\n"
                                           "")
        self.robot1_goal_btn.setObjectName("robot1_goal_btn")
        self.robot1_goal_btn.setText("목표위치 재전송")

        self.robot1_stop_btn = QtWidgets.QPushButton(self.centralwidget)
        self.robot1_stop_btn.setGeometry(QtCore.QRect(30, 610, 221, 71))
        self.robot1_stop_btn.setStyleSheet("QPushButton {\n"
                                           "     background-color: rgb(255, 152, 141);  /* 버튼 배경색 */\n"
                                           "    border-radius: 6px;        /* 둥근 테두리 */\n"
                                           "    border: 1px solid rgb(119, 118, 123); /* 테두리 색상 */\n"
                                           "}\n"
                                           "\n"
                                           "QPushButton:hover {\n"
                                           "    background-color: rgb(255, 175, 167);\n"
                                           "}\n"
                                           "\n"
                                           "QPushButton:pressed {\n"
                                           "    background-color: rgb(255, 131, 118);  /* 버튼 클릭 시 색상 */\n"
                                           "}\n"
                                           "")
        self.robot1_stop_btn.setObjectName("robot1_stop_btn")
        self.robot1_stop_btn.setText("STOP")

        self.title_label = QtWidgets.QLabel(self.centralwidget)
        self.title_label.setGeometry(QtCore.QRect(110, 30, 411, 51))
        font = QtGui.QFont()
        font.setFamily("KacstBook")
        font.setPointSize(16)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.title_label.setFont(font)
        self.title_label.setStyleSheet("background-color: rgb(246, 245, 244);\n"
                                      "border: 1px solid gray;\n"
                                      "border-radius: 6px;")
        self.title_label.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.title_label.setLineWidth(1)
        self.title_label.setScaledContents(False)
        self.title_label.setAlignment(QtCore.Qt.AlignCenter)
        self.title_label.setObjectName("title_label")
        self.title_label.setText("협동 방탈출 게임 컨트롤러")

        self.camera_view = QtWidgets.QLabel(self.centralwidget)
        self.camera_view.setGeometry(QtCore.QRect(30, 100, 571, 311))
        self.camera_view.setStyleSheet("background-color: rgb(36, 31, 49);\n"
                                       "color: white;\n"
                                       "border-radius: 6px;")
        self.camera_view.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_view.setObjectName("camera_view")
        self.camera_view.setText("Robot1's camera image")

        self.wall_color_label = QtWidgets.QLabel(self.centralwidget)
        self.wall_color_label.setGeometry(QtCore.QRect(50, 430, 161, 31))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.wall_color_label.sizePolicy().hasHeightForWidth())
        self.wall_color_label.setSizePolicy(sizePolicy)
        self.wall_color_label.setStyleSheet("font-weight: bold;")
        self.wall_color_label.setAlignment(QtCore.Qt.AlignCenter)
        self.wall_color_label.setObjectName("wall_color_label")
        self.wall_color_label.setText("앞을 막고 있는 벽의 색깔")

        self.wall_color = QtWidgets.QLabel(self.centralwidget)
        self.wall_color.setGeometry(QtCore.QRect(230, 430, 371, 31))
        self.wall_color.setStyleSheet("background-color: rgb(192, 191, 188);\n"
                                      "border-radius: 6px;\n"
                                      "color: rgb(61, 56, 70);")
        self.wall_color.setAlignment(QtCore.Qt.AlignCenter)
        self.wall_color.setObjectName("wall_color")
        self.wall_color.setText("Nothing Detected...")

        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(280, 520, 321, 161))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")

        self.blue_btn = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blue_btn.sizePolicy().hasHeightForWidth())
        self.blue_btn.setSizePolicy(sizePolicy)
        self.blue_btn.setStyleSheet("QPushButton {\n"
                                    "     background-color: rgb(28, 113, 216);  /* 버튼 배경색 */\n"
                                    "    border-radius: 6px;        /* 둥근 테두리 */\n"
                                    "    border: 1px solid rgb(119, 118, 123); /* 테두리 색상 */\n"
                                    "}\n"
                                    "\n"
                                    "QPushButton:hover {\n"
                                    "    background-color: rgb(41, 124, 225);\n"
                                    "}\n"
                                    "\n"
                                    "QPushButton:pressed {\n"
                                    "    background-color: rgb(18, 103, 207);  /* 버튼 클릭 시 색상 */\n"
                                    "}\n"
                                    "")
        self.blue_btn.setObjectName("blue_btn")
        self.blue_btn.setText("Blue")
        self.gridLayout.addWidget(self.blue_btn, 0, 0, 1, 1)

        self.green_btn = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.green_btn.sizePolicy().hasHeightForWidth())
        self.green_btn.setSizePolicy(sizePolicy)
        self.green_btn.setStyleSheet("QPushButton {\n"
                                     "     background-color: rgb(38, 162, 105);  /* 버튼 배경색 */\n"
                                     "    border-radius: 6px;        /* 둥근 테두리 */\n"
                                     "    border: 1px solid rgb(119, 118, 123); /* 테두리 색상 */\n"
                                     "}\n"
                                     "\n"
                                     "QPushButton:hover {\n"
                                     "    background-color: rgb(54, 175, 119);\n"
                                     "}\n"
                                     "\n"
                                     "QPushButton:pressed {\n"
                                     "    background-color: rgb(23, 149, 91);  /* 버튼 클릭 시 색상 */\n"
                                     "}\n"
                                     "")
        self.green_btn.setObjectName("green_btn")
        self.green_btn.setText("Green")
        self.gridLayout.addWidget(self.green_btn, 1, 0, 1, 1)

        self.red_btn = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.red_btn.sizePolicy().hasHeightForWidth())
        self.red_btn.setSizePolicy(sizePolicy)
        self.red_btn.setStyleSheet("QPushButton {\n"
                                   "     background-color: rgb(224, 27, 36);  /* 버튼 배경색 */\n"
                                   "    border-radius: 6px;        /* 둥근 테두리 */\n"
                                   "    border: 1px solid rgb(119, 118, 123); /* 테두리 색상 */\n"
                                   "}\n"
                                   "\n"
                                   "QPushButton:hover {\n"
                                   "    background-color: rgb(231, 42, 50);\n"
                                   "}\n"
                                   "\n"
                                   "QPushButton:pressed {\n"
                                   "    background-color: rgb(214, 16, 25);  /* 버튼 클릭 시 색상 */\n"
                                   "}\n"
                                   "")
        self.red_btn.setObjectName("red_btn")
        self.red_btn.setText("Red")
        self.gridLayout.addWidget(self.red_btn, 0, 1, 1, 1)

        self.yellow_btn = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.yellow_btn.sizePolicy().hasHeightForWidth())
        self.yellow_btn.setSizePolicy(sizePolicy)
        self.yellow_btn.setStyleSheet("QPushButton {\n"
                                   "     background-color: rgb(246, 211, 45);  /* 버튼 배경색 */\n"
                                   "    border-radius: 6px;        /* 둥근 테두리 */\n"
                                   "    border: 1px solid rgb(119, 118, 123); /* 테두리 색상 */\n"
                                   "}\n"
                                   "\n"
                                   "QPushButton:hover {\n"
                                   "    background-color: rgb(255, 223, 71);\n"
                                   "}\n"
                                   "\n"
                                   "QPushButton:pressed {\n"
                                   "    background-color: rgb(235, 199, 27);  /* 버튼 클릭 시 색상 */\n"
                                   "}\n"
                                   "")
        self.yellow_btn.setObjectName("yellow_btn")
        self.yellow_btn.setText("Yellow")
        self.gridLayout.addWidget(self.yellow_btn, 1, 1, 1, 1)

        self.robot1_label = QtWidgets.QLabel(self.centralwidget)
        self.robot1_label.setGeometry(QtCore.QRect(50, 480, 181, 31))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.robot1_label.sizePolicy().hasHeightForWidth())
        self.robot1_label.setSizePolicy(sizePolicy)
        self.robot1_label.setStyleSheet("font-weight: bold;")
        self.robot1_label.setAlignment(QtCore.Qt.AlignCenter)
        self.robot1_label.setObjectName("robot1_label")
        self.robot1_label.setText("[ Robot1 ]")

        self.robot2_label = QtWidgets.QLabel(self.centralwidget)
        self.robot2_label.setGeometry(QtCore.QRect(350, 480, 181, 31))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.robot2_label.sizePolicy().hasHeightForWidth())
        self.robot2_label.setSizePolicy(sizePolicy)
        self.robot2_label.setStyleSheet("font-weight: bold;")
        self.robot2_label.setAlignment(QtCore.Qt.AlignCenter)
        self.robot2_label.setObjectName("robot2_label")
        self.robot2_label.setText("[ Robot2 ]")

        self.setCentralWidget(self.centralwidget)

    def detect_color(self, cv_image):
        detected_color = None
        # BGR 이미지를 HSV로 변환
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        for color_name, (lower, upper) in COLOR_RANGES.items():
            # 색상 범위 내의 픽셀 추출
            mask = cv2.inRange(hsv_image, lower, upper)
            
            # 마스크에서 픽셀 개수를 계산
            color_pixels = cv2.countNonZero(mask)
            
            # 특정 기준 이상의 픽셀이 있으면 색상이 감지된 것으로 간주
            if color_pixels > 100:  # 감지 기준 픽셀 수 (조정 가능)
                detected_color = color_name
        
        return detected_color
    
    def update_robotcam_image(self, cv_image):
        # OpenCV 이미지를 QImage로 변환
        height, width, channels = cv_image.shape
        bytes_per_line = channels * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(q_image)

        # QLabel에 이미지 업데이트
        self.camera_view.setPixmap(pixmap)

        # 색상 감지
        what_color = self.detect_color(cv_image)
        
        if self.detected_color != what_color :            
            # 빨간색 통일
            if what_color == "red1" or what_color == "red2" : what_color = "red"
                
            # GUI 업데이트
            self.detected_color = what_color
            print(f"색상 감지 - {self.detected_color}")
            
            if self.detected_color == 'red' :
                self.wall_color.setText("빨간색 벽이 있어!")
                self.wall_color.setStyleSheet("background-color: rgb(224, 27, 36);\n"
                                              "border-radius: 6px;\n"
                                              "color: rgb(61, 56, 70);")

                self.ros_worker.node.current_target = 'red'
                
            elif self.detected_color == 'blue' :
                self.wall_color.setText("파란색 벽이 있어!")
                self.wall_color.setStyleSheet("background-color: rgb(28, 113, 216);\n"
                                              "border-radius: 6px;\n"
                                              "color: rgb(61, 56, 70);")
                
                self.ros_worker.node.current_target = 'blue'
                
            elif self.detected_color == 'green' :
                self.wall_color.setText("초록색 벽이 있어!")
                self.wall_color.setStyleSheet("background-color: rgb(38, 162, 105);\n"
                                              "border-radius: 6px;\n"
                                              "color: rgb(61, 56, 70);")
                
                self.ros_worker.node.current_target = 'green'
                
            elif self.detected_color == 'yellow' :
                self.wall_color.setText("노란색 벽이 있어!")
                self.wall_color.setStyleSheet("background-color: rgb(246, 211, 45);\n"
                                              "border-radius: 6px;\n"
                                              "color: rgb(61, 56, 70);")
                
                self.ros_worker.node.current_target = 'yellow'
                
            else :
                self.wall_color.setText("Nothing Detected...")
                self.wall_color.setStyleSheet("background-color: rgb(192, 191, 188);\n"
                                              "border-radius: 6px;\n"
                                              "color: rgb(61, 56, 70);")
                
                if self.ros_worker.node.num < 0 :
                    self.ros_worker.node.current_target = None

    def set_destination(self) :
        dest = COORDINATES["destination"]
        self.ros_worker.node.turtlebot1_send_goal(dest[0], dest[1], dest[2])
        
    def robot1_stop(self) :
        self.ros_worker.node.turtle1_cancel_goal()

    def go_red_lever(self) :
        self.ros_worker.node.dest_lever = "red"
        
        dest = COORDINATES["red"]
        self.ros_worker.node.turtlebot2_send_goal(dest[0], dest[1], dest[2])
        

    def go_blue_lever(self) :
        self.ros_worker.node.dest_lever = "blue"
        
        dest = COORDINATES["blue"]
        self.ros_worker.node.turtlebot2_send_goal(dest[0], dest[1], dest[2])
        
        
    def go_green_lever(self) :
        self.ros_worker.node.dest_lever = "green"
        
        dest = COORDINATES["green"]
        self.ros_worker.node.turtlebot2_send_goal(dest[0], dest[1], dest[2])
        
        
    def go_yellow_lever(self) :
        self.ros_worker.node.dest_lever = "yellow"
        
        dest = COORDINATES["yellow"]
        self.ros_worker.node.turtlebot2_send_goal(dest[0], dest[1], dest[2])
        
        
        
#####################################################
def main():
    app = QApplication(sys.argv)

    # Login Window
    window = Controller_GUI()
    window.show()
    
    sys.exit(app.exec_())


#####################################################
if __name__ == '__main__':
    main()
