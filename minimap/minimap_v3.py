import sys
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import QPixmap, QImage
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np

import yaml
from PIL import Image, ImageDraw

"""
1. v2오류 해결
2. turtle2 연동 예정
"""

class NODE(QThread, Node):
    signal = Signal(list)

    def __init__(self, node_name='ros_subscriber_node'):
        QThread.__init__(self)
        Node.__init__(self, node_name)

        self.amcl_pose_x = 0
        self.amcl_pose_y = 0

        #self.dot_size = 2

        self.sub_cmd_vel = self.create_subscription(
            PoseWithCovarianceStamped,
            '/turtle1/amcl_pose',  # turtle1의 amcl_pose 구독
            self.subscription_callback,
            10
        )


    def subscription_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.get_logger().info(f'Received amcl_pose x: {self.amcl_pose_x}, y: {self.amcl_pose_y}')
        print(f'Raw amcl_pose: x={self.amcl_pose_x}, y={self.amcl_pose_y}')
        
        # 맵 상의 좌표 계산
        map_x = -2.000753 + self.amcl_pose_x
        map_y = -17.086649 + self.amcl_pose_y
        print(f'Map coordinates: x={map_x}, y={map_y}')
        # 픽셀 좌표 계산
        pixel_x = map_x / 0.05
        pixel_y = map_y / 0.05
        print(f'Pixel coordinates: x={pixel_x}, y={pixel_y}')
        
        self.get_logger().info(f'emit signal')
        self.signal.emit([self.amcl_pose_x, self.amcl_pose_y])
        

    def run(self):
        rclpy.spin(self)


class GUI(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.signal.connect(self.process_signal)

        print("Loading map data...")
        try:
            with open('/home/july/map.yaml', 'r') as file:
                data = yaml.safe_load(file)
                self.resolution = data['resolution']
                origin = data['origin']
                self.map_x = -origin[0]
                self.map_y = -origin[1]

            print("Loading map image...")
            self.map_file = '/home/july/map.pgm'  # map_file 경로 설정
            print(f"Trying to open map from: {self.map_file}")
            self.map_image = Image.open(self.map_file)
            self.width, self.height = self.map_image.size
            print(f"Successfully loaded map image. Size: {self.width}x{self.height}")
            self.image_rgb = self.map_image.convert('RGB')

            self.map_y = self.map_y + self.height * self.resolution

        except Exception as e:
            print(f"Error: {e}")

        self.dot_size = 2
        self.resize = (820, 444)
        self.setupUi()

    def setupUi(self):
        # QLabel 초기화
        self.label = QLabel(self)
        self.label.setGeometry(10, 10, 500, 500)  # QLabel 크기 설정

        # 이미지 로드
        image = cv2.imread('/home/july/map.pgm', cv2.IMREAD_GRAYSCALE)
        if image is None:
            print("Error: Unable to load the image file")
            return

        # 맵 이미지를 90도 회전
        rotated_image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # OpenCV 이미지를 QImage로 변환
        height, width = rotated_image.shape
        bytes_per_line = width
        qimage = QImage(rotated_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)

        # QImage를 QPixmap으로 변환
        pixmap = QPixmap.fromImage(qimage)

        # QLabel에 QPixmap 설정
        self.label.setPixmap(pixmap)
        self.label.setScaledContents(True)  # QLabel에 맞게 비율 유지
        print("Image successfully displayed with 90-degree rotation.")


    def process_signal(self, message):
        # AMCL 좌표 수신
        self.amcl_pose_x = message[0]  # 인스턴스 변수로 저장
        self.amcl_pose_y = message[1]  # 인스턴스 변수로 저장

        # 맵 좌표를 픽셀 좌표로 변환 (RViz 좌표계와 GUI 좌표계 동기화)
        pixel_x = int((self.map_x + self.amcl_pose_x) / self.resolution)
        pixel_y = int(self.height - ((self.map_y + self.amcl_pose_y) / self.resolution))

        # 맵 이미지 범위 내로 클리핑
        pixel_x = max(0, min(self.width - 1, pixel_x))
        pixel_y = max(0, min(self.height - 1, pixel_y))

        # 좌표 확인
        print(f"Corrected Robot position - pixel: ({pixel_x}, {pixel_y})")

        # 로봇 현재 위치 갱신
        self.current_pixel = (pixel_x, pixel_y)
        self.input_image()

    def input_image(self):
        print("Drawing new position...")

        # OpenCV로 이미지 읽기
        image = cv2.imread('/home/july/map.pgm', cv2.IMREAD_GRAYSCALE)
        if image is None:
            print("Error loading image")
            return

        # 이미지 회전: 90도 반시계 방향 회전
        rotated_image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Grayscale -> BGR로 변환
        rotated_image_color = cv2.cvtColor(rotated_image, cv2.COLOR_GRAY2BGR)

        # 로봇 좌표를 픽셀 좌표로 변환
        try:
            # 좌표 변환 공식 (amcl_pose -> 픽셀 좌표)
            pixel_x = int((self.amcl_pose_y - (-17.0)) / self.resolution)
            pixel_y = int((self.amcl_pose_x - (-2.0)) / self.resolution)

            # 좌-우 반전 보정
            corrected_pixel_x = rotated_image_color.shape[1] - pixel_x  # 이미지 너비에서 x를 반전
            corrected_pixel_y = rotated_image_color.shape[0] - pixel_y  # 이미지 높이에서 y 조정

            # 경계값 확인 및 출력
            if 0 <= corrected_pixel_x < rotated_image_color.shape[1] and 0 <= corrected_pixel_y < rotated_image_color.shape[0]:
                print(f"Updated robot position on rotated map: ({corrected_pixel_x}, {corrected_pixel_y})")

                # 빨간색 점 그리기 (BGR: (0, 0, 255))
                cv2.circle(rotated_image_color, (corrected_pixel_x, corrected_pixel_y), 5, (0, 0, 255), -1)
            else:
                print("Error: Point is outside of the map boundaries.")
                return

        except Exception as e:
            print(f"Error during coordinate transformation: {e}")
            return

        # OpenCV 이미지를 QImage로 변환
        height, width, channel = rotated_image_color.shape
        bytes_per_line = 3 * width
        qimage = QImage(rotated_image_color.data, width, height, bytes_per_line, QImage.Format_BGR888)

        # QImage를 QPixmap으로 변환
        pixmap = QPixmap.fromImage(qimage)

        # QLabel에 QPixmap 설정
        self.label.setPixmap(pixmap)
        self.label.setScaledContents(True)





def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    #gui.window.show()
    gui.show()  # self.window.show() 대신 gui.show() 사용

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()

    
