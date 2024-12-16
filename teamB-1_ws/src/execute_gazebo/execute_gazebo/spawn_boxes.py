import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class RandomBoxSpawner(Node):
    def __init__(self):
        super().__init__('random_box_spawner')
        
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_entity service...')
            
        self.get_logger().info('SpawnEntity service available!')

    def spawn_random_boxes(self):
        # 색깔별 상자 모델 이름
        colors = ["box_red", "box_green", "box_blue", "box_yellow"]
        random.shuffle(colors)  # 리스트 셔플

        # 상자의 위치 설정 (순서대로 위치를 다르게 배치)
        positions = [
            (0.000753, 7.300000, 1.082375),  # 첫 번째 상자의 위치
            (0.000753, 2.586649, 1.082375),  # 두 번째 상자의 위치
            (0.000753, -1.413351, 1.082375),  # 세 번째 상자의 위치
            (0.000753, -5.413351, 1.082375)   # 네 번째 상자의 위치
        ]

        # 셔플된 색상을 순서대로 배치
        for box_id, (color, (x, y, z)) in enumerate(zip(colors, positions)):
            self.spawn_box(color, box_id, x, y, z)

    def spawn_box(self, selected_color, box_id, x, y, z):
        # Gazebo SpawnEntity 서비스 요청 생성
        request = SpawnEntity.Request()
        request.name = f"{selected_color}_{box_id}"  # 각 상자의 고유 이름 지정
        request.xml = f"""
        <sdf version="1.6">
          <model name="{selected_color}">
            <include>
              <uri>model://{selected_color}</uri>
            </include>
          </model>
        </sdf>
        """
        request.robot_namespace = ''
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        # 서비스 호출
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned {selected_color} at ({x}, {y}, {z})")
        else:
            self.get_logger().error("Failed to spawn box")

def main():
    rclpy.init()
    spawner = RandomBoxSpawner()

    # 셔플된 색상 리스트로 상자 생성
    spawner.spawn_random_boxes()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

