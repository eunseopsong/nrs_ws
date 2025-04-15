#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        # topic 이름은 "waypoints", 큐 사이즈 10
        self.publisher_ = self.create_publisher(Float64MultiArray, 'waypoints', 10)
        # 사용자로부터 입력 받아 waypoint publish 진행
        self.publish_waypoints()

    def publish_waypoints(self):
        try:
            count = int(input("생성할 waypoint의 개수를 입력하세요: "))
        except Exception as e:
            self.get_logger().error("잘못된 숫자 입력입니다.")
            return

        waypoints = []
        for i in range(count):
            raw_input_str = input(f"Waypoint {i+1}의 좌표 (x y z)를 공백으로 구분하여 입력하세요: ")
            try:
                coords = [float(val) for val in raw_input_str.split()]
                if len(coords) != 3:
                    self.get_logger().error("좌표는 x, y, z 3개의 값이어야 합니다.")
                    return
                waypoints.extend(coords)
            except Exception as e:
                self.get_logger().error("잘못된 좌표 입력입니다.")
                return

        msg = Float64MultiArray()
        msg.data = waypoints
        self.publisher_.publish(msg)
        self.get_logger().info("아래와 같이 waypoint들을 publish 하였습니다: " + str(waypoints))

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    # 메시지 publish 후, 노드가 종료되기 전에 잠깐 spin
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
