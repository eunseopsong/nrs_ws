#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'waypoints',
            10
        )

        # 사용자 입력으로 waypoints 한 번만 받아서 저장
        self.waypoints_msg = self.get_user_waypoints()

        # 1 Hz 타이머: 매 초마다 동일 메시지를 퍼블리시
        self.timer_ = self.create_timer(1.0, self.publish_timer_callback)
        self.get_logger().info('WaypointPublisher ready; publishing at 1 Hz.')

    def get_user_waypoints(self) -> Float64MultiArray:
        """
        터미널에서 사용자에게 웨이포인트 개수, 좌표(x y z)를 입력받아
        Float64MultiArray 메시지로 리턴.
        """
        try:
            count = int(input("생성할 waypoint의 개수를 입력하세요: "))
        except ValueError:
            self.get_logger().error("잘못된 숫자 입력입니다. 노드 종료.")
            rclpy.shutdown()
            return

        flat = []
        for i in range(1, count+1):
            s = input(f"Waypoint {i}의 좌표(x y z)를 공백으로 구분하여 입력하세요: ")
            parts = s.strip().split()
            if len(parts) != 3:
                self.get_logger().error("좌표는 반드시 3개(x y z)이어야 합니다. 노드 종료.")
                rclpy.shutdown()
                return
            try:
                coords = [float(v) for v in parts]
            except ValueError:
                self.get_logger().error("좌표는 숫자여야 합니다. 노드 종료.")
                rclpy.shutdown()
                return
            flat.extend(coords)

        msg = Float64MultiArray()
        msg.data = flat
        self.get_logger().info(f"입력된 waypoints: {flat}")
        return msg

    def publish_timer_callback(self):
        """타이머에 의해 매 초 호출되어 저장된 메시지를 퍼블리시."""
        self.publisher_.publish(self.waypoints_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
