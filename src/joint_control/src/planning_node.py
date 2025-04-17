#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# push test

class PlanningNode(Node):
    def __init__(self):
        super().__init__('nrs_planning_node')
        
        # 1) "waypoints" 토픽 구독 (nrs_waypoint2에서 publish하는 웨이포인트)
        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'waypoints',            # 구독할 토픽 이름
            self.waypoints_callback,
            10
        )
        
        # 2) 새로운 "planned_trajectory" 토픽에 (x,y,z,roll,pitch,yaw) 결과 publish
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'planned_trajectory',   # publish할 토픽 이름
            10
        )
        
        self.get_logger().info('nrs_planning_node is up and running.')

    def waypoints_callback(self, msg):
        """
        nrs_waypoint2에서 받아온 Float64MultiArray 메시지를
        (x, y, z) 형태로 파싱한 뒤, 구간별로 100개씩 보간한다.
        그 결과 (x, y, z, roll, pitch, yaw) 형태의 데이터를 publish한다.
        """
        data = msg.data
        
        # 안전 검사: 총 길이가 3의 배수인지 확인 (x, y, z)
        if len(data) % 3 != 0:
            self.get_logger().error("Received waypoint data is not a multiple of 3!")
            return
        
        # (x, y, z) 순으로 파싱
        waypoints = []
        for i in range(0, len(data), 3):
            x = data[i]
            y = data[i+1]
            z = data[i+2]
            waypoints.append((x, y, z))
        
        self.get_logger().info(f"Received {len(waypoints)} waypoints: {waypoints}")
        
        # 인터폴레이션 진행
        interpolated_points = self.interpolate_waypoints(waypoints)
        
        # 결과 Publish
        out_msg = Float64MultiArray()
        out_msg.data = interpolated_points  # [x, y, z, roll, pitch, yaw, x, y, z, roll, pitch, yaw, ...]
        self.publisher_.publish(out_msg)
        self.get_logger().info(f"Published {len(interpolated_points)//6} interpolated end-effector points.")

    def interpolate_waypoints(self, waypoints):
        """
        각 인접 웨이포인트 사이를 선형 보간하여
        (x, y, z, roll, pitch, yaw) 형태 리스트를 flat하게 만들어 반환한다.
        
        서로 다른 2개의 웨이포인트 사이에 100개의 interpolation point를 생성.
        => (시작점 포함 101개 점, 끝점 포함 시 그 다음 구간의 시작점이 중복될 수 있음)
        """
        result = []
        num_interpolation_points = 100  # 두 점 사이에 생성할 보간 점 개수
        
        # roll, pitch, yaw는 예시로 0을 유지 (추후 원하는 로직으로 변경 가능)
        default_roll = 0.0
        default_pitch = 0.0
        default_yaw = 0.0
        
        for i in range(len(waypoints) - 1):
            x0, y0, z0 = waypoints[i]
            x1, y1, z1 = waypoints[i+1]
            
            # 0 <= t <= 1 사이를 100등분 => 101개 지점
            for step in range(num_interpolation_points + 1):
                t = step / float(num_interpolation_points)  # 0~1 사이 비율
                
                # 선형보간
                x = (1 - t) * x0 + t * x1
                y = (1 - t) * y0 + t * y1
                z = (1 - t) * z0 + t * z1
                
                # roll, pitch, yaw 역시 원하는 방식대로 보간 가능 (현재는 고정 0)
                roll = default_roll
                pitch = default_pitch
                yaw = default_yaw
                
                # Float64MultiArray에는 flat list로 입력
                result.extend([x, y, z, roll, pitch, yaw])
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)      # 콜백 함수가 호출될 수 있도록 spin
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
