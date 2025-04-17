#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class InterpolationPublisher(Node):
    def __init__(self):
        super().__init__('interpolation_publisher')
        # 1) subscribe to raw waypoints
        self.sub = self.create_subscription(
            Float64MultiArray,
            'waypoints',
            self.on_waypoints,
            10
        )
        # 2) publish interpolated trajectory
        self.pub = self.create_publisher(
            Float64MultiArray,
            'planned_trajectory',
            10
        )
        self.get_logger().info('InterpolationPublisher up and running.')

    def on_waypoints(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().error(
                f'Bad waypoint array length {len(data)} (not multiple of 3)')
            return

        # Group into [(x,y,z), ...]
        wpts = [
            (data[i], data[i+1], data[i+2])
            for i in range(0, len(data), 3)
        ]
        self.get_logger().info(f'Received {len(wpts)} waypoints.')

        # Linear interpolate + zero orientation
        traj = self.interpolate(wpts)

        out = Float64MultiArray()
        out.data = traj
        self.pub.publish(out)
        self.get_logger().info(f'Published {len(traj)//6} trajectory points.')

    def interpolate(self, wpts):
        """Return flat [x,y,z,roll,pitch,yaw,...] by linear‑interpolating
        100 steps between each consecutive waypoint pair."""
        result = []
        steps = 100
        for (x0,y0,z0), (x1,y1,z1) in zip(wpts, wpts[1:]):
            for i in range(steps+1):
                t = i / steps
                x = (1-t)*x0 + t*x1
                y = (1-t)*y0 + t*y1
                z = (1-t)*z0 + t*z1
                # fixed orientation
                roll = pitch = yaw = 0.0
                result.extend([x, y, z, roll, pitch, yaw])
        return result

def main(args=None):
    rclpy.init(args=args)
    node = InterpolationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
