from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urp_cvt_sampling',
            executable='cvt_sampler_node',
            name='cvt_sampler_node',
            output='screen',
            parameters=[{
                'surface_type': 'mesh',           # 또는 'flat'
                'mesh_path': '/home/jiseunghyun/meshes/surface.stl',
                'standoff': 0.005,
                'tool_axis': '+Z',
                'publish_rate_hz': 5.0,
                'save_txt_path': '/home/jiseunghyun/waypoints.txt',
            }]
        ),
        Node(
            package='urp_cvt_sampling',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'waypoints_topic': '/cvt_sampler_node/waypoints',
                'path_topic': '/path_planner_node/planned_path',
                'frame_id': 'map',
                'w_dist': 1.0,     # 거리 1m당 비용
                'w_rot': 0.1,     # 회전 1rad당 비용 (필요에 따라 조정)
                'max_2opt_iter': 50,
                'smooth_window': 0,   # 원하면 7 정도로 올리면 부드러움
                'publish_marker': True,
            }]
        )
    ])
