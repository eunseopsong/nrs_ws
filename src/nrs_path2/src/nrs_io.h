#ifndef NRS_IO_H
#define NRS_IO_H

#include <rclcpp/rclcpp.hpp>                // ROS 2 기본 노드 기능
#include "std_srvs/srv/empty.hpp"           // Empty 서비스
#include "std_msgs/msg/string.hpp"          // 문자열 메시지
#include <sensor_msgs/msg/point_cloud2.hpp> // 포인트 클라우드 메시지

#include <boost/filesystem.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

// Waypoint 메시지 타입 (ROS 2)
// #include "nrs_path/msg/waypoint.hpp"
// #include "nrs_path/msg/waypoints.hpp"

// CGAL 관련
#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>

#include "nrs_math.h"

namespace fs = boost::filesystem;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Point_set_3<Point_3> Point_set;

class nrs_io
{
public:
    nrs_math n_math;
    std::string file_path_;
    int file_counter = 0;

    // 파일 내용 지우기
    void clearFile(const std::string &file_path);

    // Waypoints 저장
    void saveWaypointsToFile(const nrs_path::msg::Waypoints &final_waypoints,
                             const std::string &file_path);

    // 파일 전송 (노드 참조 필요)
    void sendFile(const std::string &file_path,
                  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &file_pub,
                  const rclcpp::Node::SharedPtr &node);
};

#endif // NRS_IO_H
