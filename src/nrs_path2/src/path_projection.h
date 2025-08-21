// src/path_projection.h
#pragma once

// STL
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <sstream>
#include <chrono>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

// TF / Eigen
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Core>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Ray_3.h>
#include <CGAL/IO/STL.h>          // read_STL(mesh) 등

// custom msgs
#include "nrs_path2/msg/waypoint.hpp"
#include "nrs_path2/msg/waypoints.hpp"

// -----------------------
// CGAL typedefs
// -----------------------
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;
typedef Traits::Point_3 Point_3;
typedef Kernel::Ray_3 Ray_3;
typedef Kernel::Vector_3 Vector_3;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Triangle_mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> AABB_tree;

// -----------------------
// using aliases (ROS msgs)
// -----------------------
using geometry_msgs::msg::Point;
using std_msgs::msg::String;
using nrs_path2::msg::Waypoint;
using nrs_path2::msg::Waypoints;

// -----------------------
// 함수 선언부 (정의는 .cpp에 있음)
// -----------------------

// mesh 상에서 점의 face/barycentric 좌표 찾기
bool locate_face_and_point(const Kernel::Point_3 &point,
                           face_descriptor &face,
                           Surface_mesh_shortest_path::Barycentric_coordinates &location,
                           const Triangle_mesh &mesh);

// STL 파일 읽기 (필요시 <CGAL/IO/STL_reader.h>를 .cpp에서 include)
bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh);

// mesh 초기화 + AABB tree 생성 + 최대 z 반환
double initializeMeshAndGetMaxZ(const std::string &mesh_file_path,
                                Triangle_mesh &mesh, AABB_tree &tree);

// 2D 경로를 수직 투영해 mesh 위의 3D 포인트들 생성
std::vector<Point_3> projectPathOntoMesh(const std::vector<Point_3> &path_2D,
                                         AABB_tree &tree);

// 주어진 점이 속한 face 기준 normal (Eigen) 계산
Eigen::Vector3d getFaceNormal(const Point &ros_point, const Triangle_mesh &mesh);

// 형 변환 헬퍼
std::vector<Point_3> convertToCGALPoints(const std::vector<Point> &pts);
Point               convertToROSPoint(const Point_3 &p);

// 쿼터니언 -> RPY
void quaternionToRPY(double qx, double qy, double qz, double qw,
                     double &roll, double &pitch, double &yaw);

// 포인트 보간 (option: 0=보간 후 투영, 1=선형 보간, 2=가변 간격), 투영 시 AABB_tree 필요
std::vector<Point> interpolatePoints(const std::vector<Point> &points,
                                     double desired_interval,
                                     int option,
                                     AABB_tree &tree);

// SLERP
tf2::Quaternion customSlerp(const tf2::Quaternion &q1,
                            const tf2::Quaternion &q2,
                            double t);

// xyz는 선형, orientation은 SLERP로 보간
Waypoints interpolatexyzrpy(const Waypoints &input, double desired_interval);

// 파일 유틸
void clearFile(const std::string &file_path);
void saveWaypointsToFile(const Waypoints &wps, const std::string &file_path);
