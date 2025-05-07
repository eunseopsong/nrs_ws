// header file Immigration (from ROS1 to ROS2; the right one is the older one)
#ifndef NRS_GEODESIC_H
#define NRS_GEODESIC_H

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <rclcpp/rclcpp.hpp>                    // #include <ros/ros.h>
#include "geometry_msgs/msg/point.hpp"          // #include <geometry_msgs/Point.h>

#include <std_msgs/msg/float64_multi_array.hpp> // #include <std_msgs/Float64MultiArray.h>

// Fail: How to include Waypoint and Waypoints
// Waypoint 메시지 타입 (ROS 2)
#include "nrs_path2/msg/waypoint.hpp"
#include "nrs_path2/msg/waypoints.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL.h> // #include <CGAL/IO/STL_reader.h> (STL_reader.h is included in STL.h)

// Vec3d etc.
#include "nrs_vec3d.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

// Pair and list of point and normal
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;

// Mesh containers
typedef std::vector<Point_3> Point_container;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Tree;
typedef boost::graph_traits<Triangle_mesh>::face_descriptor face_descriptor;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;

struct TriangleFace
{
    nrs_vec3d::Vec3d vertices[3];
    nrs_vec3d::Vec3d normal;
};

/**
 * @brief nrs_geodesic class: geodesic computation and spline/Bezier-related processing
 */
class nrs_geodesic
{
public:
    // CGAL type
    Surface_mesh_shortest_path *shortest_paths;

    nrs_vec3d n_vec3d;

    void geodesicbasecalcuation(const Eigen::Vector3d &p,
                                const Eigen::Vector3d &q,
                                Eigen::Vector3d &V_p,
                                Eigen::Vector3d &V_q,
                                double &geodesic_distance,
                                const Triangle_mesh &tmesh,
                                const std::vector<TriangleFace> &mesh);

    double calculateAngleBetweenVectors(const Eigen::Vector3d &vec1,
                                        const Eigen::Vector3d &vec2,
                                        const Eigen::Vector3d &p,
                                        const Triangle_mesh &tmesh);

    Eigen::Vector3d geodesicextend(const Eigen::Vector3d &p,
                                   const Eigen::Vector3d &q,
                                   const Eigen::Vector3d &V_q,
                                   const Triangle_mesh &tmesh,
                                   const std::vector<TriangleFace> &mesh,
                                   double angle);

    std::tuple<nrs_vec3d::Vec3d, nrs_vec3d::Vec3d> project_and_find_intersection(const nrs_vec3d::Vec3d &current_point,
                                                                                 const nrs_vec3d::Vec3d &current_direction,
                                                                                 double &distance_traveled,
                                                                                 const Triangle_mesh &tmesh,
                                                                                 const std::vector<TriangleFace> &mesh);

    Eigen::Vector3d geodesicAddVector(const Eigen::Vector3d &p,
                                      const Eigen::Vector3d &start_direction_p,
                                      double total_distance,
                                      const Eigen::Vector3d &q,
                                      const Triangle_mesh &tmesh,
                                      const std::vector<TriangleFace> &mesh);

    double computeGeodesicDistance(const Eigen::Vector3d &p0,
                                   const Eigen::Vector3d &p1,
                                   const Triangle_mesh &mesh);

    std::vector<double> calculateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points,
                                                         bool chord_length, const Triangle_mesh &tmesh);

    std::vector<TriangleFace> convertMeshToTriangleFaces(const Triangle_mesh &tmesh);

    Eigen::Vector3d geodesicSubtract(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2,
                                     const Triangle_mesh &tmesh);

    std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(const std::vector<Eigen::Vector3d> &selected_points,
                                                                 const std::vector<double> &u_values,
                                                                 const Triangle_mesh &tmesh);

    std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(const std::vector<Eigen::Vector3d> &selected_points,
                                                                         const std::vector<double> &u_values,
                                                                         const std::vector<Eigen::Vector3d> &tangent_vectors,
                                                                         const Triangle_mesh &tmesh);

    std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(const std::vector<Eigen::Vector3d> &control_points,
                                                                  const Triangle_mesh &tmesh,
                                                                  int steps);

    bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &tmesh);

    // ROS1: #include <nrs_path/Waypoints.h>
    // ROS2:
    nrs_path::msg::Waypoints ConvertToWaypoints(const std::vector<geometry_msgs::msg::Point> &points);

    nrs_path::msg::Waypoints GenerateStraightGeodesicPath(const std::vector<Eigen::Vector3d> &points, const Triangle_mesh &tmesh);

    nrs_path::msg::Waypoints GenerateHermiteSplinePath(std::vector<Eigen::Vector3d> &points, const Triangle_mesh &tmesh);

    bool load_stl_file(std::ifstream &input, Triangle_mesh &mesh);
};

#endif // NRS_GEODESIC_H