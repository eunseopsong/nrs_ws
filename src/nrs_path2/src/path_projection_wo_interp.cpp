#include "path_projection.h"
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <sstream>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <iostream>

//////////////////////////////////////
// 보조 자료구조
//////////////////////////////////////

// 입력 파일 한 줄의 데이터 보관용
// - x,y : 경로 평면 좌표 (필수)
// - fx,fy,fz : force (원본에서 7,8,9열에 해당하는 값; 그대로 써야 함)
struct InputRow {
  double x;
  double y;
  double fx;
  double fy;
  double fz;
};

//////////////////////////////////////
// CGAL / Geometry 유틸
//////////////////////////////////////

// mesh 상의 한 점이 속한 face와 barycentric 좌표를 tree를 이용해 찾는다
static bool locate_face_and_point_with_tree(
    const Kernel::Point_3 &point,
    face_descriptor &face,
    CGAL::cpp11::array<double, 3> &location,
    const Triangle_mesh &mesh,
    AABB_tree &tree)
{
  if (mesh.faces().empty()) {
    std::cerr << "Mesh is empty, cannot locate face." << std::endl;
    return false;
  }

  auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, mesh);
  face = result.first;
  location = result.second;

  if (face == Triangle_mesh::null_face()) {
    std::cerr << "Failed to locate face for point: "
              << point << std::endl;
    return false;
  }
  return true;
}

// STL 메쉬를 읽고 AABB 트리를 만든 뒤, mesh 상의 최대 z값 반환
double initializeMeshAndGetMaxZ(const std::string &mesh_file_path,
                                Triangle_mesh &mesh,
                                AABB_tree &tree)
{
  std::ifstream input(mesh_file_path, std::ios::binary);
  if (!input || !CGAL::IO::read_STL(input, mesh)) {
    std::cerr << "Failed to load mesh file." << std::endl;
    std::exit(-1);
  }

  tree = AABB_tree(faces(mesh).begin(), faces(mesh).end(), mesh);
  tree.accelerate_distance_queries();

  double max_z = std::numeric_limits<double>::lowest();
  for (const auto &v : vertices(mesh)) {
    const Point_3 &p = mesh.point(v);
    if (p.z() > max_z) {
      max_z = p.z();
    }
  }
  return max_z;
}

// 입력 경로 파일을 읽는다.
// 요구사항:
//  - 행 수는 그대로 유지해야 함
//  - geodesic_waypoints.txt에는 최소 x y ... fx fy fz 가 있다고 가정
//    (즉 최소 5개 이상의 숫자. 중간에 다른 값들이 있어도 마지막 3개는 force라고 본다.)
// 반환:
//  - InputRow 목록 (x,y,fx,fy,fz)
//  - path_points_2d_zmax : 각 점을 (x, y, max_z)로 올려둔 초기 3D path (투영 전)
std::vector<InputRow> read_input_path_and_prepare_points(
    const std::string &file_path,
    double max_z,
    std::vector<Point_3> &path_points_2d_zmax)
{
  std::vector<InputRow> rows;
  path_points_2d_zmax.clear();

  std::ifstream infile(file_path);
  if (!infile.is_open()) {
    std::cerr << "Error: Unable to open file: " << file_path << std::endl;
    return rows;
  }

  std::string line;
  while (std::getline(infile, line)) {
    if (line.empty())
      continue;

    std::istringstream iss(line);
    std::vector<double> vals;
    double v;
    while (iss >> v) {
      vals.push_back(v);
    }

    // 최소한 x,y,fx,fy,fz를 뽑아야 한다.
    // 가정:
    //  - vals[0] = x
    //  - vals[1] = y
    //  - force는 마지막 3개가 fx, fy, fz
    if (vals.size() < 5) {
      std::cerr << "Invalid line (need at least x y fx fy fz): " << line << std::endl;
      continue;
    }

    double x = vals[0];
    double y = vals[1];

    double fx = vals[vals.size() - 3];
    double fy = vals[vals.size() - 2];
    double fz = vals[vals.size() - 1];

    InputRow row;
    row.x = x;
    row.y = y;
    row.fx = fx;
    row.fy = fy;
    row.fz = fz;
    rows.push_back(row);

    // 이 점을 (x, y, max_z)에 올려서 raycast 시작점으로 쓴다
    path_points_2d_zmax.emplace_back(x, y, max_z);
  }

  infile.close();
  return rows;
}

// -Z 방향(0,0,-1) 레이를 쏴서 메쉬와의 첫 번째 교점을 찾는다
// path_2D 의 각 점을 표면으로 "낙하 투영"시킨 결과를 반환
// ★중요: 입력 벡터와 동일한 길이의 벡터를 항상 반환하도록 한다.
//  -> intersection 없는 경우에는 그냥 원래 점(max_z) 그대로 넣어 행 수 유지
std::vector<Point_3> projectPathOntoMesh_sameSize(
    const std::vector<Point_3> &path_2D,
    AABB_tree &tree)
{
  std::vector<Point_3> projected_points;
  projected_points.reserve(path_2D.size());

  for (const auto &p : path_2D) {
    Ray_3 vertical_ray(p, Vector_3(0, 0, -1));
    auto intersection = tree.first_intersection(vertical_ray);

    if (intersection) {
      if (const Point_3 *proj_point = boost::get<Point_3>(&intersection->first)) {
        projected_points.push_back(*proj_point);
        continue;
      }
    }

    // 만약 교점이 없으면 그대로 (x,y,max_z) 사용해서 row 수 유지
    projected_points.push_back(p);
  }

  return projected_points;
}

// 투영된 점에서의 표면 노멀을 이용해 EE 자세 쿼터니언(q_out)을 계산
// - z축: 표면 노멀의 반대(-n) 방향
// - x축: 대략 (-1,-1,0)을 기준으로 직교화
// - y축: z × x
// 실패 시 단위 쿼터니언(0,0,0,1)
static tf2::Quaternion computeOrientationOnSurface(
    const Triangle_mesh &mesh,
    AABB_tree &tree,
    const Point &pt_ros)
{
  Kernel::Point_3 cp(pt_ros.x, pt_ros.y, pt_ros.z);

  face_descriptor face;
  CGAL::cpp11::array<double,3> loc;
  if (!locate_face_and_point_with_tree(cp, face, loc, mesh, tree)) {
    tf2::Quaternion q_ident;
    q_ident.setRPY(0,0,0);
    return q_ident;
  }

  Triangle_mesh::Halfedge_index h = mesh.halfedge(face);
  vertex_descriptor v0 = mesh.source(h);
  vertex_descriptor v1 = mesh.target(h);
  vertex_descriptor v2 = mesh.target(mesh.next(h));

  Kernel::Vector_3 n0 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v0, mesh);
  Kernel::Vector_3 n1 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v1, mesh);
  Kernel::Vector_3 n2 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v2, mesh);

  Kernel::Vector_3 n = loc[0]*n0 + loc[1]*n1 + loc[2]*n2;

  tf2::Vector3 z_axis(-n.x(), -n.y(), -n.z());
  if (z_axis.length() < 1e-12) {
    tf2::Quaternion q_ident;
    q_ident.setRPY(0,0,0);
    return q_ident;
  }
  z_axis.normalize();

  tf2::Vector3 x_axis_init(-1.0, -1.0, 0.0);
  if (x_axis_init.length() < 1e-12) {
    x_axis_init = tf2::Vector3(1.0, 0.0, 0.0);
  }
  tf2::Vector3 x_axis_dir = x_axis_init.normalized();

  tf2::Vector3 y_axis = z_axis.cross(x_axis_dir);
  if (y_axis.length() < 1e-12) {
    x_axis_init = tf2::Vector3(0.0, -1.0, 0.0);
    x_axis_dir = x_axis_init.normalized();
    y_axis = z_axis.cross(x_axis_dir);
  }
  y_axis.normalize();

  tf2::Vector3 x_axis = y_axis.cross(z_axis);
  x_axis.normalize();

  tf2::Matrix3x3 R(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());

  tf2::Quaternion q;
  R.getRotation(q);
  q.normalize();
  return q;
}

// quaternion -> RPY(rad)
void quaternionToRPY(double qx, double qy, double qz, double qw,
                     double &roll, double &pitch, double &yaw)
{
  double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (norm == 0) norm = 1;
  qw /= norm; qx /= norm; qy /= norm; qz /= norm;

  double sinr_cosp = 2.0 * (qw*qx + qy*qz);
  double cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);
  double raw_roll = std::atan2(sinr_cosp, cosr_cosp);
  roll = (raw_roll < -2.5) ? raw_roll + 2*M_PI : raw_roll;

  double sinp = 2.0 * (qw*qy - qz*qx);
  pitch = (std::fabs(sinp) >= 1)
        ? std::copysign(M_PI/2.0, sinp)
        : std::asin(sinp);

  double siny_cosp = 2.0 * (qw*qz + qx*qy);
  double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

// 파일 비우기
void clearFile(const std::string &file_path)
{
  std::ofstream file(file_path, std::ofstream::trunc);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << file_path << std::endl;
  }
}

// Waypoints를 x y z roll pitch yaw fx fy fz (9열)로 저장
// - 1~6열: projection 결과 기반 pose
// - 7~9열: 입력에서 읽은 force 그대로
void saveWaypointsToFile9cols(const Waypoints &wps,
                              const std::vector<InputRow> &rows_in,
                              const std::string &file_path)
{
  std::ofstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Error: Unable to open file " << file_path << std::endl;
    return;
  }

  // 길이 보장: proj_wps.waypoints.size() == rows_in.size()
  size_t N = std::min(wps.waypoints.size(), rows_in.size());

  for (size_t i = 0; i < N; ++i) {
    const auto &wp = wps.waypoints[i];
    const auto &rin = rows_in[i];

    double roll, pitch, yaw;
    quaternionToRPY(wp.qx, wp.qy, wp.qz, wp.qw, roll, pitch, yaw);

    file << wp.x << " " << wp.y << " " << wp.z << " "
         << roll << " " << pitch << " " << yaw << " "
         << rin.fx << " " << rin.fy << " " << rin.fz << "\n";
  }

  file.close();
  std::cout << "Waypoints (9 cols) saved to " << file_path << std::endl;
}

// rpy_file_path_ 내용을 std_msgs::msg::String 으로 publish
void sendFile(const std::string &file_path,
              const rclcpp::Publisher<String>::SharedPtr &pub,
              rclcpp::Logger logger)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open file: %s", file_path.c_str());
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string data = buffer.str();
  file.close();

  String msg;
  msg.data = data;
  RCLCPP_INFO(logger, "Sending file data (%zu bytes)...", data.size());
  pub->publish(msg);
  RCLCPP_INFO(logger, "File data sent.");
}

//////////////////////////////////////
// PathProjectionNode (ROS2 노드)
//////////////////////////////////////
class PathProjectionNode : public rclcpp::Node {
public:
  PathProjectionNode()
  : Node("path_projection_node")
  {
    // ---- 파라미터 선언 (원본 유지) ----
    desired_interval_   = this->declare_parameter<double>("desired_interval", 0.00005);
    sampling_time_      = this->declare_parameter<double>("sampling_time", 0.002);
    starting_time_      = this->declare_parameter<double>("starting_time", 3.0);
    last_resting_time_  = this->declare_parameter<double>("last_resting_time", 3.0);
    acceleration_time_  = this->declare_parameter<double>("acceleration_time", 1.0);
    density_multiplier_ = this->declare_parameter<double>("density_multiplier", 1.0);

    // mesh 파일 경로들 (원본과 동일하게 그대로 둠)
    mesh_file_path_     = this->declare_parameter<std::string>("mesh_file_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/mesh/complex_surface_1.stl");

    plane_path_file_path_ = this->declare_parameter<std::string>("plane_path_file_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/data/geodesic_waypoints.txt");

    rpy_file_path_      = this->declare_parameter<std::string>("rpy_output_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/data/final_waypoints_RPY.txt");

    visual_file_path_   = this->declare_parameter<std::string>("visual_output_path",
                              "/home/eunseop/nrs_ws/src/rtde_handarm2/data/hand_g_recording.txt");

    // ---- 파라미터 로그 ----
    RCLCPP_INFO(this->get_logger(),
                "Params | desired_interval=%.9f, density_multiplier=%.3f, sampling_time=%.6f, starting=%.3f, last_rest=%.3f, accel=%.3f",
                desired_interval_, density_multiplier_, sampling_time_,
                starting_time_, last_resting_time_, acceleration_time_);

    // ---- 퍼블리셔 ----
    using QoS = rclcpp::QoS;
    interpolated_pub_ = this->create_publisher<Waypoints>("interpolated_waypoints", QoS(10));
    file_pub_         = this->create_publisher<String>("path_publisher", QoS(10));

    // ---- 서비스 ----
    service_ = this->create_service<std_srvs::srv::Empty>(
      "projection",
      std::bind(&PathProjectionNode::executePathProjectionCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(),
                "Path projection service node ready. Waiting for service calls...");
  }

private:
  // === 멤버 변수 ===
  Triangle_mesh mesh_;
  AABB_tree tree_;

  // projection 결과 점들을 (x,y,z)로 저장
  std::vector<Point> projected_points_ros_;

  // 입력 원본 (x,y,fx,fy,fz) 전체 행
  std::vector<InputRow> input_rows_;

  std::vector<Point> clicked_points_;            // (미사용 유지)
  std::vector<Eigen::Vector3d> selected_points_; // (미사용 유지)
  bool clickedPointReceived_ = false;

  double desired_interval_;
  double sampling_time_;
  double starting_time_;
  double last_resting_time_;
  double acceleration_time_;
  double density_multiplier_;
  double time_counter_ = 0.0;
  double projection_z_ = 0.5;

  std::string mesh_file_path_;
  std::string plane_path_file_path_;
  std::string rpy_file_path_;
  std::string visual_file_path_;

  rclcpp::Publisher<Waypoints>::SharedPtr interpolated_pub_;
  rclcpp::Publisher<String>::SharedPtr    file_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;

  // === 서비스 콜백: PROJECTION ONLY + force 유지 ===
  void executePathProjectionCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(),
                "Service called: PROJECTION-ONLY with original force passthrough.");
    std::cout << "mesh data: " << mesh_file_path_ << std::endl;

    auto t0 = std::chrono::high_resolution_clock::now();

    // 1) 메쉬 로드 + AABB 트리 구성 + 표면 최고 z 계산
    projected_points_ros_.clear();
    input_rows_.clear();
    projection_z_ = initializeMeshAndGetMaxZ(mesh_file_path_, mesh_, tree_);

    // 2) 입력 경로 로드
    //    -> input_rows_: x,y,fx,fy,fz (행 수 그대로)
    //    -> path_points_2d_zmax: (x,y,max_z) 형태, 투영 전 시작점
    std::vector<Point_3> path_points_2d_zmax;
    input_rows_ = read_input_path_and_prepare_points(
        plane_path_file_path_,
        projection_z_,
        path_points_2d_zmax);

    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu rows from input file", input_rows_.size());

    // 행 수가 0이면 더 진행할 게 없음
    if (input_rows_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "No valid rows found in input file. Aborting projection.");
      return;
    }

    // 3) STL 표면으로 낙하 투영 (행 수 유지)
    std::vector<Point_3> projected_pts_3d =
        projectPathOntoMesh_sameSize(path_points_2d_zmax, tree_);

    if (projected_pts_3d.size() != input_rows_.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "Projected size (%zu) != input size (%zu). Truncating to min.",
                  projected_pts_3d.size(), input_rows_.size());
    }

    size_t N = std::min(projected_pts_3d.size(), input_rows_.size());
    projected_points_ros_.reserve(N);

    for (size_t i = 0; i < N; ++i) {
      const auto &p = projected_pts_3d[i];
      Point pr;
      pr.x = p.x();
      pr.y = p.y();
      pr.z = p.z();
      projected_points_ros_.push_back(pr);
    }

    RCLCPP_INFO(this->get_logger(),
                "Projection complete. %zu points.", projected_points_ros_.size());

    // 4) Waypoints 구성
    //    - x y z: projection 결과
    //    - roll pitch yaw: 표면 노멀 기반
    //    - fx fy fz: input_rows_에서 그대로 복사
    Waypoints proj_wps;
    proj_wps.waypoints.reserve(N);

    for (size_t i = 0; i < N; ++i) {
      const auto &pt = projected_points_ros_[i];
      const auto &rin = input_rows_[i];

      Waypoint w;
      w.x  = pt.x;
      w.y  = pt.y;
      w.z  = pt.z;

      // orientation from surface normal
      tf2::Quaternion q = computeOrientationOnSurface(mesh_, tree_, pt);
      w.qw = q.getW();
      w.qx = q.getX();
      w.qy = q.getY();
      w.qz = q.getZ();

      // force 그대로
      w.fx = rin.fx;
      w.fy = rin.fy;
      w.fz = rin.fz;

      proj_wps.waypoints.push_back(w);
    }

    // 5) 파일 저장 (두 파일에 동일하게 저장)
    //    출력 포맷: x y z roll pitch yaw fx fy fz (9열)
    clearFile(rpy_file_path_);
    saveWaypointsToFile9cols(proj_wps, input_rows_, rpy_file_path_);

    clearFile(visual_file_path_);
    saveWaypointsToFile9cols(proj_wps, input_rows_, visual_file_path_);

    // 6) publish
    interpolated_pub_->publish(proj_wps);
    sendFile(rpy_file_path_, file_pub_, this->get_logger());

    // 7) 처리시간 로깅
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count();

    std::cout << "Projection-only processing time: " << dur << " s" << std::endl;
    RCLCPP_INFO(this->get_logger(),
                "Saved %zu projected waypoints (pose from projection, force passthrough).",
                proj_wps.waypoints.size());
  }
};

//////////////////////////////////////
// main
//////////////////////////////////////
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
