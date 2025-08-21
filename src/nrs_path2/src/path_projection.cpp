#include "path_projection.h"

//////////////////////////////////////
// 전방 선언 (로거 전달 안 하도록 표준출력/에러 사용)
//////////////////////////////////////

class Profiler {
protected:
  std::vector<double> time_;
  std::vector<double> data_;
  double SamplingTime_;
  double StartingTime_;
  double LastRestingTime_;
  double AccelerationTime_;
public:
  Profiler(const std::vector<double>& time,
           const std::vector<double>& data,
           double StartingTime, double LastRestingTime,
           double AccelerationTime, double SamplingTime);
  virtual std::vector<std::vector<double>> AccDecProfiling() = 0;
};

class NRSProfiler : public Profiler {
public:
  NRSProfiler(const std::vector<double>& time,
              const std::vector<double>& data,
              double StartingTime, double LastRestingTime,
              double AccelerationTime, double SamplingTime);
  std::vector<std::vector<double>> AccDecProfiling() override;
};

Profiler::Profiler(const std::vector<double> &time, const std::vector<double> &data,
                   double StartingTime, double LastRestingTime,
                   double AccelerationTime, double SamplingTime)
  : time_(time), data_(data), StartingTime_(StartingTime),
    LastRestingTime_(LastRestingTime), AccelerationTime_(AccelerationTime),
    SamplingTime_(SamplingTime) {}

NRSProfiler::NRSProfiler(const std::vector<double> &time, const std::vector<double> &data,
                         double StartingTime, double LastRestingTime,
                         double AccelerationTime, double SamplingTime)
  : Profiler(time, data, StartingTime, LastRestingTime, AccelerationTime, SamplingTime) {}

std::vector<std::vector<double>> NRSProfiler::AccDecProfiling() {
  std::vector<std::vector<double>> Final_pos_interval;
  std::vector<double> Target_velocity(time_.size(), 0.0);
  double Ti = StartingTime_;
  double Ta = AccelerationTime_;
  double Ts = SamplingTime_;
  double Tl = LastRestingTime_;
  double Tf = Ti + Ts * data_.size() + Tl;

  for (size_t j = 1; j < data_.size(); j++) {
    Target_velocity[j] = (data_[j] - data_[j - 1]) / Ts;
  }

  std::vector<double> t;
  for (double i = 0; i <= Tf; i += Ts) t.push_back(i);

  std::vector<std::vector<double>> Interpolated(t.size(), std::vector<double>(2, 0.0));
  size_t Last_flag = 0;
  for (size_t i = 0; i < t.size(); i++) {
    if (t[i] <= Ti) { Interpolated[i] = {t[i], 0}; Last_flag++; }
    else if (t[i] <= Ti + Ts * data_.size()) {
      Interpolated[i] = {t[i], Target_velocity[i - Last_flag]};
    } else { Interpolated[i] = {t[i], 0}; }
  }

  double m = Ta / Ts;
  std::vector<std::vector<double>> Final(t.size(), std::vector<double>(2, 0.0));
  Final_pos_interval.push_back({time_[0], data_[0]});

  for (size_t i = 1; i < t.size(); i++) {
    if (i <= m) {
      Final[i] = {t[i], Final[i-1][1] + (Interpolated[i][1] - Interpolated[0][1]) / (i)};
    } else {
      Final[i] = {t[i], Final[i-1][1] + (Interpolated[i][1] - Interpolated[i - (int)m][1]) / m};
    }
    Final_pos_interval.push_back({t[i], Final_pos_interval[i-1][1] + Final[i][1] * Ts});
  }
  return Final_pos_interval;
}

//////////////////////////////////////
// 유틸/헬퍼 (ROS2-agnostic)
//////////////////////////////////////

bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face,
                           Surface_mesh_shortest_path::Barycentric_coordinates &location,
                           const Triangle_mesh &mesh) {
  if (mesh.faces().empty()) {
    std::cerr << "Mesh is empty, cannot build AABB tree." << std::endl;
    return false;
  }
  AABB_tree tree(mesh.faces().begin(), mesh.faces().end(), mesh);
  tree.build();

  auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, mesh);
  face = result.first;
  location = result.second;
  if (face == Triangle_mesh::null_face()) {
    std::cerr << "Failed to locate face for point: " << point << std::endl;
    return false;
  }
  return true;
}

bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh) {
  std::vector<Kernel::Point_3> points;
  std::vector<std::array<std::size_t, 3>> triangles;
  if (!CGAL::IO::read_STL(input, points, triangles)) {
    std::cerr << "Failed to read STL file." << std::endl;
    return false;
  }

  std::map<std::size_t, vertex_descriptor> index_to_vertex;
  for (std::size_t i = 0; i < points.size(); ++i) {
    index_to_vertex[i] = mesh.add_vertex(points[i]);
  }
  for (const auto &t : triangles) {
    if (mesh.add_face(index_to_vertex[t[0]], index_to_vertex[t[1]], index_to_vertex[t[2]]) == Triangle_mesh::null_face()) {
      std::cerr << "Failed to add face." << std::endl;
      return false;
    }
  }
  std::cout << "Successfully read STL file." << std::endl;
  return true;
}

double initializeMeshAndGetMaxZ(const std::string &mesh_file_path, Triangle_mesh &mesh, AABB_tree &tree) {
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
    if (p.z() > max_z) max_z = p.z();
  }
  return max_z;
}

std::vector<Point_3> projectPathOntoMesh(const std::vector<Point_3> &path_2D, AABB_tree &tree) {
  std::vector<Point_3> projected_points;
  for (const auto &p : path_2D) {
    Ray_3 vertical_ray(p, Vector_3(0, 0, -1));
    auto intersection = tree.first_intersection(vertical_ray);
    if (intersection) {
      if (const Point_3 *proj_point = boost::get<Point_3>(&intersection->first)) {
        projected_points.push_back(*proj_point);
      }
    }
  }
  return projected_points;
}

Eigen::Vector3d getFaceNormal(const Point &ros_point, const Triangle_mesh &mesh) {
  Point_3 cgal_point(ros_point.x, ros_point.y, ros_point.z);
  face_descriptor face;
  CGAL::cpp11::array<double, 3> location;

  if (!locate_face_and_point(cgal_point, face, location, mesh)) {
    std::cerr << "Failed to locate face and point for point: [" << ros_point.x << ", " << ros_point.y << ", " << ros_point.z << "]\n";
    return Eigen::Vector3d::Zero();
  }
  Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);
  Eigen::Vector3d eigen_normal(normal.x(), normal.y(), normal.z());
  eigen_normal.normalize();
  return eigen_normal;
}

std::vector<Point_3> convertToCGALPoints(const std::vector<Point> &pts) {
  std::vector<Point_3> cgal_points;
  cgal_points.reserve(pts.size());
  for (const auto &p : pts) cgal_points.emplace_back(p.x, p.y, p.z);
  return cgal_points;
}

Point convertToROSPoint(const Point_3 &p) {
  Point pt;
  pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
  return pt;
}

void quaternionToRPY(double qx, double qy, double qz, double qw,
                     double &roll, double &pitch, double &yaw) {
  double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (norm == 0) norm = 1;
  qw /= norm; qx /= norm; qy /= norm; qz /= norm;

  double sinr_cosp = 2.0 * (qw*qx + qy*qz);
  double cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);
  double raw_roll = std::atan2(sinr_cosp, cosr_cosp);
  roll = (raw_roll < -2.5) ? raw_roll + 2*M_PI : raw_roll;

  double sinp = 2.0 * (qw*qy - qz*qx);
  pitch = (std::fabs(sinp) >= 1) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);

  double siny_cosp = 2.0 * (qw*qz + qx*qy);
  double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

std::vector<Point> interpolatePoints(
    const std::vector<Point> &points,
    double desired_interval,
    int option,
    AABB_tree &tree)
{
  std::vector<Point> interpolated_points;
  std::vector<double> cumulative_distances(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    Eigen::Vector3d p0(points[i-1].x, points[i-1].y, points[i-1].z);
    Eigen::Vector3d p1(points[i].x, points[i].y, points[i].z);
    cumulative_distances[i] = cumulative_distances[i-1] + (p1 - p0).norm();
  }

  auto lin_interp_block = [&](double interval) {
    double current_distance = 0.0;
    size_t j = 1;
    while (j < points.size()) {
      if (cumulative_distances[j] >= current_distance + interval) {
        double t = (current_distance + interval - cumulative_distances[j-1]) /
                   (cumulative_distances[j] - cumulative_distances[j-1]);
        Point ip;
        ip.x = points[j-1].x + t*(points[j].x - points[j-1].x);
        ip.y = points[j-1].y + t*(points[j].y - points[j-1].y);
        ip.z = points[j-1].z + t*(points[j].z - points[j-1].z);
        interpolated_points.push_back(ip);
        current_distance += interval;
      } else { ++j; }
    }
  };

  if (option == 0) {
    lin_interp_block(desired_interval);
    // project to mesh
    std::vector<Point_3> cgal_pts = convertToCGALPoints(interpolated_points);
    std::vector<Point_3> proj = projectPathOntoMesh(cgal_pts, tree);
    std::vector<Point> final_pts; final_pts.reserve(proj.size());
    for (const auto &p : proj) final_pts.push_back(convertToROSPoint(p));
    return final_pts;
  } else if (option == 1) {
    lin_interp_block(desired_interval);
    return interpolated_points;
  } else { // option == 2 variable interval
    double total_distance = cumulative_distances.back();
    double transition_length = 0.03;
    auto computeVariableInterval = [&](double current_distance) -> double {
      if (current_distance < transition_length) {
        double scale = 0.5 * (1 - std::cos((current_distance / transition_length) * M_PI));
        return desired_interval/6.0 + scale * (desired_interval - desired_interval/6.0);
      } else if (current_distance > total_distance - transition_length) {
        double remaining_distance = total_distance - current_distance;
        double scale = 0.5 * (1 - std::cos((remaining_distance / transition_length) * M_PI));
        return desired_interval/6.0 + scale * (desired_interval - desired_interval/6.0);
      } else return desired_interval;
    };

    double current_distance = 0.0;
    size_t j = 1;
    while (j < points.size()) {
      double current_interval = computeVariableInterval(current_distance);
      if (cumulative_distances[j] >= current_distance + current_interval) {
        double t = (current_distance + current_interval - cumulative_distances[j-1]) /
                   (cumulative_distances[j] - cumulative_distances[j-1]);

        Point ip;
        ip.x = points[j-1].x + t*(points[j].x - points[j-1].x);
        ip.y = points[j-1].y + t*(points[j].y - points[j-1].y);
        ip.z = points[j-1].z + t*(points[j].z - points[j-1].z);
        interpolated_points.push_back(ip);
        current_distance += current_interval;
      } else { ++j; }
    }
    return interpolated_points;
  }
}

tf2::Quaternion customSlerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, double t) {
  double dot = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() + q1.w()*q2.w();
  tf2::Quaternion q2_copy = q2;
  if (dot < 0.0) { q2_copy = q2_copy.operator-(); dot = -dot; }
  if (dot > 0.9995) {
    tf2::Quaternion result = q1 * (1.0 - t) + q2_copy * t;
    result.normalize(); return result;
  }
  double theta_0 = std::acos(dot);
  double theta = theta_0 * t;
  double sin_theta = std::sin(theta);
  double sin_theta_0 = std::sin(theta_0);

  double s1 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  double s2 = sin_theta / sin_theta_0;
  tf2::Quaternion result = (q1 * s1) + (q2_copy * s2);
  result.normalize();
  return result;
}

Waypoints interpolatexyzrpy(const Waypoints &input, double desired_interval) {
  Waypoints output;
  if (input.waypoints.empty()) return output;

  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);
  for (size_t i = 1; i < input.waypoints.size(); i++) {
    const auto &p0 = input.waypoints[i-1];
    const auto &p1 = input.waypoints[i];
    double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
    double seg = std::sqrt(dx*dx + dy*dy + dz*dz);
    cumulative_distances.push_back(cumulative_distances.back() + seg);
  }
  double total_distance = cumulative_distances.back();

  for (double d = 0.0; d <= total_distance; d += desired_interval) {
    size_t i = 1;
    while (i < cumulative_distances.size() && cumulative_distances[i] < d) i++;
    double t = (d - cumulative_distances[i-1]) / (cumulative_distances[i] - cumulative_distances[i-1]);

    const auto &p0 = input.waypoints[i-1];
    const auto &p1 = input.waypoints[i];

    Waypoint w;
    w.x = p0.x + t*(p1.x - p0.x);
    w.y = p0.y + t*(p1.y - p0.y);
    w.z = p0.z + t*(p1.z - p0.z);

    w.fx = p0.fx + t*(p1.fx - p0.fx);
    w.fy = p0.fy + t*(p1.fy - p0.fy);
    w.fz = p0.fz + t*(p1.fz - p0.fz);

    tf2::Quaternion q0(p0.qx, p0.qy, p0.qz, p0.qw);
    tf2::Quaternion q1(p1.qx, p1.qy, p1.qz, p1.qw);
    tf2::Quaternion qi = customSlerp(q0, q1, t);
    w.qw = qi.getW(); w.qx = qi.getX(); w.qy = qi.getY(); w.qz = qi.getZ();

    output.waypoints.push_back(w);
  }
  return output;
}

void clearFile(const std::string &file_path) {
  std::ofstream file(file_path, std::ofstream::trunc);
  if (!file.is_open()) std::cerr << "Failed to open file: " << file_path << std::endl;
}

void saveWaypointsToFile(const Waypoints &wps, const std::string &file_path) {
  std::ofstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Error: Unable to open file " << file_path << std::endl;
    return;
  }
  for (const auto &wp : wps.waypoints) {
    double roll, pitch, yaw;
    quaternionToRPY(wp.qx, wp.qy, wp.qz, wp.qw, roll, pitch, yaw);
    file << wp.x << " " << wp.y << " " << wp.z << " "
         << roll << " " << pitch << " " << yaw << " "
         << wp.fx << " " << wp.fy << " " << wp.fz << "\n";
  }
  file.close();
  std::cout << "Waypoints saved to " << file_path << std::endl;
}

//////////////////////////////////////
// Node 클래스
//////////////////////////////////////
class PathProjectionNode : public rclcpp::Node {
public:
  PathProjectionNode()
  : Node("path_projection_node")
  {
    // ---- 파라미터(필요 시 declare_parameter로 외부에서 바꿀 수 있도록) ----
    desired_interval_   = this->declare_parameter<double>("desired_interval", 0.00003);
    sampling_time_      = this->declare_parameter<double>("sampling_time", 0.002);
    starting_time_      = this->declare_parameter<double>("starting_time", 3.0);
    last_resting_time_  = this->declare_parameter<double>("last_resting_time", 3.0);
    acceleration_time_  = this->declare_parameter<double>("acceleration_time", 1.0);
    mesh_file_path_     = this->declare_parameter<std::string>("mesh_file_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/mesh/workpiece.stl");
    plane_path_file_path_ = this->declare_parameter<std::string>("plane_path_file_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/data/Ori_path_transformed.txt");
    rpy_file_path_      = this->declare_parameter<std::string>("rpy_output_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/data/final_waypoints_RPY.txt");   // 필요시 수정
    visual_file_path_   = this->declare_parameter<std::string>("visual_output_path",
                              "/home/eunseop/nrs_ws/src/nrs_path2/data/visual_final_waypoints.txt");

    // ---- 퍼블리셔 ----
    using QoS = rclcpp::QoS;
    interpolated_pub_ = this->create_publisher<Waypoints>("interpolated_waypoints", QoS(10));
    file_pub_         = this->create_publisher<String>("path_publisher", QoS(10));

    // ---- 서비스 ----
    service_ = this->create_service<std_srvs::srv::Empty>(
      "projection",
      std::bind(&PathProjectionNode::executePathProjectionCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Path projection service node ready. Waiting for service calls...");
  }

private:
  // 멤버 변수
  Triangle_mesh mesh_;
  AABB_tree tree_;
  std::vector<Point> original_points_;
  std::vector<Point> clicked_points_;            // (미사용) 필요 시 ROS2 subscriber로 연결
  std::vector<Eigen::Vector3d> selected_points_; // (미사용) 클릭 포인트 투영 저장
  bool clickedPointReceived_ = false;

  double desired_interval_;
  double sampling_time_;
  double starting_time_;
  double last_resting_time_;
  double acceleration_time_;
  double time_counter_ = 0.0;
  double projection_z_ = 0.5;

  std::string mesh_file_path_;
  std::string plane_path_file_path_;
  std::string rpy_file_path_;
  std::string visual_file_path_;

  rclcpp::Publisher<Waypoints>::SharedPtr interpolated_pub_;
  rclcpp::Publisher<String>::SharedPtr    file_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;

  // === 세그먼트 생성 (approach/retreat/home) ===
  std::vector<Point> generate_segment(std::vector<Point> &orig, int option, const Triangle_mesh &mesh) {
    Point start_point = orig.front();
    Point end_point   = orig.back();
    Eigen::Vector3d start_normal = getFaceNormal(start_point, mesh);
    Eigen::Vector3d end_normal   = getFaceNormal(end_point, mesh);

    Point start_approach, end_retreat, home_position;
    if (option == 1) {
      start_approach.x = start_point.x + 0.1 * start_normal.x();
      start_approach.y = start_point.y + 0.1 * start_normal.y();
      start_approach.z = start_point.z + 0.1 * start_normal.z();
      return std::vector<Point>{start_approach, start_point};
    } else if (option == 2) {
      end_retreat.x = end_point.x + 0.1 * end_normal.x();
      end_retreat.y = end_point.y + 0.1 * end_normal.y();
      end_retreat.z = end_point.z + 0.1 * end_normal.z();
      return std::vector<Point>{end_point, end_retreat};
    } else { // option == 3 home
      end_retreat.x = end_point.x + 0.1 * end_normal.x();
      end_retreat.y = end_point.y + 0.1 * end_normal.y();
      end_retreat.z = end_point.z + 0.1 * end_normal.z();
      home_position.x = 0.568; home_position.y = 0.318; home_position.z = 0.326;
      return std::vector<Point>{end_retreat, home_position};
    }
  }

  // === 점열 → Waypoints 변환(옵션별 자세 계산 포함) ===
  Waypoints convertToWaypoints(const std::vector<Point> &points,
                               const std::vector<Point> &reference_points,
                               const Triangle_mesh &mesh, int option)
  {
    Waypoints final_wps;

    Point_3 start_point(reference_points.begin()->x, reference_points.begin()->y, reference_points.begin()->z);
    Point_3 end_point(reference_points.back().x, reference_points.back().y, reference_points.back().z);

    face_descriptor start_face, end_face;
    CGAL::cpp11::array<double,3> start_location, end_location;
    locate_face_and_point(start_point, start_face, start_location, mesh);
    locate_face_and_point(end_point,   end_face,   end_location,   mesh);

    Kernel::Vector_3 start_normal = CGAL::Polygon_mesh_processing::compute_face_normal(start_face, mesh);
    Kernel::Vector_3 end_normal   = CGAL::Polygon_mesh_processing::compute_face_normal(end_face,   mesh);

    auto pack_with_fixed_normal = [&](const std::vector<Point>& pts, const Kernel::Vector_3& n){
      tf2::Vector3 z_axis(-n.x(), -n.y(), -n.z()); z_axis.normalize();
      tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
      tf2::Vector3 y_axis = z_axis.cross(x_axis.normalized());
      x_axis = y_axis.cross(z_axis).normalized();

      tf2::Matrix3x3 R(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());
      tf2::Quaternion q; R.getRotation(q);

      for (const auto& p: pts) {
        Waypoint w;
        w.x = p.x; w.y = p.y; w.z = p.z;
        w.qw = q.getW(); w.qx = q.getX(); w.qy = q.getY(); w.qz = q.getZ();
        final_wps.waypoints.push_back(w);
      }
    };

    if (option == 1) {        // approach: start normal
      pack_with_fixed_normal(points, start_normal);
    } else if (option == 3) { // retreat: end normal
      pack_with_fixed_normal(points, end_normal);
    } else if (option == 4) { // home:   end normal
      pack_with_fixed_normal(points, end_normal);
    } else if (option == 2) { // original: face/vertex normal 보간
      for (const auto &p : points) {
        Kernel::Point_3 cp(p.x, p.y, p.z);
        face_descriptor face;
        CGAL::cpp11::array<double,3> loc;
        if (!locate_face_and_point(cp, face, loc, mesh)) {
          std::cerr << "Failed to locate face for point\n";
          continue;
        }
        Triangle_mesh::Halfedge_index h = mesh.halfedge(face);
        vertex_descriptor v0 = mesh.source(h);
        vertex_descriptor v1 = mesh.target(h);
        vertex_descriptor v2 = mesh.target(mesh.next(h));

        Kernel::Vector_3 n0 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v0, mesh);
        Kernel::Vector_3 n1 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v1, mesh);
        Kernel::Vector_3 n2 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v2, mesh);
        Kernel::Vector_3 n = loc[0]*n0 + loc[1]*n1 + loc[2]*n2;

        tf2::Vector3 z_axis(-n.x(), -n.y(), -n.z()); z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis.normalized());
        x_axis = y_axis.cross(z_axis).normalized();

        tf2::Matrix3x3 R(
          x_axis.x(), y_axis.x(), z_axis.x(),
          x_axis.y(), y_axis.y(), z_axis.y(),
          x_axis.z(), y_axis.z(), z_axis.z());
        tf2::Quaternion q; R.getRotation(q);

        Waypoint w;
        w.x = p.x; w.y = p.y; w.z = p.z;
        w.qw = q.getW(); w.qx = q.getX(); w.qy = q.getY(); w.qz = q.getZ();
        w.fx = 0.0; w.fy = 0.0; w.fz = 10.0;
        final_wps.waypoints.push_back(w);
      }
    }
    return final_wps;
    }

  std::vector<Point_3> readpathfile(const std::string &file_path, double &max_z, double desired_interval) {
    std::vector<Point_3> orig;
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
      std::cerr << "Error: Unable to open file: " << file_path << std::endl;
      return orig;
    }
    std::string line;
    while (std::getline(infile, line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      double x, y;
      if (!(iss >> x >> y)) {
        std::cerr << "Invalid line: " << line << std::endl;
        continue;
      }
      orig.emplace_back(x, y, max_z);
    }
    infile.close();
    if (orig.size() < 2) return orig;

    std::vector<double> cum; cum.push_back(0.0);
    for (size_t i=1; i<orig.size(); ++i) {
      double dx=orig[i].x()-orig[i-1].x(), dy=orig[i].y()-orig[i-1].y(), dz=orig[i].z()-orig[i-1].z();
      double d = std::sqrt(dx*dx+dy*dy+dz*dz);
      cum.push_back(cum.back()+d);
    }
    double total = cum.back();

    std::vector<Point_3> interp;
    interp.push_back(orig.front());
    double cur = desired_interval;
    size_t seg = 1;
    while (cur < total && seg < orig.size()) {
      while (seg < orig.size() && cum[seg] < cur) seg++;
      if (seg >= orig.size()) break;
      double d1=cum[seg-1], d2=cum[seg];
      double t=(cur-d1)/(d2-d1);
      double xi = orig[seg-1].x() + t*(orig[seg].x()-orig[seg-1].x());
      double yi = orig[seg-1].y() + t*(orig[seg].y()-orig[seg-1].y());
      double zi = max_z;
      interp.emplace_back(xi, yi, zi);
      cur += desired_interval;
    }
    if (interp.back() != orig.back()) interp.push_back(orig.back());
    return interp;
  }

  Waypoints ACCProfiling(const Waypoints &in,
                         double &sampling_time, double &starting_time,
                         double &last_resting_time, double &acceleration_time,
                         double &time_counter)
  {
    Waypoints out;
    std::vector<double> t, x, y, z, qw, qx, qy, qz;
    for (size_t i=0;i<in.waypoints.size();++i) {
      t.push_back(time_counter * sampling_time);
      x.push_back(in.waypoints[i].x);
      y.push_back(in.waypoints[i].y);
      z.push_back(in.waypoints[i].z);
      qw.push_back(in.waypoints[i].qw);
      qx.push_back(in.waypoints[i].qx);
      qy.push_back(in.waypoints[i].qy);
      qz.push_back(in.waypoints[i].qz);
      time_counter++;
    }

    NRSProfiler px(t,x, starting_time,last_resting_time,acceleration_time,sampling_time);
    NRSProfiler py(t,y, starting_time,last_resting_time,acceleration_time,sampling_time);
    NRSProfiler pz(t,z, starting_time,last_resting_time,acceleration_time,sampling_time);
    NRSProfiler pqw(t,qw, starting_time,last_resting_time,acceleration_time,sampling_time);
    NRSProfiler pqx(t,qx, starting_time,last_resting_time,acceleration_time,sampling_time);
    NRSProfiler pqy(t,qy, starting_time,last_resting_time,acceleration_time,sampling_time);
    NRSProfiler pqz(t,qz, starting_time,last_resting_time,acceleration_time,sampling_time);

    auto xr = px.AccDecProfiling();
    auto yr = py.AccDecProfiling();
    auto zr = pz.AccDecProfiling();
    auto qwr= pqw.AccDecProfiling();
    auto qxr= pqx.AccDecProfiling();
    auto qyr= pqy.AccDecProfiling();
    auto qzr= pqz.AccDecProfiling();

    for (size_t i=0;i<xr.size();++i) {
      Waypoint w;
      w.x = xr[i][1]; w.y = yr[i][1]; w.z = zr[i][1];
      w.qw = qwr[i][1]; w.qx = qxr[i][1]; w.qy = qyr[i][1]; w.qz = qzr[i][1];
      double norm = std::sqrt(w.qw*w.qw + w.qx*w.qx + w.qy*w.qy + w.qz*w.qz);
      if (norm > 0) { w.qw/=norm; w.qx/=norm; w.qy/=norm; w.qz/=norm; }
      out.waypoints.push_back(w);
    }
    return out;
  }

  void sendFile(const std::string &file_path, const rclcpp::Publisher<String>::SharedPtr &pub) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
      return;
    }
    std::stringstream buffer; buffer << file.rdbuf(); std::string data = buffer.str();
    file.close();

    String msg; msg.data = data;
    RCLCPP_INFO(this->get_logger(), "Sending file data (%zu bytes)...", data.size());
    pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "File data sent.");
  }

  // === 서비스 콜백 (ROS2 시그니처) ===
  void executePathProjectionCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Service called: Executing path projection and waypoint generation.");
    std::cout << "mesh data: " << mesh_file_path_ << std::endl;

    original_points_.clear();
    projection_z_ = initializeMeshAndGetMaxZ(mesh_file_path_, mesh_, tree_);

    std::vector<Point_3> path_2D = readpathfile(plane_path_file_path_, projection_z_, 0.001);
    RCLCPP_INFO(this->get_logger(), "Number of original points: %zu", path_2D.size());

    std::vector<Point_3> projected_points = projectPathOntoMesh(path_2D, tree_);
    RCLCPP_INFO(this->get_logger(), "Path projection complete. Number of projected points: %zu", projected_points.size());

    auto t0 = std::chrono::high_resolution_clock::now();
    for (const auto &p : projected_points) {
      Point rp; rp.x = p.x(); rp.y = p.y(); rp.z = p.z();
      original_points_.push_back(rp);
    }

    // approach / retreat / home
    std::vector<Point> approach_segment = generate_segment(original_points_, 1, mesh_);
    std::vector<Point> retreat_segment  = generate_segment(original_points_, 2, mesh_);
    std::vector<Point> home_segment     = generate_segment(original_points_, 3, mesh_);

    // 시각화용 보간 (option=1: 위치 선형 보간)
    std::vector<Point> visual_approach_interpolated = interpolatePoints(approach_segment, 0.001, 1, tree_);
    std::vector<Point> visual_original_interpolated = interpolatePoints(original_points_, 0.001, 1, tree_);
    std::vector<Point> visual_retreat_interpolated  = interpolatePoints(retreat_segment, 0.001, 1, tree_);

    Waypoints visual_approach_wps = convertToWaypoints(visual_approach_interpolated, original_points_, mesh_, 1);
    Waypoints visual_original_wps = convertToWaypoints(visual_original_interpolated, original_points_, mesh_, 2);
    Waypoints visual_retreat_wps  = convertToWaypoints(visual_retreat_interpolated,  original_points_, mesh_, 3);

    Waypoints visual_final;
    visual_final.waypoints.insert(visual_final.waypoints.end(),
                                  visual_approach_wps.waypoints.begin(), visual_approach_wps.waypoints.end());
    visual_final.waypoints.insert(visual_final.waypoints.end(),
                                  visual_original_wps.waypoints.begin(), visual_original_wps.waypoints.end());
    visual_final.waypoints.insert(visual_final.waypoints.end(),
                                  visual_retreat_wps.waypoints.begin(), visual_retreat_wps.waypoints.end());

    double accel_for_visual = 0.05;
    visual_final = ACCProfiling(visual_final, sampling_time_, starting_time_, last_resting_time_, accel_for_visual, time_counter_);

    clearFile(visual_file_path_);
    saveWaypointsToFile(visual_final, visual_file_path_);

    // 제어용 보간 (option=2: 가변 간격)
    double interval = 0.00005;
    std::vector<Point> control_approach_interpolated = interpolatePoints(approach_segment, interval, 2, tree_);
    std::vector<Point> control_original_interpolated = interpolatePoints(original_points_, 0.001, 2, tree_);
    std::vector<Point> control_retreat_interpolated  = interpolatePoints(retreat_segment, interval, 2, tree_);
    std::vector<Point> control_home_interpolated     = interpolatePoints(home_segment, interval, 2, tree_);

    Waypoints control_approach_wps = convertToWaypoints(control_approach_interpolated, control_original_interpolated, mesh_, 1);
    Waypoints control_original_wps = convertToWaypoints(control_original_interpolated, control_original_interpolated, mesh_, 2);
    Waypoints control_retreat_wps  = convertToWaypoints(control_retreat_interpolated,  control_original_interpolated, mesh_, 3);
    Waypoints control_home_wps     = convertToWaypoints(control_home_interpolated,     control_original_interpolated, mesh_, 4);

    Waypoints control_final;
    control_final.waypoints.insert(control_final.waypoints.end(),
                                   control_approach_wps.waypoints.begin(), control_approach_wps.waypoints.end());
    control_final.waypoints.insert(control_final.waypoints.end(),
                                   control_original_wps.waypoints.begin(), control_original_wps.waypoints.end());
    control_final.waypoints.insert(control_final.waypoints.end(),
                                   control_retreat_wps.waypoints.begin(), control_retreat_wps.waypoints.end());
    control_final.waypoints.insert(control_final.waypoints.end(),
                                   control_home_wps.waypoints.begin(), control_home_wps.waypoints.end());

    control_final = interpolatexyzrpy(control_final, desired_interval_);

    int approach_size = static_cast<int>(control_approach_wps.waypoints.size());
    int original_size = static_cast<int>(control_original_wps.waypoints.size());

    clearFile(rpy_file_path_);
    saveWaypointsToFile(control_final, rpy_file_path_);

    interpolated_pub_->publish(control_final);

    RCLCPP_INFO(this->get_logger(), "original_size: %d", original_size);
    RCLCPP_INFO(this->get_logger(), "Saved %zu final waypoints", control_final.waypoints.size());

    // 파일 전체 내용을 String으로 퍼블리시
    sendFile(rpy_file_path_, file_pub_);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count();
    std::cout << "Interpolation & Normal smoothing time: " << dur << " s" << std::endl;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
