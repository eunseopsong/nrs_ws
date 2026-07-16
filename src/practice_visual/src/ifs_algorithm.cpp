#include "ifs_algorithm.hpp"

using namespace std::chrono_literals;

IfsAlgorithm::IfsAlgorithm() : Node("nonlinear_surface_contactsensing") {
  // 1. 파라미터 선언 및 가져오기 (ROS 2 필수 과정)
  this->declare_parameter<std::string>("filename", "workpiece_practice");
  this->declare_parameter<std::string>("mesh_directory", "");
  this->declare_parameter<std::vector<double>>("mesh_transformation", std::vector<double>(16, 0.0));
  this->declare_parameter<std::string>("contact_sensor_topic", "/ftsensor/measured_Cvalue");

  this->get_parameter("filename", filename);
  std::string mesh_directory;
  this->get_parameter("mesh_directory", mesh_directory);
  std::vector<double> transformation;
  this->get_parameter("mesh_transformation", transformation);
  std::string contact_sensor_topic;
  this->get_parameter("contact_sensor_topic", contact_sensor_topic);

  // 2. 퍼블리셔 / 서브스크라이버 생성
  cp_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/contactsensing/cp", 1000);
  arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/contactsensing/force_arrow", 1000);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/mesh_pc", 100);

  ftsensor_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    contact_sensor_topic, 100, std::bind(&IfsAlgorithm::ftsensor_cb, this, std::placeholders::_1));

  ft_value_.resize(6, 0.0);
  cp_.resize(3, 0.0);
  cnt = 0;

  cp_pubdata_.header.frame_id = "sensor_link";

  // 3. T Matrix 조립 (X축 기준 -90도 회전)
  Transformation T;
  
  T << 1.0,  0.0,  0.0,  0.0,
       0.0,  0.0,  1.0,  0.0,
       0.0, -1.0,  0.0,  0.0,  // 
       0.0,  0.0,  0.0,  1.0;

  set_ContactShape(mesh_directory, T);

  // 4. 타이머 생성 (ROS 1의 while(ros::ok()) 대체, 10ms마다 실행 = 100Hz)
  timer_ = this->create_wall_timer(10ms, std::bind(&IfsAlgorithm::run_step, this));
}

void IfsAlgorithm::ftsensor_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
  if(!ft_measure_flag_){
    double algor_sign = 1.0;
    double lambda = 0.05;

    force_norm = std::abs(msg->wrench.force.x) + std::abs(msg->wrench.force.y) + std::abs(msg->wrench.force.z);
    ft_value_[0] = lambda*algor_sign*msg->wrench.force.x  + (1-lambda)*algor_sign*ft_value_[0];
    ft_value_[1] = lambda*algor_sign*msg->wrench.force.y  + (1-lambda)*algor_sign*ft_value_[1];
    ft_value_[2] = lambda*algor_sign*msg->wrench.force.z  + (1-lambda)*algor_sign*ft_value_[2];
    ft_value_[3] = lambda*algor_sign*msg->wrench.torque.x + (1-lambda)*algor_sign*ft_value_[3];
    ft_value_[4] = lambda*algor_sign*msg->wrench.torque.y + (1-lambda)*algor_sign*ft_value_[4];
    ft_value_[5] = lambda*algor_sign*msg->wrench.torque.z + (1-lambda)*algor_sign*ft_value_[5];

    ft_measure_flag_ = true;
  }
}

int IfsAlgorithm::get_ClosestSurface(WrenchAxis r){
  double min_val = INFINITY;
  double dist = 0;
  int idx = 0;
  float x, y, z;
  for(int i = 0; i < surface_shape_.get_Nvertex() ; i++)  {
    surface_shape_.get_Normal(i, x, y, z);
    if ((x * r.f.at[0] + y * r.f.at[1] + z * r.f.at[2]) < 0) {
      surface_shape_.get_CenterPoint(i, x, y, z);
      dist = get_DistanceLinePoint(r, x, y, z);
      if(dist < min_val){min_val = dist; idx = i;}
    }
  }
  return idx;
}

float IfsAlgorithm::get_DistanceLinePoint(WrenchAxis r, float x, float y, float z){
  float p[3], dist[3], f_norm;
  f_norm = sqrt(r.f.at[0] * r.f.at[0] + r.f.at[1] * r.f.at[1] + r.f.at[2] * r.f.at[2]);
  p[0] = x - r.r0.at[0];  p[1] = y - r.r0.at[1];  p[2] = z - r.r0.at[2];
  dist[0] = r.f.at[1] * p[2] - r.f.at[2] * p[1];
  dist[1] = r.f.at[2] * p[0] - r.f.at[0] * p[2];
  dist[2] = r.f.at[0] * p[1] - r.f.at[1] * p[0];
  return sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2]) / f_norm;
}

bool IfsAlgorithm::set_ContactShape(const std::string& filename, Transformation pos){
  surface_shape_.STLImport(filename, pos);
  if(surface_shape_.get_Npoint()>0){
    pc_data_.header.frame_id = "sensor_link";
    for (int i = 0; i < surface_shape_.get_Npoint(); i++) {
        geometry_msgs::msg::Point32 pt;
        float x, y, z;
        surface_shape_.get_Point(i, x, y, z);
        pt.x = x; pt.y = y; pt.z = z;
        pc_data_.points.push_back(pt);
    }
    RCLCPP_INFO(this->get_logger(), "pointcloud published!");
    return true;
  } else{
    RCLCPP_ERROR(this->get_logger(), "STLloader Error: can not find STL file");
    return false;
  }
}

void IfsAlgorithm::get_WrenchAxis(std::vector<float> ft, WrenchAxis &out){
  float f_norm = sqrt(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]);
  insert_wrench(out.f, ft[0] / f_norm, ft[1] / f_norm, ft[2] / f_norm);
  if (f_norm == 0) insert_wrench(out.r0, 0, 0, 0);
  else {
      insert_wrench(out.r0, (ft[1] * ft[5] - ft[2] * ft[4]) / (f_norm*f_norm),
          (ft[2] * ft[3] - ft[0] * ft[5]) / (f_norm*f_norm), (ft[0] * ft[4] - ft[1] * ft[3]) / (f_norm*f_norm));
  }
}

bool IfsAlgorithm::is_SurfaceContact(int idx, WrenchAxis r, std::vector<int>& vert_idx, std::vector<float>& cp) {
  int f1, f2, f3;
  float* v1, * v2, * v3, * normal;
  v1 = new float[3]; v2 = new float[3]; v3 = new float[3]; normal = new float[3];

  surface_shape_.get_TriangleIndex(idx, f1, f2, f3);
  surface_shape_.get_Triangle(idx, v1, v2, v3, normal);

  float lambda = ((v1[0] * normal[0] + v1[1] * normal[1] + v1[2] * normal[2]) - (r.r0.at[0] * normal[0] + r.r0.at[1] * normal[1] + r.r0.at[2] * normal[2]))
      / (r.f.at[0] * normal[0] + r.f.at[1] * normal[1] + r.f.at[2] * normal[2]);

  float ctp[3], p1[3], p2[3], p3[3], l1[3], l2[3], l3[3];
  for(int i = 0; i < 3; i++){
    cp[i] = r.r0.at[i] + lambda * r.f.at[i];
    ctp[i] = (v1[i] + v2[i] + v3[i]) / 3;
    p1[i] = 0.5 * (v2[i] + v3[i]); p2[i] = 0.5 * (v1[i] + v3[i]); p3[i] = 0.5 * (v2[i] + v1[i]);
    l1[i] = v2[i] - v3[i]; l2[i] = v1[i] - v3[i]; l3[i] = v1[i] - v2[i];
  }

  float l1_norm = sqrt(l1[0] * l1[0] + l1[1] * l1[1] + l1[2] * l1[2]);
  float l2_norm = sqrt(l2[0] * l2[0] + l2[1] * l2[1] + l2[2] * l2[2]);
  float l3_norm = sqrt(l3[0] * l3[0] + l3[1] * l3[1] + l3[2] * l3[2]);

  for(int i = 0; i < 3; i++){
    l1[i] = l1[i] / l1_norm; l2[i] = l2[i] / l2_norm; l3[i] = l3[i] / l3_norm;
  }

  float n1[3], n2[3], n3[3];
  for(int i = 0; i < 3; i++){
    n1[i] = (ctp[i] - p1[i]) - (l1[0] * (ctp[0] - p1[0]) + l1[1] * (ctp[1] - p1[1]) + l1[2] * (ctp[2] - p1[2])) * l1[i];
    n2[i] = (ctp[i] - p2[i]) - (l2[0] * (ctp[0] - p2[0]) + l2[1] * (ctp[1] - p2[1]) + l2[2] * (ctp[2] - p2[2])) * l2[i];
    n3[i] = (ctp[i] - p3[i]) - (l3[0] * (ctp[0] - p3[0]) + l3[1] * (ctp[1] - p3[1]) + l3[2] * (ctp[2] - p3[2])) * l3[i];
  }

  int num_positive = 0;
  if ((n1[0] * (cp[0] - p1[0]) + n1[1] * (cp[1] - p1[1]) + n1[2] * (cp[2] - p1[2])) > 0) { vert_idx.push_back(f1); num_positive++; }
  if ((n2[0] * (cp[0] - p2[0]) + n2[1] * (cp[1] - p2[1]) + n2[2] * (cp[2] - p2[2])) > 0) { num_positive++; vert_idx.push_back(f2); }
  if ((n3[0] * (cp[0] - p3[0]) + n3[1] * (cp[1] - p3[1]) + n3[2] * (cp[2] - p3[2])) > 0) { num_positive++; vert_idx.push_back(f3); }

  delete[] v1; delete[] v2; delete[] v3; delete[] normal;

  if (num_positive == 3) return true;
  else return false;
}

int IfsAlgorithm::get_NextTriangle(int idx, std::vector<int>& vert_idx){
  int idx_next = idx;
  int f[3], f_idx[3];
  surface_shape_.get_TriangleIndex(idx, f_idx[0], f_idx[1], f_idx[2]);
  int count_vert = vert_idx.size();
  int size = surface_shape_.get_Nvertex();

  for (int i = 0 ; i < size; i++) {
    surface_shape_.get_TriangleIndex(i, f[0], f[1], f[2]);
    if(((f[0] == vert_idx[0])||(f[1] == vert_idx[0])||(f[2] == vert_idx[0]))&&(idx!=i)) {
        if(count_vert == 2) {
            if ((f[0] == vert_idx[1]) || (f[1] == vert_idx[1]) || (f[2] == vert_idx[1])) { idx_next = i; break; }
        }
        else if(count_vert == 1) {
            int count = 0;
            for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) if (f[j] == f_idx[k]) count++;
            if (count == 1) { idx_next = i; break; }
        }
    }
  }
  return idx_next;
}

std::vector<float> IfsAlgorithm::get_ContactPoint(std::vector<float> ft){
  bool bContact = false;
  std::vector<float> cp; cp.resize(3);
  int idx, iter = 0;

  if ((abs(ft[0]) + abs(ft[1]) + abs(ft[2]) + abs(ft[3]) + abs(ft[4]) + abs(ft[5])) == 0){
    cp[0] = 0.0;    cp[1] = 0.0;    cp[2] = 0.0;
  } else {
    WrenchAxis r;
    get_WrenchAxis(ft, r);
    idx = get_ClosestSurface(r);
    std::vector<int> vert_idx;
    while(!bContact) {
      vert_idx.clear();
      bContact = is_SurfaceContact(idx, r, vert_idx, cp);
      if(!bContact) idx = get_NextTriangle(idx, vert_idx);
      iter++;
      if(iter > 100) { cp[0] = 0.0; cp[1] = 0.0; cp[2] = 0.0; break; }
    }
  }
  return cp;
}

void IfsAlgorithm::draw_arrow(geometry_msgs::msg::PointStamped cp, geometry_msgs::msg::Wrench force){
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "sensor_link";
  arrow.header.stamp = this->now();
  arrow.ns = "contact_force";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;

  if(first_draw_){
    arrow.action = visualization_msgs::msg::Marker::ADD;
    first_draw_ = false;
  } else {
    arrow.action = visualization_msgs::msg::Marker::MODIFY;
  }

  // 색상 설정 (빨간색, 불투명)
  arrow.color.r = 1.0; 
  arrow.color.g = 0.0; 
  arrow.color.b = 0.0; 
  arrow.color.a = 1.0;

  // [수정] 스케일(Scale) 고정: x=몸통 굵기, y=머리 굵기, z=머리 길이
  arrow.scale.x = 0.002; // mm 굵기 (일정하게 유지)
  arrow.scale.y = 0.004; // mm 머리 굵기
  arrow.scale.z = 0.002; // mm 머리 길이

  double length_ratio = 0.001; // 힘(N)을 미터(m) 단위 길이로 변환하는 비율 (예: 10N -> 1cm)

  arrow.points.resize(2);

  // 시작점 (꼬리): 접촉점에서 힘 벡터의 *반대* 방향으로 뻗어나간 위치
  // (이렇게 해야 화살표가 가리키는 고유의 방향이 기존과 똑같이 유지됩니다.)
  arrow.points[0].x = cp.point.x + (force.force.x * length_ratio);
  arrow.points[0].y = cp.point.y + (force.force.y * length_ratio);
  arrow.points[0].z = cp.point.z + (force.force.z * length_ratio);

  // 끝점 (머리): 보라색 접촉점 위치에 정확히 안착
  arrow.points[1].x = cp.point.x;
  arrow.points[1].y = cp.point.y;
  arrow.points[1].z = cp.point.z;

  // [중요] 화살표를 points로 정의할 때는 pose와 orientation은 건드리지 않아야 합니다.
  arrow.pose.position.x = 0;
  arrow.pose.position.y = 0;
  arrow.pose.position.z = 0;
  arrow.pose.orientation.w = 1.0;
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = 0.0;
  arrow.pose.orientation.z = 0.0;

  arrow_pub_->publish(arrow);
}

void IfsAlgorithm::delete_arrow(geometry_msgs::msg::PointStamped cp, geometry_msgs::msg::Wrench force){
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "sensor_link";
  arrow.header.stamp = this->now();
  arrow.ns = "contact_force";
  arrow.id = 0;
  arrow.action = visualization_msgs::msg::Marker::DELETE;
  arrow_pub_->publish(arrow);
}

void IfsAlgorithm::run_step(){
    pc_data_.header.stamp = this->now();
    pointcloud_pub_->publish(pc_data_);

    if(ft_measure_flag_){
        geometry_msgs::msg::Wrench force_;

        double filtered_force_norm = std::abs(ft_value_[0]) + std::abs(ft_value_[1]) + std::abs(ft_value_[2]);

        if(filtered_force_norm > 0.5){ 
            if(cnt > 3){ 
              cp_ = get_ContactPoint(ft_value_);

              double lambda = 1;
              if (cp_pubdata_.point.x == 0.0&&cp_pubdata_.point.y == 0.0&&cp_pubdata_.point.z == 0.0){
                  cp_pubdata_.point.x = cp_[0];
                  cp_pubdata_.point.y = cp_[1];
                  cp_pubdata_.point.z = cp_[2]+0.0005;
              } else {
                  if (cp_[0] != 0.0 || cp_[1] != 0.0 || cp_[2] != 0.0){
                      cp_pubdata_.point.x = lambda*cp_[0] + (1-lambda)*cp_pubdata_.point.x;
                      cp_pubdata_.point.y = lambda*cp_[1] + (1-lambda)*cp_pubdata_.point.y;
                      cp_pubdata_.point.z = lambda*(cp_[2]+0.001) + (1-lambda)*cp_pubdata_.point.z;
                  }
              }
              force_.force.x = ft_value_[0];
              force_.force.y = ft_value_[1];
              force_.force.z = ft_value_[2];

              // RCLCPP_INFO 사용 (std::cout 대체)
              RCLCPP_INFO(this->get_logger(), "[mm] Cx: %.1f, Cy: %.1f, Cz: %.1f", cp_pubdata_.point.x * 1000.0,cp_pubdata_.point.y * 1000.0, cp_pubdata_.point.z * 1000.0);

              draw_arrow(cp_pubdata_, force_);

              //if(cp_[0] == 0.0 && cp_[1] == 0.0 && cp_[2] == 0.0){
              //    delete_arrow(cp_pubdata_, force_);
              //}
            }
            cnt++;
        } else {
            cp_[0] = 0.0; cp_[1] = 0.0; cp_[2] = 0.0;
            cp_pubdata_.point.x = 0.0; cp_pubdata_.point.y = 0.0; cp_pubdata_.point.z = 0.0;
            force_.force.x = 0.0; force_.force.y = 0.0; force_.force.z = 0.0;

            delete_arrow(cp_pubdata_, force_);
            cnt = 0;
        }

        cp_pubdata_.header.stamp = this->now();
        cp_pub_->publish(cp_pubdata_);
        ft_measure_flag_ = false;
    }
}