#ifndef CONTACT_SENSING_HPP
#define CONTACT_SENSING_HPP

#include <rclcpp/rclcpp.hpp>
#include "stl_loader.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <chrono>

#ifndef EIGEN
#define EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#endif

struct Wrench {float at[3];};
struct WrenchAxis{Wrench r0; Wrench f;};

inline void insert_wrench(Wrench &w, float x, float y, float z) {
    w.at[0] = x;    w.at[1] = y;    w.at[2] = z;
}

class IfsAlgorithm : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub_;

  StlLoader surface_shape_;
  sensor_msgs::msg::PointCloud pc_data_;

  bool ft_measure_flag_{false};
  double force_norm{0.0};
  int cnt;

  geometry_msgs::msg::PointStamped cp_pubdata_;
  std::vector<float> ft_value_;
  std::vector<float> cp_;
  std::string filename;

  void get_WrenchAxis(std::vector<float> ft, WrenchAxis& out);
  int get_ClosestSurface(WrenchAxis r);
  float get_DistanceLinePoint(WrenchAxis r, float x, float y, float z);
  int get_NextTriangle(int idx, std::vector<int>& vert_idx);
  bool is_SurfaceContact(int idx, WrenchAxis r, std::vector<int> &vert_idx, std::vector<float>& cp);
  void ftsensor_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  bool first_draw_{true};

  void run_step(); // ROS2 타이머용 루프 함수

public:
  IfsAlgorithm(); // 생성자 변경
  bool set_ContactShape(const std::string& filename, Transformation pos);
  void draw_arrow(geometry_msgs::msg::PointStamped cp, geometry_msgs::msg::Wrench force);
  void delete_arrow(geometry_msgs::msg::PointStamped cp, geometry_msgs::msg::Wrench force);
  std::vector<float> get_ContactPoint(std::vector<float> ft);
};

#endif // CONTACT_SENSING_HPP