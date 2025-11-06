#include "FT_Processing.hpp"
#include <cmath>
#include <iostream>

FT_processing::FT_processing(std::shared_ptr<rclcpp::Node> node,
                             double Ts,
                             unsigned char& HandleID_,
                             unsigned char& ContactID_,
                             bool HaccSwitch_,
                             bool CaccSwitch_)
: node_(node),
  NRS_FTSensor(HandleID_, ContactID_, HaccSwitch_, CaccSwitch_),
  Ts_(Ts),
  HaccSwitch(HaccSwitch_),
  CaccSwitch(CaccSwitch_),
  movF(3, NRS_MovFilter(Mov_num)),
  movM(3, NRS_MovFilter(Mov_num)),
  movCF(3, NRS_MovFilter(Mov_num)),
  movCM(3, NRS_MovFilter(Mov_num)),
  LPF_F(3, NRS_FreqFilter(Ts_)),
  LPF_M(3, NRS_FreqFilter(Ts_)),
  LPF_CF(3, NRS_FreqFilter(Ts_)),
  LPF_CM(3, NRS_FreqFilter(Ts_)),
  BSF_F(3, NRS_FreqFilter(Ts_)),
  BSF_M(3, NRS_FreqFilter(Ts_)),
  BSF_CF(3, NRS_FreqFilter(Ts_)),
  BSF_CM(3, NRS_FreqFilter(Ts_))
{
  // 파라미터 읽기
  if (!node_->get_parameter("AFT80IP", YamlString_IP)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find AFT80IP!");
    YamlString_IP = "192.168.0.42";
  }
  if (!node_->get_parameter("Data1_path", YamlData1_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find Data1_path!");
    YamlData1_path = "/tmp/data1.txt";
  }
  if (!node_->get_parameter("Data1_switch", YamlData1_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find Data1_switch!");
    YamlData1_switch = 0;
  }
  if (!node_->get_parameter("Print_switch", YamlPrint_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find Print_switch!");
    YamlPrint_switch = 0;
  }

  // 여기 4개가 문제였음 → int64 벡터로 받아서 int로 옮긴다
  {
    std::vector<int64_t> tmp;
    if (node_->get_parameter("Handle_Sensor_Order", tmp)) {
      H_sen_order.assign(tmp.begin(), tmp.end());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Can't find Handle_Sensor_Order!");
    }
  }
  {
    std::vector<int64_t> tmp;
    if (node_->get_parameter("Handle_Sensor_sign", tmp)) {
      H_sen_sign.assign(tmp.begin(), tmp.end());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Can't find Handle_Sensor_sign!");
    }
  }
  {
    std::vector<int64_t> tmp;
    if (node_->get_parameter("Contact_Sensor_Order", tmp)) {
      C_sen_order.assign(tmp.begin(), tmp.end());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Can't find Contact_Sensor_Order!");
    }
  }
  {
    std::vector<int64_t> tmp;
    if (node_->get_parameter("Contact_Sensor_sign", tmp)) {
      C_sen_sign.assign(tmp.begin(), tmp.end());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Can't find Contact_Sensor_sign!");
    }
  }

  if (!node_->get_parameter("Hmov_switch", Hmov_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find Hmov_switch!");
    Hmov_switch = false;
  }
  if (!node_->get_parameter("HLPF_switch", HLPF_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find HLPF_switch!");
    HLPF_switch = false;
  }
  if (!node_->get_parameter("HBSF_switch", HBSF_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find HBSF_switch!");
    HBSF_switch = false;
  }

  if (!node_->get_parameter("Cmov_switch", Cmov_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find Cmov_switch!");
    Cmov_switch = false;
  }
  if (!node_->get_parameter("CLPF_switch", CLPF_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find CLPF_switch!");
    CLPF_switch = false;
  }
  if (!node_->get_parameter("CBSF_switch", CBSF_switch)) {
    RCLCPP_ERROR(node_->get_logger(), "Can't find CBSF_switch!");
    CBSF_switch = false;
  }

  /* ROS2 publisher init*/
  ftsensor_pub_ = node_->create_publisher<geometry_msgs::msg::Wrench>("/ftsensor/measured_Hvalue", 10);
  Cftsensor_pub_ = node_->create_publisher<geometry_msgs::msg::Wrench>("/ftsensor/measured_Cvalue", 10);

  // 커스텀 msg 대신 표준
  vive_force_pub_  = node_->create_publisher<geometry_msgs::msg::Vector3>("vive_force", 10);
  vive_moment_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("vive_moment", 10);
  vive_acc_pub_    = node_->create_publisher<std_msgs::msg::Float64MultiArray>("vive_acc", 10);

  aidinGui_statePub = node_->create_publisher<std_msgs::msg::String>("Aidin_State_Text", 20);

  /* ROS2 Service init*/
  Aidin_gui_srv5 = node_->create_service<std_srvs::srv::Empty>(
    "sensor_zeroset",
    std::bind(&FT_processing::SRV5_Handle, this, std::placeholders::_1, std::placeholders::_2));

  CAN_sampling = Ts_;
}

FT_processing::~FT_processing()
{
  RCLCPP_ERROR(node_->get_logger(), "FT_processing was terminated");
  if (Data1_txt) {
    fclose(Data1_txt);
    Data1_txt = nullptr;
  }
}

void FT_processing::FT_init(int sen_init_num)
{
  if (YamlData1_switch == 1)
  {
    Data1_txt = fopen(YamlData1_path.c_str(), "wt");
  }

  char *AFT80_IP = const_cast<char *>(YamlString_IP.c_str());
  TCP_init(AFT80_IP, 4001);

  for (int i = 0; i < 3; i++)
  {
    LPF_F[i].LPF_cutF  = LPF_cutF;
    LPF_M[i].LPF_cutF  = LPF_cutF;
    LPF_CF[i].LPF_cutF = CLPF_cutF;
    LPF_CM[i].LPF_cutF = CLPF_cutF;

    BSF_F[i].BSF_cutF  = BSF_cutF;
    BSF_M[i].BSF_cutF  = BSF_cutF;
    BSF_CF[i].BSF_cutF = CBSF_cutF;
    BSF_CM[i].BSF_cutF = CBSF_cutF;

    BSF_F[i].BSF_BW  = BSF_BW;
    BSF_M[i].BSF_BW  = BSF_BW;
    BSF_CF[i].BSF_BW = CBSF_BW;
    BSF_CM[i].BSF_BW = CBSF_BW;
  }
  init_average_num = sen_init_num;
}

void FT_processing::FT_filtering()
{
  for (int i = 0; i < 3; i++)
  {
    if (Hmov_switch)
    {
      Force_val[i]  = movF[i].MovFilter(Force_val[i]);
      Moment_val[i] = movM[i].MovFilter(Moment_val[i]);
    }
    if (Cmov_switch)
    {
      Contact_Force_val[i]  = movCF[i].MovFilter(Contact_Force_val[i]);
      Contact_Moment_val[i] = movCM[i].MovFilter(Contact_Moment_val[i]);
    }

    if (HLPF_switch)
    {
      Force_val[i]  = LPF_F[i].LPF(Force_val[i]);
      Moment_val[i] = LPF_M[i].LPF(Moment_val[i]);
    }
    if (CLPF_switch)
    {
      Contact_Force_val[i]  = LPF_CF[i].LPF(Contact_Force_val[i]);
      Contact_Moment_val[i] = LPF_CM[i].LPF(Contact_Moment_val[i]);
    }

    if (HBSF_switch)
    {
      Force_val[i]  = BSF_F[i].BSF(Force_val[i]);
      Moment_val[i] = BSF_M[i].BSF(Moment_val[i]);
    }
    if (CBSF_switch)
    {
      Contact_Force_val[i]  = BSF_CF[i].BSF(Contact_Force_val[i]);
      Contact_Moment_val[i] = BSF_CM[i].BSF(Contact_Moment_val[i]);
    }
  }
}

void FT_processing::FT_publish()
{
  pub_data.force.x  = Force_val[0];
  pub_data.force.y  = Force_val[1];
  pub_data.force.z  = Force_val[2];
  pub_data.torque.x = Moment_val[0];
  pub_data.torque.y = Moment_val[1];
  pub_data.torque.z = Moment_val[2];
  ftsensor_pub_->publish(pub_data);

  Cpub_data.force.x  = Contact_Force_val[0];
  Cpub_data.force.y  = Contact_Force_val[1];
  Cpub_data.force.z  = Contact_Force_val[2];
  Cpub_data.torque.x = Contact_Moment_val[0];
  Cpub_data.torque.y = Contact_Moment_val[1];
  Cpub_data.torque.z = Contact_Moment_val[2];
  Cftsensor_pub_->publish(Cpub_data);

  geometry_msgs::msg::Vector3 force_msg;
  force_msg.x = Contact_Force_val[0];
  force_msg.y = Contact_Force_val[1];
  force_msg.z = Contact_Force_val[2];
  vive_force_pub_->publish(force_msg);

  geometry_msgs::msg::Vector3 moment_msg;
  moment_msg.x = Contact_Moment_val[0];
  moment_msg.y = Contact_Moment_val[1];
  moment_msg.z = Contact_Moment_val[2];
  vive_moment_pub_->publish(moment_msg);

  std_msgs::msg::Float64MultiArray acc_msg;
  acc_msg.data.reserve(9);
  acc_msg.data.push_back(CPos_acc_val[0]);
  acc_msg.data.push_back(CPos_acc_val[1]);
  acc_msg.data.push_back(CPos_acc_val[2]);
  acc_msg.data.push_back(CAng_acc_val[0]);
  acc_msg.data.push_back(CAng_acc_val[1]);
  acc_msg.data.push_back(CAng_acc_val[2]);
  acc_msg.data.push_back(CAng_vel_val[0]);
  acc_msg.data.push_back(CAng_vel_val[1]);
  acc_msg.data.push_back(CAng_vel_val[2]);
  vive_acc_pub_->publish(acc_msg);
}

void FT_processing::FT_print()
{
  if (YamlPrint_switch == 1)
  {
    printf("Fx:%10f, Fy:%10f, Fz:%10f \n", Force_val[0], Force_val[1], Force_val[2]);
    printf("Mx:%10f, My:%10f, Mz:%10f \n", Moment_val[0], Moment_val[1], Moment_val[2]);
    printf("CFx:%10f, CFy:%10f, CFz:%10f \n", Contact_Force_val[0], Contact_Force_val[1], Contact_Force_val[2]);
    printf("CMx:%10f, CMy:%10f, CMz:%10f \n", Contact_Moment_val[0], Contact_Moment_val[1], Contact_Moment_val[2]);
    if (HaccSwitch)
    {
      printf("Hacc_x:%10f, Hacc_y:%10f, Hacc_z:%10f \n", Pos_acc_val[0], Pos_acc_val[1], Pos_acc_val[2]);
      printf("Hang_acc_x:%10f, Hang_acc_y:%10f, Hang_acc_z:%10f \n", Ang_acc_val[0], Ang_acc_val[1], Ang_acc_val[2]);
      printf("Hang_vel_x:%10f, Hang_vel_y:%10f, Hang_vel_z:%10f \n", Ang_vel_val[0], Ang_vel_val[1], Ang_vel_val[2]);
    }
    if (CaccSwitch)
    {
      printf("Cacc_x:%10f, Cacc_y:%10f, Cacc_z:%10f \n", CPos_acc_val[0], CPos_acc_val[1], CPos_acc_val[2]);
      printf("Cang_acc_x:%10f, Cang_acc_y:%10f, Cang_acc_z:%10f \n", CAng_acc_val[0], CAng_acc_val[1], CAng_acc_val[2]);
      printf("Cang_vel_x:%10f, Cang_vel_y:%10f, Cang_vel_z:%10f \n", CAng_vel_val[0], CAng_vel_val[1], CAng_vel_val[2]);
    }
    printf("--------------------------------------------------\n");
  }
}

void FT_processing::FT_record()
{
  if (YamlData1_switch == 1)
  {
    if (Data1_txt != NULL)
    {
      fprintf(Data1_txt, "%10f %10f %10f %10f %10f %10f\n",
              Force_val[0], Force_val[1], Force_val[2],
              Moment_val[0], Moment_val[1], Moment_val[2]);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Data1 does not open : warning !!");
    }
  }
}

bool FT_processing::SRV5_Handle(
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  sensor_init_counter = 0;
  aidinGui_stateMsg.data = "Sensor was initialized";
  aidinGui_statePub->publish(aidinGui_stateMsg);
  return true;
}

void FT_processing::FT_run()
{
  double init_sec = 5;
  FT_init((int)(init_sec / Ts_));
  std::cout << "Sensor was initialized" << std::endl;

  rclcpp::WallRate loop_rate(3 * (1.0 / Ts_));

  while (rclcpp::ok())
  {
    if (TCP_start() != 0)
    {
      if (Sensor_value_init())
      {
        FT_filtering();
        FT_publish();
        FT_print();
        FT_record();
      }
    }

    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
}
