#include "FtProcessing.h"

FtProcessing::FtProcessing(double Ts, unsigned char& HandleID_, unsigned char& ContactID_, bool HaccSwitch_, bool CaccSwitch_)
: Node("ft_processing_node"),
  NrsFtSensor(HandleID_, ContactID_, HaccSwitch_, CaccSwitch_),
  Ts_(Ts), HaccSwitch(HaccSwitch_), CaccSwitch(CaccSwitch_),
  movF(3, NRS_MovFilter(Mov_num)), movM(3, NRS_MovFilter(Mov_num)),
  movCF(3, NRS_MovFilter(Mov_num)), movCM(3, NRS_MovFilter(Mov_num)),
  LPF_F(3, NRS_FreqFilter(Ts_)), LPF_M(3, NRS_FreqFilter(Ts_)),
  LPF_CF(3, NRS_FreqFilter(Ts_)), LPF_CM(3, NRS_FreqFilter(Ts_)),
  BSF_F(3, NRS_FreqFilter(Ts_)), BSF_M(3, NRS_FreqFilter(Ts_)),
  BSF_CF(3, NRS_FreqFilter(Ts_)), BSF_CM(3, NRS_FreqFilter(Ts_))
{
  // Declare and read parameters
  this->declare_parameter("AFT80IP", "");
  this->declare_parameter("Data1_path", "");
  this->declare_parameter("Data1_switch", 0);
  this->declare_parameter("Print_switch", 0);

  this->declare_parameter("Handle_Sensor_Order", std::vector<int>{});
  this->declare_parameter("Handle_Sensor_sign", std::vector<int>{});
  this->declare_parameter("Contact_Sensor_Order", std::vector<int>{});
  this->declare_parameter("Contact_Sensor_sign", std::vector<int>{});

  this->declare_parameter("Hmov_switch", false);
  this->declare_parameter("HLPF_switch", false);
  this->declare_parameter("HBSF_switch", false);
  this->declare_parameter("Cmov_switch", false);
  this->declare_parameter("CLPF_switch", false);
  this->declare_parameter("CBSF_switch", false);

  // Get parameters
  if (!this->get_parameter("AFT80IP", YamlString_IP)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find AFT80IP!");
  }
  if (!this->get_parameter("Data1_path", YamlData1_path)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Data1_path!");
  }
  if (!this->get_parameter("Data1_switch", YamlData1_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Data1_switch!");
  }
  if (!this->get_parameter("Print_switch", YamlPrint_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Print_switch!");
  }

  if (!this->get_parameter("Handle_Sensor_Order", H_sen_order)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Handle_Sensor_Order!");
  }
  if (!this->get_parameter("Handle_Sensor_sign", H_sen_sign)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Handle_Sensor_sign!");
  }
  if (!this->get_parameter("Contact_Sensor_Order", C_sen_order)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Contact_Sensor_Order!");
  }
  if (!this->get_parameter("Contact_Sensor_sign", C_sen_sign)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Contact_Sensor_sign!");
  }

  if (!this->get_parameter("Hmov_switch", Hmov_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Hmov_switch!");
  }
  if (!this->get_parameter("HLPF_switch", HLPF_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find HLPF_switch!");
  }
  if (!this->get_parameter("HBSF_switch", HBSF_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find HBSF_switch!");
  }
  if (!this->get_parameter("Cmov_switch", Cmov_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find Cmov_switch!");
  }
  if (!this->get_parameter("CLPF_switch", CLPF_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find CLPF_switch!");
  }
  if (!this->get_parameter("CBSF_switch", CBSF_switch)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find CBSF_switch!");
  }

  // ROS 2 Publisher init
  ftsensor_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ftsensor/measured_Hvalue", 10);
  Cftsensor_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ftsensor/measured_Cvalue", 10);
  aidinGui_statePub = this->create_publisher<std_msgs::msg::String>("Aidin_State_Text", 20);

  Vive_Force_pub = this->create_publisher<nrs_ft_aq2::msg::ViveFTMsg>("vive_force", 10);
  Vive_Moment_pub = this->create_publisher<nrs_ft_aq2::msg::ViveFTMsg>("vive_moment", 10);
  Vive_Acc_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("vive_acc", 10);

  // ROS 2 Service init
  Aidin_gui_srv5 = this->create_service<std_srvs::srv::Empty>(
    "sensor_zeroset",
    std::bind(&FtProcessing::SRV5_Handle, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Sampling time
  CAN_sampling = Ts;
}
// FtProcessing::FtProcessing(ros::NodeHandle nh, double Ts, unsigned char& HandleID_, unsigned char& ContactID_, bool HaccSwitch_, bool CaccSwitch_)
// :nh_(nh),NRS_FTSensor(HandleID_, ContactID_, HaccSwitch_, CaccSwitch_),Ts_(Ts), HaccSwitch(HaccSwitch_), CaccSwitch(CaccSwitch_),
// movF(3,NRS_MovFilter(Mov_num)), movM(3,NRS_MovFilter(Mov_num)), movCF(3,NRS_MovFilter(Mov_num)), movCM(3,NRS_MovFilter(Mov_num)),
// LPF_F(3,NRS_FreqFilter(Ts_)), LPF_M(3,NRS_FreqFilter(Ts_)), LPF_CF(3,NRS_FreqFilter(Ts_)), LPF_CM(3,NRS_FreqFilter(Ts_)),
// BSF_F(3,NRS_FreqFilter(Ts_)), BSF_M(3,NRS_FreqFilter(Ts_)), BSF_CF(3,NRS_FreqFilter(Ts_)), BSF_CM(3,NRS_FreqFilter(Ts_))
// {
// 	/* Read from the "config.yaml" */
// 	if(!nh_.getParam("AFT80IP",YamlString_IP)) {ROS_ERROR("Can't find AFT80IP!");}
// 	if(!nh_.getParam("Data1_path",YamlData1_path)) {ROS_ERROR("Can't find Data1_path!");}
// 	if(!nh_.getParam("Data1_switch",YamlData1_switch)) {ROS_ERROR("Can't find Data1_switch!");}
// 	if(!nh_.getParam("Print_switch",YamlPrint_switch)) {ROS_ERROR("Can't find Print_switch!");}

// 	if(!nh_.getParam("Handle_Sensor_Order",H_sen_order)) {ROS_ERROR("Can't find Handle_Sensor_Order!");}
// 	if(!nh_.getParam("Handle_Sensor_sign",H_sen_sign)) {ROS_ERROR("Can't find Handle_Sensor_sign!");}
// 	if(!nh_.getParam("Contact_Sensor_Order",C_sen_order)) {ROS_ERROR("Can't find Contact_Sensor_Order!");}
// 	if(!nh_.getParam("Contact_Sensor_sign",C_sen_sign)) {ROS_ERROR("Can't find Contact_Sensor_sign!");}

// 	if(!nh_.getParam("Hmov_switch",Hmov_switch)) {ROS_ERROR("Can't find Hmov_switch!");}
// 	if(!nh_.getParam("HLPF_switch",HLPF_switch)) {ROS_ERROR("Can't find HLPF_switch!");}
// 	if(!nh_.getParam("HBSF_switch",HBSF_switch)) {ROS_ERROR("Can't find HBSF_switch!");}

// 	if(!nh_.getParam("Cmov_switch",Cmov_switch)) {ROS_ERROR("Can't find Cmov_switch!");}
// 	if(!nh_.getParam("CLPF_switch",CLPF_switch)) {ROS_ERROR("Can't find CLPF_switch!");}
// 	if(!nh_.getParam("CBSF_switch",CBSF_switch)) {ROS_ERROR("Can't find CBSF_switch!");}

// 	/* ROS publisher init*/
// 	ftsensor_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ftsensor/measured_Hvalue",10);
// 	Cftsensor_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ftsensor/measured_Cvalue",10);
// 	aidinGui_statePub = nh.advertise<std_msgs::String>("Aidin_State_Text", 20);

//     Vive_Force_pub = nh_.advertise<NRS_FT_AQ::vive_ft_msg>("vive_force", 10);
//     Vive_Moment_pub = nh_.advertise<NRS_FT_AQ::vive_ft_msg>("vive_moment", 10);
// 	Vive_Acc_pub = nh_.advertise<std_msgs::Float64MultiArray>("vive_acc", 10);

// 	/* ROS Service init*/
// 	Aidin_gui_srv5 = nh.advertiseService("sensor_zeroset", &FtProcessing::SRV5_Handle, this); // Sensor initial

// 	CAN_sampling = Ts;

// }

FtProcessing::~FtProcessing()
{
  RCLCPP_ERROR(this->get_logger(), "FtProcessing was terminated");

  if (Data1_txt != nullptr) {
    fclose(Data1_txt);
    Data1_txt = nullptr;  // 안전하게 초기화
  }
}
// FtProcessing::~FtProcessing()
// {
// 	//// ROS_ERROR("FtProcessing was terminated \n");
// 	RCLCPP_ERROR(rclcpp::get_logger("FtProcessing"), "FtProcessing was terminated");

// 	fclose(Data1_txt);
// }

void FtProcessing::FT_init(int sen_init_num)
{
  // Recording initialization
  if (YamlData1_switch == 1)
  {
    Data1_txt = fopen(YamlData1_path.c_str(), "wt");
    if (Data1_txt == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to open Data1 file for writing: %s", YamlData1_path.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Opened Data1 file: %s", YamlData1_path.c_str());
    }
  }

  // eCAN initialization
  char* AFT80_IP = const_cast<char*>(YamlString_IP.c_str());
  TCP_init(AFT80_IP, 4001);

  // Filters setting
  for (int i = 0; i < 3; ++i)
  {
    // LPF
    LPF_F[i].LPF_cutF = LPF_cutF;
    LPF_M[i].LPF_cutF = LPF_cutF;
    LPF_CF[i].LPF_cutF = CLPF_cutF;
    LPF_CM[i].LPF_cutF = CLPF_cutF;

    // BSF
    BSF_F[i].BSF_cutF = BSF_cutF;
    BSF_M[i].BSF_cutF = BSF_cutF;
    BSF_CF[i].BSF_cutF = CBSF_cutF;
    BSF_CM[i].BSF_cutF = CBSF_cutF;

    BSF_F[i].BSF_BW = BSF_BW;
    BSF_M[i].BSF_BW = BSF_BW;
    BSF_CF[i].BSF_BW = CBSF_BW;
    BSF_CM[i].BSF_BW = CBSF_BW;
  }

  init_average_num = sen_init_num;

  RCLCPP_INFO(this->get_logger(), "FT_init completed with average num: %d", init_average_num);
}
// void FtProcessing::FT_init(int sen_init_num)
// {
// 	/* Recording initialization */
// 	if(YamlData1_switch == 1)
// 	{Data1_txt = fopen(YamlData1_path.c_str(), "wt");}

// 	/* eCAN initialization */
// 	char *AFT80_IP = const_cast<char *>(YamlString_IP.c_str());
// 	TCP_init(AFT80_IP, 4001);

// 	/* Filters setting */
// 	for(int i = 0; i<3; i++)
// 	{
// 		/* LPF */
// 		LPF_F[i].LPF_cutF = LPF_cutF;
// 		LPF_M[i].LPF_cutF = LPF_cutF;
// 		LPF_CF[i].LPF_cutF = CLPF_cutF;
// 		LPF_CM[i].LPF_cutF = CLPF_cutF;

// 		/* BSF */
// 		BSF_F[i].BSF_cutF = BSF_cutF;
// 		BSF_M[i].BSF_cutF = BSF_cutF;
// 		BSF_CF[i].BSF_cutF = CBSF_cutF;
// 		BSF_CM[i].BSF_cutF = CBSF_cutF;

// 		BSF_F[i].BSF_BW = BSF_BW;
// 		BSF_M[i].BSF_BW = BSF_BW;
// 		BSF_CF[i].BSF_BW = CBSF_BW;
// 		BSF_CM[i].BSF_BW = CBSF_BW;
// 	}
// 	init_average_num = sen_init_num;
// }

void FtProcessing::FT_filtering()
{
  for (int i = 0; i < 3; ++i)
  {
    // Moving average filter
    if (Hmov_switch)
    {
      Force_val[i] = movF[i].MovFilter(Force_val[i]);
      Moment_val[i] = movM[i].MovFilter(Moment_val[i]);
    }

    if (Cmov_switch)
    {
      Contact_Force_val[i] = movCF[i].MovFilter(Contact_Force_val[i]);
      Contact_Moment_val[i] = movCM[i].MovFilter(Contact_Moment_val[i]);
    }

    // Low pass filter
    if (HLPF_switch)
    {
      Force_val[i] = LPF_F[i].LPF(Force_val[i]);
      Moment_val[i] = LPF_M[i].LPF(Moment_val[i]);
    }

    if (CLPF_switch)
    {
      Contact_Force_val[i] = LPF_CF[i].LPF(Contact_Force_val[i]);
      Contact_Moment_val[i] = LPF_CM[i].LPF(Contact_Moment_val[i]);
    }

    // Band stop filter
    if (HBSF_switch)
    {
      Force_val[i] = BSF_F[i].BSF(Force_val[i]);
      Moment_val[i] = BSF_M[i].BSF(Moment_val[i]);
    }

    if (CBSF_switch)
    {
      Contact_Force_val[i] = BSF_CF[i].BSF(Contact_Force_val[i]);
      Contact_Moment_val[i] = BSF_CM[i].BSF(Contact_Moment_val[i]);
    }
  }
}
// void FtProcessing::FT_filtering()
// {
// 	for(int i = 0; i < 3; i++)
// 	{

// 		/* Moving average filter */
// 		if(Hmov_switch)
// 		{
// 			Force_val[i] = movF[i].MovFilter(Force_val[i]);
// 			Moment_val[i] = movM[i].MovFilter(Moment_val[i]);
// 		}
// 		if(Cmov_switch)
// 		{
// 			Contact_Force_val[i] = movCF[i].MovFilter(Contact_Force_val[i]);
// 			Contact_Moment_val[i] = movCM[i].MovFilter(Contact_Moment_val[i]);
// 		}

// 		/* Low pass filter */
// 		if(HLPF_switch)
// 		{
// 			Force_val[i] = LPF_F[i].LPF(Force_val[i]);
// 			Moment_val[i] = LPF_M[i].LPF(Moment_val[i]);
// 		}
// 		if(CLPF_switch)
// 		{
// 			Contact_Force_val[i] = LPF_CF[i].LPF(Contact_Force_val[i]);
// 			Contact_Moment_val[i] = LPF_CM[i].LPF(Contact_Moment_val[i]);
// 		}

// 		/* Band stop filter */
// 		if(HBSF_switch)
// 		{
// 			Force_val[i] = BSF_F[i].BSF(Force_val[i]);
// 			Moment_val[i] = BSF_M[i].BSF(Moment_val[i]);
// 		}
// 		if(CBSF_switch)
// 		{
// 			Contact_Force_val[i] = BSF_CF[i].BSF(Contact_Force_val[i]);
// 			Contact_Moment_val[i] = BSF_CM[i].BSF(Contact_Moment_val[i]);
// 		}

// 	}
// }

void FtProcessing::FT_publish()
{
  // Handle Force/Moment Pub
  auto pub_data_msg = std::make_shared<geometry_msgs::msg::Wrench>();
  pub_data_msg->force.x = Force_val[0];
  pub_data_msg->force.y = Force_val[1];
  pub_data_msg->force.z = Force_val[2];
  pub_data_msg->torque.x = Moment_val[0];
  pub_data_msg->torque.y = Moment_val[1];
  pub_data_msg->torque.z = Moment_val[2];
  ftsensor_pub_->publish(*pub_data_msg);  // 또는 ftsensor_pub_->publish(pub_data_msg)

  // Contact Force/Moment Pub
  auto cpub_data_msg = std::make_shared<geometry_msgs::msg::Wrench>();
  cpub_data_msg->force.x = Contact_Force_val[0];
  cpub_data_msg->force.y = Contact_Force_val[1];
  cpub_data_msg->force.z = Contact_Force_val[2];
  cpub_data_msg->torque.x = Contact_Moment_val[0];
  cpub_data_msg->torque.y = Contact_Moment_val[1];
  cpub_data_msg->torque.z = Contact_Moment_val[2];
  Cftsensor_pub_->publish(*cpub_data_msg);

  // Vive Force/Moment Pub
  auto vive_force_msg = std::make_shared<nrs_ft_aq2::msg::ViveFTMsg>();
  vive_force_msg->fx = Contact_Force_val[0];
  vive_force_msg->fy = Contact_Force_val[1];
  vive_force_msg->fz = Contact_Force_val[2];

  auto vive_moment_msg = std::make_shared<nrs_ft_aq2::msg::ViveFTMsg>();
  vive_moment_msg->mx = Contact_Moment_val[0];
  vive_moment_msg->my = Contact_Moment_val[1];
  vive_moment_msg->mz = Contact_Moment_val[2];

  Vive_Force_pub->publish(*vive_force_msg);
  Vive_Moment_pub->publish(*vive_moment_msg);

  // Vive Acceleration Pub
  auto vive_acc_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  vive_acc_msg->data.clear();
  vive_acc_msg->data.push_back(CPos_acc_val[0]);
  vive_acc_msg->data.push_back(CPos_acc_val[1]);
  vive_acc_msg->data.push_back(CPos_acc_val[2]);
  vive_acc_msg->data.push_back(CAng_acc_val[0]);
  vive_acc_msg->data.push_back(CAng_acc_val[1]);
  vive_acc_msg->data.push_back(CAng_acc_val[2]);
  vive_acc_msg->data.push_back(CAng_vel_val[0]);
  vive_acc_msg->data.push_back(CAng_vel_val[1]);
  vive_acc_msg->data.push_back(CAng_vel_val[2]);
  Vive_Acc_pub->publish(*vive_acc_msg);
}
// void FtProcessing::FT_publish()
// {
// 	/* Handle Force/Moment Pub */
// 	pub_data.force.x = Force_val[0];
// 	pub_data.force.y = Force_val[1];
// 	pub_data.force.z = Force_val[2];
// 	pub_data.torque.x = Moment_val[0];
// 	pub_data.torque.y = Moment_val[1];
// 	pub_data.torque.z = Moment_val[2];
// 	ftsensor_pub_.publish(pub_data);

// 	/* Contact Force/Moment Pub */
// 	Cpub_data.force.x = Contact_Force_val[0];
// 	Cpub_data.force.y = Contact_Force_val[1];
// 	Cpub_data.force.z = Contact_Force_val[2];
// 	Cpub_data.torque.x = Contact_Moment_val[0];
// 	Cpub_data.torque.y = Contact_Moment_val[1];
// 	Cpub_data.torque.z = Contact_Moment_val[2];
// 	Cftsensor_pub_.publish(Cpub_data);

// 	/* Vive Force/Moment Pub */
// 	Vive_Force_data.Fx = Contact_Force_val[0];
// 	Vive_Force_data.Fy = Contact_Force_val[1];
// 	Vive_Force_data.Fz = Contact_Force_val[2];
// 	Vive_Moment_data.Mx = Contact_Moment_val[0];
// 	Vive_Moment_data.My = Contact_Moment_val[1];
// 	Vive_Moment_data.Mz = Contact_Moment_val[2];
// 	Vive_Force_pub.publish(Vive_Force_data);
// 	Vive_Moment_pub.publish(Vive_Moment_data);
	

// 	/* Vive Acceleration Pub */
// 	Vive_Acc_data.data.clear();
// 	Vive_Acc_data.data.push_back(CPos_acc_val[0]);
// 	Vive_Acc_data.data.push_back(CPos_acc_val[1]);
// 	Vive_Acc_data.data.push_back(CPos_acc_val[2]);
// 	Vive_Acc_data.data.push_back(CAng_acc_val[0]);
// 	Vive_Acc_data.data.push_back(CAng_acc_val[1]);
// 	Vive_Acc_data.data.push_back(CAng_acc_val[2]);
// 	Vive_Acc_data.data.push_back(CAng_vel_val[0]);
// 	Vive_Acc_data.data.push_back(CAng_vel_val[1]);
// 	Vive_Acc_data.data.push_back(CAng_vel_val[2]);
// 	Vive_Acc_pub.publish(Vive_Acc_data);

// }

void FtProcessing::FT_print()
{
  if (YamlPrint_switch == 1)
  {
    RCLCPP_INFO(this->get_logger(), "Fx:%10f, Fy:%10f, Fz:%10f", Force_val[0], Force_val[1], Force_val[2]);
    RCLCPP_INFO(this->get_logger(), "Mx:%10f, My:%10f, Mz:%10f", Moment_val[0], Moment_val[1], Moment_val[2]);
    RCLCPP_INFO(this->get_logger(), "CFx:%10f, CFy:%10f, CFz:%10f", Contact_Force_val[0], Contact_Force_val[1], Contact_Force_val[2]);
    RCLCPP_INFO(this->get_logger(), "CMx:%10f, CMy:%10f, CMz:%10f", Contact_Moment_val[0], Contact_Moment_val[1], Contact_Moment_val[2]);

    if (HaccSwitch)
    {
      RCLCPP_INFO(this->get_logger(), "Hacc_x:%10f, Hacc_y:%10f, Hacc_z:%10f", Pos_acc_val[0], Pos_acc_val[1], Pos_acc_val[2]);
      RCLCPP_INFO(this->get_logger(), "Hang_acc_x:%10f, Hang_acc_y:%10f, Hang_acc_z:%10f", Ang_acc_val[0], Ang_acc_val[1], Ang_acc_val[2]);
      RCLCPP_INFO(this->get_logger(), "Hang_vel_x:%10f, Hang_vel_y:%10f, Hang_vel_z:%10f", Ang_vel_val[0], Ang_vel_val[1], Ang_vel_val[2]);
    }
    if (CaccSwitch)
    {
      RCLCPP_INFO(this->get_logger(), "Cacc_x:%10f, Cacc_y:%10f, Cacc_z:%10f", CPos_acc_val[0], CPos_acc_val[1], CPos_acc_val[2]);
      RCLCPP_INFO(this->get_logger(), "Cang_acc_x:%10f, Cang_acc_y:%10f, Cang_acc_z:%10f", CAng_acc_val[0], CAng_acc_val[1], CAng_acc_val[2]);
      RCLCPP_INFO(this->get_logger(), "Cang_vel_x:%10f, Cang_vel_y:%10f, Cang_vel_z:%10f", CAng_vel_val[0], CAng_vel_val[1], CAng_vel_val[2]);
    }

    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
  }
}
// void FtProcessing::FT_print()
// {
// 	if (YamlPrint_switch == 1)
// 	{
// 		printf("Fx:%10f, Fy:%10f, Fz:%10f \n", Force_val[0], Force_val[1], Force_val[2]);
// 		printf("Mx:%10f, My:%10f, Mz:%10f \n", Moment_val[0], Moment_val[1], Moment_val[2]);
// 		printf("CFx:%10f, CFy:%10f, CFz:%10f \n", Contact_Force_val[0], Contact_Force_val[1], Contact_Force_val[2]);
// 		printf("CMx:%10f, CMy:%10f, CMz:%10f \n", Contact_Moment_val[0], Contact_Moment_val[1], Contact_Moment_val[2]);

// 		if(HaccSwitch)
// 		{
// 			printf("Hacc_x:%10f, Hacc_y:%10f, Hacc_z:%10f \n", Pos_acc_val[0], Pos_acc_val[1], Pos_acc_val[2]);
// 			printf("Hang_acc_x:%10f, Hang_acc_y:%10f, Hang_acc_z:%10f \n", Ang_acc_val[0], Ang_acc_val[1], Ang_acc_val[2]);
// 			printf("Hang_vel_x:%10f, Hang_vel_y:%10f, Hang_vel_z:%10f \n", Ang_vel_val[0], Ang_vel_val[1], Ang_vel_val[2]);
// 		}
// 		if(CaccSwitch)
// 		{
// 			printf("Cacc_x:%10f, Cacc_y:%10f, Cacc_z:%10f \n", CPos_acc_val[0], CPos_acc_val[1], CPos_acc_val[2]);
// 			printf("Cang_acc_x:%10f, Cang_acc_y:%10f, Cang_acc_z:%10f \n", CAng_acc_val[0], CAng_acc_val[1], CAng_acc_val[2]);
// 			printf("Cang_vel_x:%10f, Cang_vel_y:%10f, Cang_vel_z:%10f \n", CAng_vel_val[0], CAng_vel_val[1], CAng_vel_val[2]);
// 		}
// 		printf("--------------------------------------------------\n");
// 	}
// }

void FtProcessing::FT_record()
{
	if (YamlData1_switch == 1)
	{
		if (Data1_txt != nullptr)
		{
			// data recording
			fprintf(Data1_txt, "%10f %10f %10f %10f %10f %10f\n",
					Force_val[0], Force_val[1], Force_val[2],
					Moment_val[0], Moment_val[1], Moment_val[2]);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Data1 does not open : warning!!");
		}
	}
}
// void FtProcessing::FT_record()
// {
// 	if (YamlData1_switch == 1)
// 	{
// 		if (Data1_txt != NULL)
// 		{
// 			// data recording
// 			fprintf(Data1_txt, "%10f %10f %10f %10f %10f %10f\n", Force_val[0], Force_val[1], Force_val[2],
// 					Moment_val[0], Moment_val[1], Moment_val[2]);
// 		}
// 		else
// 		{
// 			ROS_ERROR("Data1 does not open : warnning !!");
// 		}
// 	}
// }

void FtProcessing::SRV5_Handle(
	const std::shared_ptr<std_srvs::srv::Empty::Request> request,
	std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	(void)request;  // 사용하지 않음
	(void)response;

	sensor_init_counter = 0;
	aidinGui_stateMsg.data = "Sensor was initialized";
	aidinGui_statePub->publish(aidinGui_stateMsg);
}
// bool FtProcessing::SRV5_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
// {
// 	sensor_init_counter = 0;
// 	aidinGui_stateMsg.data = "Sensor was initialized";
// 	aidinGui_statePub.publish(aidinGui_stateMsg);
// }

void FtProcessing::FT_run()
{
    // 센서 초기화
    double init_sec = 5.0;
    FT_init(static_cast<int>(init_sec / Ts_));
    std::cout << "Sensor was initialized" << std::endl;

    // 루프 주기 설정 (Hz = 1 / Ts_)
    rclcpp::Rate loop_rate(3.0 / Ts_);

    // ROS 2 노드가 실행 중인 동안 루프
    while (rclcpp::ok())
    {
        if (TCP_start() != 0)
        {
            // 센서값 초기화
            if (Sensor_value_init())
            {
                FT_filtering();
                FT_publish();
                FT_print();
                FT_record();
            }

            // 서비스 콜백 처리
            rclcpp::spin_some(shared_from_this());
        }

        loop_rate.sleep();
    }
}
// void FtProcessing::FT_run()
// {
// 	/* Sensor init before execution */
// 	double init_sec = 5; // init during "init_sec" 
// 	FT_init((int)(init_sec/Ts_));
// 	std::cout<< "Sensor was initialized" << std::endl;

//     ros::Rate loop_rate(3*(1/Ts_));

// 	/* Sensor execution */
// 	while(ros::ok()) // Ros node 종료시 해당 while문을 종료 해야하기에 while내부는 왠만하면 ros::ok() 사용!!
// 	{
// 		/* 
// 		- 만약 1kHz의 센서를 쓴다면, aliasing을 고려 5배 이상의 sampling time을
// 		  유지하는 것이 좋다.
// 		- 이에 Loop date를 1kHz로 설정하여 데이터를 받아 오는 방식이 아닌 컴터가 돌수 있는
// 		  최대 속도로 돌리는 while(1)을 사용하였다.
// 		*/

// 		if (TCP_start() != 0) // if the contact sensor added, must be changed
// 		{
// 			/* Sensor value initialization */ 
// 			if (Sensor_value_init())
// 			{
// 				/* Sensor value filtering */
// 				FT_filtering();

// 				/* Publish the FT data */
// 				FT_publish();

// 				/* Printing */
// 				FT_print();

// 				/* Recording */
// 				FT_record();
// 			}

// 			/* ROS spinOnce for ros-service */
// 			ros::spinOnce();

// 		}

// 		loop_rate.sleep();
// 	}
// }

