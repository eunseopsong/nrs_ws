#include "nrs_ft_aq2/FT_Processing.hpp"

FT_processing::FT_processing(ros::NodeHandle nh, double Ts, unsigned char& HandleID_, unsigned char& ContactID_, bool HaccSwitch_, bool CaccSwitch_)
:nh_(nh),NRS_FTSensor(HandleID_, ContactID_, HaccSwitch_, CaccSwitch_),Ts_(Ts), HaccSwitch(HaccSwitch_), CaccSwitch(CaccSwitch_),
movF(3,NRS_MovFilter(Mov_num)), movM(3,NRS_MovFilter(Mov_num)), movCF(3,NRS_MovFilter(Mov_num)), movCM(3,NRS_MovFilter(Mov_num)),
LPF_F(3,NRS_FreqFilter(Ts_)), LPF_M(3,NRS_FreqFilter(Ts_)), LPF_CF(3,NRS_FreqFilter(Ts_)), LPF_CM(3,NRS_FreqFilter(Ts_)),
BSF_F(3,NRS_FreqFilter(Ts_)), BSF_M(3,NRS_FreqFilter(Ts_)), BSF_CF(3,NRS_FreqFilter(Ts_)), BSF_CM(3,NRS_FreqFilter(Ts_))
{
	/* Read from the "config.yaml" */
	if(!nh_.getParam("AFT80IP",YamlString_IP)) {ROS_ERROR("Can't find AFT80IP!");}
	if(!nh_.getParam("Data1_path",YamlData1_path)) {ROS_ERROR("Can't find Data1_path!");}
	if(!nh_.getParam("Data1_switch",YamlData1_switch)) {ROS_ERROR("Can't find Data1_switch!");}
	if(!nh_.getParam("Print_switch",YamlPrint_switch)) {ROS_ERROR("Can't find Print_switch!");}

	if(!nh_.getParam("Handle_Sensor_Order",H_sen_order)) {ROS_ERROR("Can't find Handle_Sensor_Order!");}
	if(!nh_.getParam("Handle_Sensor_sign",H_sen_sign)) {ROS_ERROR("Can't find Handle_Sensor_sign!");}
	if(!nh_.getParam("Contact_Sensor_Order",C_sen_order)) {ROS_ERROR("Can't find Contact_Sensor_Order!");}
	if(!nh_.getParam("Contact_Sensor_sign",C_sen_sign)) {ROS_ERROR("Can't find Contact_Sensor_sign!");}

	if(!nh_.getParam("Hmov_switch",Hmov_switch)) {ROS_ERROR("Can't find Hmov_switch!");}
	if(!nh_.getParam("HLPF_switch",HLPF_switch)) {ROS_ERROR("Can't find HLPF_switch!");}
	if(!nh_.getParam("HBSF_switch",HBSF_switch)) {ROS_ERROR("Can't find HBSF_switch!");}

	if(!nh_.getParam("Cmov_switch",Cmov_switch)) {ROS_ERROR("Can't find Cmov_switch!");}
	if(!nh_.getParam("CLPF_switch",CLPF_switch)) {ROS_ERROR("Can't find CLPF_switch!");}
	if(!nh_.getParam("CBSF_switch",CBSF_switch)) {ROS_ERROR("Can't find CBSF_switch!");}

	/* ROS publisher init*/
	ftsensor_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ftsensor/measured_Hvalue",10);
	Cftsensor_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ftsensor/measured_Cvalue",10);
	aidinGui_statePub = nh.advertise<std_msgs::String>("Aidin_State_Text", 20);

    Vive_Force_pub = nh_.advertise<NRS_FT_AQ::vive_ft_msg>("vive_force", 10);
    Vive_Moment_pub = nh_.advertise<NRS_FT_AQ::vive_ft_msg>("vive_moment", 10);
	Vive_Acc_pub = nh_.advertise<std_msgs::Float64MultiArray>("vive_acc", 10);

	/* ROS Service init*/
	Aidin_gui_srv5 = nh.advertiseService("sensor_zeroset", &FT_processing::SRV5_Handle, this); // Sensor initial

	CAN_sampling = Ts;

}
FT_processing::~FT_processing()
{
	ROS_ERROR("FT_processing was terminated \n");
	fclose(Data1_txt);
}

void FT_processing::FT_init(int sen_init_num)
{
	/* Recording initialization */
	if(YamlData1_switch == 1)
	{Data1_txt = fopen(YamlData1_path.c_str(), "wt");}

	/* eCAN initialization */
	char *AFT80_IP = const_cast<char *>(YamlString_IP.c_str());
	TCP_init(AFT80_IP, 4001);

	/* Filters setting */
	for(int i = 0; i<3; i++)
	{
		/* LPF */
		LPF_F[i].LPF_cutF = LPF_cutF;
		LPF_M[i].LPF_cutF = LPF_cutF;
		LPF_CF[i].LPF_cutF = CLPF_cutF;
		LPF_CM[i].LPF_cutF = CLPF_cutF;

		/* BSF */
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
}

void FT_processing::FT_filtering()
{
	for(int i = 0; i < 3; i++)
	{

		/* Moving average filter */
		if(Hmov_switch)
		{
			Force_val[i] = movF[i].MovFilter(Force_val[i]);
			Moment_val[i] = movM[i].MovFilter(Moment_val[i]);
		}
		if(Cmov_switch)
		{
			Contact_Force_val[i] = movCF[i].MovFilter(Contact_Force_val[i]);
			Contact_Moment_val[i] = movCM[i].MovFilter(Contact_Moment_val[i]);
		}

		/* Low pass filter */
		if(HLPF_switch)
		{
			Force_val[i] = LPF_F[i].LPF(Force_val[i]);
			Moment_val[i] = LPF_M[i].LPF(Moment_val[i]);
		}
		if(CLPF_switch)
		{
			Contact_Force_val[i] = LPF_CF[i].LPF(Contact_Force_val[i]);
			Contact_Moment_val[i] = LPF_CM[i].LPF(Contact_Moment_val[i]);
		}

		/* Band stop filter */
		if(HBSF_switch)
		{
			Force_val[i] = BSF_F[i].BSF(Force_val[i]);
			Moment_val[i] = BSF_M[i].BSF(Moment_val[i]);
		}
		if(CBSF_switch)
		{
			Contact_Force_val[i] = BSF_CF[i].BSF(Contact_Force_val[i]);
			Contact_Moment_val[i] = BSF_CM[i].BSF(Contact_Moment_val[i]);
		}

	}
}

void FT_processing::FT_publish()
{
	/* Handle Force/Moment Pub */
	pub_data.force.x = Force_val[0];
	pub_data.force.y = Force_val[1];
	pub_data.force.z = Force_val[2];
	pub_data.torque.x = Moment_val[0];
	pub_data.torque.y = Moment_val[1];
	pub_data.torque.z = Moment_val[2];
	ftsensor_pub_.publish(pub_data);

	/* Contact Force/Moment Pub */
	Cpub_data.force.x = Contact_Force_val[0];
	Cpub_data.force.y = Contact_Force_val[1];
	Cpub_data.force.z = Contact_Force_val[2];
	Cpub_data.torque.x = Contact_Moment_val[0];
	Cpub_data.torque.y = Contact_Moment_val[1];
	Cpub_data.torque.z = Contact_Moment_val[2];
	Cftsensor_pub_.publish(Cpub_data);

	/* Vive Force/Moment Pub */
	Vive_Force_data.Fx = Contact_Force_val[0];
	Vive_Force_data.Fy = Contact_Force_val[1];
	Vive_Force_data.Fz = Contact_Force_val[2];
	Vive_Moment_data.Mx = Contact_Moment_val[0];
	Vive_Moment_data.My = Contact_Moment_val[1];
	Vive_Moment_data.Mz = Contact_Moment_val[2];
	Vive_Force_pub.publish(Vive_Force_data);
	Vive_Moment_pub.publish(Vive_Moment_data);
	

	/* Vive Acceleration Pub */
	Vive_Acc_data.data.clear();
	Vive_Acc_data.data.push_back(CPos_acc_val[0]);
	Vive_Acc_data.data.push_back(CPos_acc_val[1]);
	Vive_Acc_data.data.push_back(CPos_acc_val[2]);
	Vive_Acc_data.data.push_back(CAng_acc_val[0]);
	Vive_Acc_data.data.push_back(CAng_acc_val[1]);
	Vive_Acc_data.data.push_back(CAng_acc_val[2]);
	Vive_Acc_data.data.push_back(CAng_vel_val[0]);
	Vive_Acc_data.data.push_back(CAng_vel_val[1]);
	Vive_Acc_data.data.push_back(CAng_vel_val[2]);
	Vive_Acc_pub.publish(Vive_Acc_data);

}

void FT_processing::FT_print()
{
	if (YamlPrint_switch == 1)
	{
		printf("Fx:%10f, Fy:%10f, Fz:%10f \n", Force_val[0], Force_val[1], Force_val[2]);
		printf("Mx:%10f, My:%10f, Mz:%10f \n", Moment_val[0], Moment_val[1], Moment_val[2]);
		printf("CFx:%10f, CFy:%10f, CFz:%10f \n", Contact_Force_val[0], Contact_Force_val[1], Contact_Force_val[2]);
		printf("CMx:%10f, CMy:%10f, CMz:%10f \n", Contact_Moment_val[0], Contact_Moment_val[1], Contact_Moment_val[2]);

		if(HaccSwitch)
		{
			printf("Hacc_x:%10f, Hacc_y:%10f, Hacc_z:%10f \n", Pos_acc_val[0], Pos_acc_val[1], Pos_acc_val[2]);
			printf("Hang_acc_x:%10f, Hang_acc_y:%10f, Hang_acc_z:%10f \n", Ang_acc_val[0], Ang_acc_val[1], Ang_acc_val[2]);
			printf("Hang_vel_x:%10f, Hang_vel_y:%10f, Hang_vel_z:%10f \n", Ang_vel_val[0], Ang_vel_val[1], Ang_vel_val[2]);
		}
		if(CaccSwitch)
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
			// data recording
			fprintf(Data1_txt, "%10f %10f %10f %10f %10f %10f\n", Force_val[0], Force_val[1], Force_val[2],
					Moment_val[0], Moment_val[1], Moment_val[2]);
		}
		else
		{
			ROS_ERROR("Data1 does not open : warnning !!");
		}
	}
}

bool FT_processing::SRV5_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	sensor_init_counter = 0;
	aidinGui_stateMsg.data = "Sensor was initialized";
	aidinGui_statePub.publish(aidinGui_stateMsg);
}

void FT_processing::FT_run()
{
	/* Sensor init before execution */
	double init_sec = 5; // init during "init_sec" 
	FT_init((int)(init_sec/Ts_));
	std::cout<< "Sensor was initialized" << std::endl;

    ros::Rate loop_rate(3*(1/Ts_));

	/* Sensor execution */
	while(ros::ok()) // Ros node 종료시 해당 while문을 종료 해야하기에 while내부는 왠만하면 ros::ok() 사용!!
	{
		/* 
		- 만약 1kHz의 센서를 쓴다면, aliasing을 고려 5배 이상의 sampling time을
		  유지하는 것이 좋다.
		- 이에 Loop date를 1kHz로 설정하여 데이터를 받아 오는 방식이 아닌 컴터가 돌수 있는
		  최대 속도로 돌리는 while(1)을 사용하였다.
		*/

		if (TCP_start() != 0) // if the contact sensor added, must be changed
		{
			/* Sensor value initialization */ 
			if (Sensor_value_init())
			{
				/* Sensor value filtering */
				FT_filtering();

				/* Publish the FT data */
				FT_publish();

				/* Printing */
				FT_print();

				/* Recording */
				FT_record();
			}

			/* ROS spinOnce for ros-service */
			ros::spinOnce();

		}

		loop_rate.sleep();
	}
}

