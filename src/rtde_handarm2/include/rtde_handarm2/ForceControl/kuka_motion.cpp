#include "Y2RobMotion/kuka_motion.hpp"

/* Constructor */
kuka_motion::kuka_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name, double Control_period, int numOfJoint, const YMatrix& HTMEE2TCP): 
node_(node),robot_name(RB_name),KinematicsKUKAiiwa(Control_period, static_cast<size_t>(numOfJoint), HTMEE2TCP),
numOfJoints(numOfJoint),current_angles(numOfJoint,0.0), target_angles(numOfJoint,0.0), 
current_angvel(numOfJoint,0.0), target_angvel(numOfJoint,0.0), Control_period_(Control_period),
current_pose(6,0.0), target_pose(6,0.0), AC_pose(6,0.0), HG_AC_desX(6,0.0), target_HTM(4,4), ft1data(6,0.0),
current_carvel(6,0.0), target_carvel(6,0.0), pre_current_angles(numOfJoint,0.0), pre_target_angles(numOfJoint,0.0), 
pre_current_pose(6,0.0), pre_target_pose(6,0.0), FC_AC_desX(9,0.0), FC_MASS(6,0.0), FC_DAMPER(6,0.0), FC_STIFFNESS(6,0.0)
{

    /* Target HTM  init */
    target_HTM = YMatrix::identity(4);

    /* Control mode init */
    control_mode = "Idling";
    pre_control_mode = "none";

    /* Force control mode init */
    force_con_mode = "Defualt";

    // Set QP - parameters
    setControlGains(1.0, 1.0);
    setQPWeights(1.0, 0.1, 0.01);

    // Set joint limits
    std::vector<double> q_min =
    {-DegreeToRadian(170),-DegreeToRadian(120),-DegreeToRadian(170),
    -DegreeToRadian(120),-DegreeToRadian(170),-DegreeToRadian(120),-DegreeToRadian(175)};
    std::vector<double> q_max =
    {DegreeToRadian(170), DegreeToRadian(120), DegreeToRadian(170),
     DegreeToRadian(120), DegreeToRadian(170), DegreeToRadian(120), DegreeToRadian(175)};
    
    std::vector<double> qd_min = 
    {-DegreeToRadian(60),-DegreeToRadian(60), -DegreeToRadian(0), // 70
     -DegreeToRadian(90),-DegreeToRadian(100),-DegreeToRadian(120),-DegreeToRadian(120)};
    std::vector<double> qd_max =
    {DegreeToRadian(60), DegreeToRadian(60), DegreeToRadian(0), //70
     DegreeToRadian(90), DegreeToRadian(100), DegreeToRadian(120), DegreeToRadian(120)};

    setJointLimits(q_min, q_max, qd_min, qd_max);

    std::vector<double> qdd_min = 
    {-DegreeToRadian(200),-DegreeToRadian(200),-DegreeToRadian(250),
    -DegreeToRadian(400),-DegreeToRadian(400),-DegreeToRadian(700),-DegreeToRadian(700)};
    std::vector<double> qdd_max=
    {DegreeToRadian(200), DegreeToRadian(200), DegreeToRadian(250),
     DegreeToRadian(400), DegreeToRadian(400), DegreeToRadian(700), DegreeToRadian(700)};

    setAccelLimits(qdd_min,qdd_max);

    /* ROS Publisher init */
    std::string currentJ_TP = robot_name + "/currentJ";
    std::string currentP_TP = robot_name + "/currentP";
    std::string currentF_TP = robot_name + "/currentF";
    std::string targetJ_TP = robot_name + "/targetJ";
    std::string targetP_TP = robot_name + "/targetP";
    std::string targetF_TP = robot_name + "/targetF";
    std::string ctlMode_TP = robot_name + "/ctlMode";
    std::string kuka_targetJ_TP = robot_name + "/command/joint_position"; // Kuka command message topic

    currentJ_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentJ_TP,1);
    currentP_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentP_TP,1);
    currentF_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentF_TP,1);
    targetJ_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(targetJ_TP,1);
    targetP_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(targetP_TP,1);
    targetF_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(targetF_TP,1);
    ctlMode_pub = node_->create_publisher<std_msgs::msg::String>(ctlMode_TP,10);
    kuka_targetJ_pub = node_->create_publisher<lbr_fri_idl::msg::LBRJointPositionCommand>(kuka_targetJ_TP,1);
    RCLCPP_INFO(node_->get_logger(),"Publisher was generated");

    /* ROS Subscriber init */
    std::string cmdMotion_TP = robot_name + "/cmdMotion";
    std::string cmdMode_TP = robot_name + "/cmdMode";
    std::string jointState_TP = robot_name + "/joint_states";
    std::string ftdata_TP = robot_name + "/ftdata";

    cmdMotion_sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>(cmdMotion_TP, 1, 
        std::bind(&kuka_motion::cmdMotionCB, this, std::placeholders::_1));
    cmdMode_sub = node_->create_subscription<std_msgs::msg::String>(cmdMode_TP, 10, 
        std::bind(&kuka_motion::cmdModeCB, this, std::placeholders::_1));
    
    JointState_sub = node_->create_subscription<sensor_msgs::msg::JointState>(jointState_TP, 1, 
        std::bind(&kuka_motion::JointStateCB, this, std::placeholders::_1));
    ftsensor_sub = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(ftdata_TP, 1, 
        std::bind(&kuka_motion::ftsensorCB, this, std::placeholders::_1));

    /* ROS timer callback */
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(static_cast<int>(Control_period*1000)),
        std::bind(&kuka_motion::main_control,this));

    minitoring_timer_ = node_->create_wall_timer(std::chrono::milliseconds(static_cast<int>(100)),
        std::bind(&kuka_motion::state_monitoring,this));

    RCLCPP_INFO(node_->get_logger(),"Subscription was generated");

    /* Admittance controller initialization */
    for(int i =0;i<6;i++){AControl[i] = Yadmittance_control(Control_period);}

    /* FAAC instance initialization */
    std::vector<double> process_noise = {0.1,0.1,0.1};
    std::vector<double> measurement_noise = {10,10,10};
    for(int i =0;i<3;i++){FAAC3step[i] = std::make_unique<Nrs3StepFAAC>(Control_period, process_noise, measurement_noise);}
}
/* ROS CMD Motion Callback */
void kuka_motion::cmdMotionCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if(!strcmp(control_mode.c_str(), "Position"))
    {
        for (size_t i = 0; i < msg->data.size() && i < 6; ++i) {
            target_pose[i] = msg->data[i];
        }
    }
    else if(!strcmp(control_mode.c_str(), "Force"))
    {
        for (size_t i = 0; i < msg->data.size() && i < 9; ++i) {
            /* mm->m, AC input: m, rad */
            if(i<3) {FC_AC_desX[i] = (msg->data[i])/(1000.0);}
            else{FC_AC_desX[i] = msg->data[i];}
        }
    }
    else {printf("\033[33mWrong data was received \033[0m\n");}
}

/* ROS CMD Mode Callback */
void kuka_motion::cmdModeCB(const std_msgs::msg::String::SharedPtr msg)
{
    printf("Control mode callback \n");
    control_mode = msg->data;
}

/* ROS Joint State Callback */
void kuka_motion::JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(!current_angles_received) {
        current_angles_received = true;
    }
    
    if (msg->position.size() < numOfJoints || msg->name.size() != msg->position.size()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid joint state message");
        return;
    }
    
    // 첫 번째 메시지에서 매핑 테이블 생성
    if (!mapping_initialized_) {
        initializeJointMapping(msg);
    }
    
    // 매핑 테이블을 사용해서 빠르게 복사
    for (size_t i = 0; i < numOfJoints; ++i) {
        if (joint_mapping_[i] >= 0 && joint_mapping_[i] < static_cast<int>(msg->position.size())) {
            current_angles[i] = msg->position[joint_mapping_[i]];
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                    "Invalid mapping for joint %zu", i);
        }
    }
}

/* ROS FT Sensor Callback */
void kuka_motion::ftsensorCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    // 힘(Force) 정보 추출
    ft1data[0] = msg->wrench.force.x;
    ft1data[1] = msg->wrench.force.y;
    ft1data[2] = msg->wrench.force.z;
    
    // 토크(Torque) 정보 추출
    ft1data[3] = msg->wrench.torque.x;
    ft1data[4] = msg->wrench.torque.y;
    ft1data[5] = msg->wrench.torque.z;
}

/* joint state mapping function */
void kuka_motion::initializeJointMapping(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_mapping_.resize(numOfJoints, -1);
    
    for (size_t i = 0; i < numOfJoints; ++i) {
        for (size_t j = 0; j < msg->name.size(); ++j) {
            if (joint_names[i] == msg->name[j]) {
                joint_mapping_[i] = j;
                RCLCPP_INFO(node_->get_logger(), "Joint %s mapped: expected_index=%zu -> msg_index=%zu", 
                            joint_names[i].c_str(), i, j);
                break;
            }
        }
        
        if (joint_mapping_[i] == -1) {
            RCLCPP_ERROR(node_->get_logger(), "Joint %s not found in joint state message!", 
                        joint_names[i].c_str());
        }
    }
    mapping_initialized_ = true;
}

/* State Update */
void kuka_motion::state_update()
{
    /* Joint velocity calculation */
    for(int i=0;i<numOfJoints;i++)
    {
        current_angvel[i] = (current_angles[i]-pre_current_angles[i])/Control_period_;
        target_angvel[i] = (target_angles[i]-pre_target_angles[i])/Control_period_;
    }

    /* Current pose update */
    YMatrix current_HTM = forwardKinematics(current_angles);
    YMatrix current_orientation = current_HTM.extract(0,0,3,3);
    SpatialAngle current_SPangle = current_orientation.toSpatialAngle();

    current_pose[0] = current_HTM[0][3];
    current_pose[1] = current_HTM[1][3];
    current_pose[2] = current_HTM[2][3];
    current_pose[3] = current_SPangle.x;
    current_pose[4] = current_SPangle.y;
    current_pose[5] = current_SPangle.z;

    /* Cartesian velocity calculation */
    for(int i=0;i<6;i++)
    {
        current_carvel[i] = (current_pose[i]-pre_current_pose[i])/Control_period_;
        target_carvel[i] = (target_pose[i]-pre_target_pose[i])/Control_period_;
    }

    /* State update */
    pre_current_angles = current_angles;
    pre_target_angles = target_angles;
    pre_current_pose = current_pose;
    pre_target_pose = target_pose;
}

/* ROS State Publisher */
void kuka_motion::state_publisher()
{
    /* Clear the message array */
    ctlMode_msg.data.clear();
    currentJ_msg.data.clear();
    currentp_msg.data.clear();
    currentF_msg.data.clear();

    targetJ_msg.data.clear();
    targetP_msg.data.clear();
    targetF_msg.data.clear();

    /* Enter the data */
    for(int i =0; i<6; i++)
    {
        currentp_msg.data.push_back(current_pose[i]);
        currentF_msg.data.push_back(ft1data[i]);

        targetP_msg.data.push_back(target_pose[i]);
        if(i<3){targetF_msg.data.push_back(FC_AC_desX[i+6]);}
        else {targetF_msg.data.push_back(0.0);}
    }
    for(int i =0; i<numOfJoints; i++)
    {
        currentJ_msg.data.push_back(current_angles[i]);
        targetJ_msg.data.push_back(target_angles[i]);
        kuka_targetJ_msg.joint_position[i] = target_angles[i];
    }

    ctlMode_msg.data = control_mode;

    /* Publish the data */
    ctlMode_pub->publish(ctlMode_msg);
    currentJ_pub->publish(currentJ_msg);
    currentP_pub->publish(currentp_msg);
    currentF_pub->publish(currentF_msg);

    targetJ_pub->publish(targetJ_msg);
    targetP_pub->publish(targetP_msg);
    targetF_pub->publish(targetF_msg);
    kuka_targetJ_pub->publish(kuka_targetJ_msg);
}

/* Control Mode : Idling */
void kuka_motion::control_idling()
{
    control_mode = "Idling";
    
    /* Target position init - accential!!! */
    if(pre_control_mode != control_mode)
    {
        target_pose = current_pose;
        target_angles = current_angles;
    }

    /* Idling Mode */
    if(control_mode == "Idling"){
        /* Generate target HTM */
        std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
        auto target_rot = YMatrix::fromSpatialAngle(target_ori);
        target_HTM = YMatrix::identity(4);
        target_HTM.insert(0, 0, target_rot);
        target_HTM[0][3] = target_pose[0]; // mm unit
        target_HTM[1][3] = target_pose[1]; // mm unit
        target_HTM[2][3] = target_pose[2]; // mm unit

        /* Inverse kinematics using QP-solver */
        target_angles = solve_IK(target_angles, target_HTM);
    }

    pre_control_mode = control_mode; // Store previous control mode for comparison
}

/* Control Mode : Position */
void kuka_motion::control_position()
{
    control_mode = "Position";

    /* Target position init - accential!!! */
    if(pre_control_mode != control_mode)
    {
        target_pose = current_pose;
        target_angles = current_angles;
    }

    /* Position Mode */
    if(control_mode == "Position"){
        /* Generate target HTM */
        std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
        auto target_rot = YMatrix::fromSpatialAngle(target_ori);
        target_HTM = YMatrix::identity(4);
        target_HTM.insert(0, 0, target_rot);
        target_HTM[0][3] = target_pose[0]; // mm unit
        target_HTM[1][3] = target_pose[1]; // mm unit
        target_HTM[2][3] = target_pose[2]; // mm unit
        
        /* Inverse kinematics using QP-solver */
        target_angles = solve_IK(target_angles, target_HTM);

    }
    pre_control_mode = control_mode; // Store previous control mode for comparison
}

void kuka_motion::main_control()
{   
    if(start_flag){
        /* 1. state_update - Must place at first */
        state_update();

        /* 2. Controller selection */
        if(control_mode == "Idling") control_idling();
        else if (control_mode == "Position") control_position();
        else if (control_mode == "Guiding") control_guiding();
        else if (control_mode == "Force") control_force();
        else control_idling();
        
        /* 3. ROS State publishing */
        state_publisher();

    }
}