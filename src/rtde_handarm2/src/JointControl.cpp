#include "JointControl.h"
#include "Controllers/convexMPC/RobotState.h"


JointControl::JointControl()
    : Node("aidin_m1_control_node"), _count(0), previous_command{0, 0, 0}
{
    sub_bodypose = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/BodyPose_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 6, body_pose.begin());
        });
    sub_jointpos = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/JointPos_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 12, joint_pos.begin());
        });
    sub_jointvel = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/JointVel_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 12, joint_vel.begin());
        });
    sub_bodypos = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/BodyPos_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 3, body_pos.begin());
        });
    sub_bodyvel = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/BodyVel_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 3, body_vel.begin());
        });
    sub_imu = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/IMU_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 9, imu.begin());
        });
    sub_contact = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/Contact_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 4, contact.begin());
        });
    sub_command = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/Command_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            // Check if the command has changed
            if (!equal(command.begin(), command.end(), msg->data.begin())) {
                _count = 0; // Reset count if command has changed
                copy(msg->data.begin(), msg->data.begin() + 3, previous_command.begin());
            }
            copy(msg->data.begin(), msg->data.begin() + 3, command.begin());
        });
    sub_distance = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/Distance_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 4, dist.begin());
        });
    sub_link_force = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aidin_m1/LinkForce_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            copy(msg->data.begin(), msg->data.begin() + 21, dist.begin());
        });

    // Publish torque & target joint poses
    pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>("/aidin_m1/Torque_sim", 10);
    pub_targetpos = this->create_publisher<std_msgs::msg::Float32MultiArray>("/aidin_m1/TargetPos", 10);

    // node Publish 실행 주기 설정 (1ms)
    timer_ = this->create_wall_timer(1ms, std::bind(&JointControl::CalculateAndPublishTorque, this));
}

void JointControl::CalculateAndPublishTorque()
{
    // Check if the command array contains valid values
    if (std::any_of(command.begin(), command.end(), [](double c) { return c != 0; })) {
        _count += 0.001; // Increment simulation time by 1ms
    }

    // Gain Initialization
    Matrix3d Kp, Kd;
    ////  Initial_Standing  SWing_Phase  STanding_Phase
    Kp <<       600,            600,             600, //// Scapula
               4000,           4000,            6000, //// Hip
              26000,          50000,           55000; //// Knee
    Kd <<        20,             20,              20, //// Scapula
                 20,             20,              20, //// Hip
                 10,             20,              15; //// Knee

    double vel_of_body = command[1]; // Target velocity of the whole robot body (mm/s)
    double T = command[2];           // Period of the whole trajectory phase    (sec)
    double t1 = fmod(_count +   (T/4), T);
    double t2 = fmod(_count + (3*T/4), T);
    // double t_counter = T/4;
    double _t = fmod(_count, T*4);

    // RCLCPP_INFO(this->get_logger(), "t: %f, count_: %f", _t, _count); // t 값을 디버깅하기 위해 출력

    // Calculate the coordinate using Trajectory Function
    double y_const = 95;
    double x1, z1;
    SplineTrajectory(t1, T, vel_of_body, x1, z1);

    double x2, z2;
    SplineTrajectory(t2, T, vel_of_body, x2, z2);

    // Calculate the target_pos using Inverse Kinematics
    array<double, 3> LF_target_pos, RF_target_pos, LB_target_pos, RB_target_pos;

    switch (int(command[0])) {
        case 1: // the command to keep a robot "standing still"

            break;
        case 2: {// the command to keep a robot "walking in place"
            double t_LF = T/4, t_RF = T/4, t_LB = T/4, t_RB = T/4;
            double x_LF, x_RF, x_LB, x_RB;
            double z_LF, z_RF, z_LB, z_RB;

            if (_t <= T)        t_LF = t1;
            else if (_t <= 2*T) t_RB = t1;
            else if (_t <= 3*T) t_RF = t1;
            else                t_LB = t1;

            SplineTrajectory(t_LF, T, vel_of_body, x_LF, z_LF);
            SplineTrajectory(t_RF, T, vel_of_body, x_RF, z_RF);
            SplineTrajectory(t_LB, T, vel_of_body, x_LB, z_LB);
            SplineTrajectory(t_RB, T, vel_of_body, x_RB, z_RB);

            InverseKinematics3D(y_const, z_LF, x_LF, y_const, 250, 250, LF_target_pos.data());
            InverseKinematics3D(y_const, z_RF, x_RF, y_const, 250, 250, RF_target_pos.data());
            InverseKinematics3D(y_const, z_LB, x_LB, y_const, 250, 250, LB_target_pos.data());
            InverseKinematics3D(y_const, z_RB, x_RB, y_const, 250, 250, RB_target_pos.data());
            break;
        }
        case 3: // the command to make a robot "run" (trotting)
            InverseKinematics3D(y_const, z1, x1, y_const, 250, 250, LF_target_pos.data());
            InverseKinematics3D(y_const, z2, x2, y_const, 250, 250, RF_target_pos.data());
            InverseKinematics3D(y_const, z2, x2, y_const, 250, 250, LB_target_pos.data());
            InverseKinematics3D(y_const, z1, x1, y_const, 250, 250, RB_target_pos.data());
            break;
        default:
            break;
    }
    // RCLCPP_INFO(this->get_logger(), "t: %f, count_: %f", RF_target_pos[1], RF_target_pos[2]);

    array<double, 12> target_pos;
    copy(LF_target_pos.begin(), LF_target_pos.end(), target_pos.begin());
    copy(RF_target_pos.begin(), RF_target_pos.end(), target_pos.begin() + 3);
    copy(LB_target_pos.begin(), LB_target_pos.end(), target_pos.begin() + 6);
    copy(RB_target_pos.begin(), RB_target_pos.end(), target_pos.begin() + 9);

    // Output torque to pubilsh Initialization
    vector<double> output_torque(12);

    switch (int(command[0])) {
        case 1: // the command to keep a robot "standing still"
            CalculateTorqueStanding(output_torque.data(), {Kp(0,0), Kp(1,0), Kp(2,0)}, {Kd(0,0), Kd(1,0), Kd(2,0)}, _count);
            break;
        case 2: // the command to keep a robot "walking in place"
            CalculateTorqueRunning(output_torque.data(), target_pos.data(), {Kp(0, 2), Kp(1, 2), Kp(2, 2)}, {Kd(0, 2), Kd(1, 2), Kd(2, 2)});
            break;
        case 3: // the command to make a robot "run"
            CalculateTorqueRunning(output_torque.data(), target_pos.data(), {Kp(0,1), Kp(1,1), Kp(2,1)}, {Kd(0,1), Kd(1,1), Kd(2,1)});
            break;
        default:
            fill(output_torque.begin(), output_torque.end(), 0.0);
            break;
    }


    RobotState rs;
    // rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);
    rs.set(body_pos.data(), body_vel.data(), dist.data(), body_vel.data(), joint_pos.data(), imu[2]);
    // rs.print(); // 여기다 넣으면 1ms마다 출력되는데 어케 해야되지


    // Publish Desired Pose
    std_msgs::msg::Float32MultiArray targetpos_msg;
    targetpos_msg.data.insert(targetpos_msg.data.end(), target_pos.begin(), target_pos.end());
    pub_targetpos->publish(targetpos_msg);

    // Publish Torque
    std_msgs::msg::Float32MultiArray torque_msg;
    torque_msg.data.insert(torque_msg.data.end(), output_torque.begin(), output_torque.end());
    pub_torque->publish(torque_msg);
}