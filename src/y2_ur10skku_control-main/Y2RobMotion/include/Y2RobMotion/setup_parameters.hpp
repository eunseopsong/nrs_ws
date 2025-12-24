#pragma once
#include "Y2Matrix/YMatrix.hpp"

/* Parameters for setup */
#define ROBOT_KINEMATICS 1 // 0: KUKA_IIWA, 1: UR10, 2: UR10e
#define PACKAGE_BUNDLE_DIR "/home/jay/ur10skku_ws/src/y2_ur10skku_control"
#define ROBOT_NAME "ur10skku"
#define CONTROL_PERIOD 0.008 // seconds (UR10CB3: 0.008(recommended), UR10e: 0.002)
#define NUMBER_OF_JOINTS 6

#define TEST_MODE 0 // 0: Disabled, 1: Enabled
#define REMAPPING_ENABLED 1 // 0: Disabled, 1: Enabled
#define REMAP_STATE_TOPIC "/joint_states" // Topic remapping for joint states
#define REMAP_COMMAND_TOPIC "/forward_position_controller/commands" // Topic remapping for joint commands

/* EE to TCP HTM setting (translation: mm) */
const YMatrix EE2TCP = {
    {-1.0,  0.0,  0.0,  0.0 },
    { 0.0,  1.0,  0.0,  0.0 },
    { 0.0,  0.0, -1.0,  0.0 },
    { 0.0,  0.0,  0.0,  1.0 }
};

/**************************************************/

/* Kinematics of Robot */
#if (ROBOT_KINEMATICS == 0)
    #include "Y2Kinematics/KinematicsKUKAiiwa.hpp"
    typedef KinematicsKUKAiiwa BaseKinematics;
#elif (ROBOT_KINEMATICS == 1)
    #include "Y2Kinematics/KinematicsUR10.hpp"
    typedef KinematicsUR10 BaseKinematics;
#elif (ROBOT_KINEMATICS == 2)
    #include "Y2Kinematics/KinematicsUR10e.hpp"
    typedef KinematicsUR10e BaseKinematics;
#else
    #error "Invalid ROBOT_KINEMATICS value (0: KUKA, 1: UR10)"
#endif

/* joint state naming for mapping */ 
#if (ROBOT_KINEMATICS == 0)
    #define JOINT_NAME_1 "lbr_A1"
    #define JOINT_NAME_2 "lbr_A2"
    #define JOINT_NAME_3 "lbr_A3"
    #define JOINT_NAME_4 "lbr_A4"
    #define JOINT_NAME_5 "lbr_A5"
    #define JOINT_NAME_6 "lbr_A6"
    #define JOINT_NAME_7 "lbr_A7"
#elif (ROBOT_KINEMATICS == 1)
    #define JOINT_NAME_1 "shoulder_pan_joint"
    #define JOINT_NAME_2 "shoulder_lift_joint"
    #define JOINT_NAME_3 "elbow_joint"
    #define JOINT_NAME_4 "wrist_1_joint"
    #define JOINT_NAME_5 "wrist_2_joint"
    #define JOINT_NAME_6 "wrist_3_joint"
    #define JOINT_NAME_7 ""
#elif (ROBOT_KINEMATICS == 2)
    #define JOINT_NAME_1 "shoulder_pan_joint"
    #define JOINT_NAME_2 "shoulder_lift_joint"
    #define JOINT_NAME_3 "elbow_joint"
    #define JOINT_NAME_4 "wrist_1_joint"
    #define JOINT_NAME_5 "wrist_2_joint"
    #define JOINT_NAME_6 "wrist_3_joint"
    #define JOINT_NAME_7 ""
#else
    #error "Invalid ROBOT_KINEMATICS value (0: KUKA, 1: UR10)"
#endif