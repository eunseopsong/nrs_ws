#include "Y2RobMotion/robot_motion.hpp"

void robot_motion::control_guiding()
{
    control_mode = "Guiding";
    /* Target position init - accential!!! */
    if(pre_control_mode != control_mode)
    {
        /* Motion initial */
        target_pose = current_pose;
        target_angles = current_angles;
        
        /* admittance contorl parameters */
        HG_AC_desX = current_pose; // Current_pose: mm, rad
        HG_AC_desX[0] = HG_AC_desX[0]/1000; // mm->m, AC input: m, rad
        HG_AC_desX[1] = HG_AC_desX[1]/1000; // mm->m, AC input: m, rad
        HG_AC_desX[2] = HG_AC_desX[2]/1000; // mm->m, AC input: m, rad

        std::vector<double> HG_MASS = {0.5, 0.5, 0.5, 0.1, 0.1, 0.1}; // Position(3), Orientation(3)
        std::vector<double> HG_DAMPER = {400, 400, 400, 8, 8, 8}; // Position(3), Orientation(3)
        std::vector<double> HG_STIFFNESS = {0,0,0,0,0,0}; // Position(3), Orientation(3)

        for(int i=0;i<6;i++){AControl[i].adm_1D_MDK(HG_MASS[i],HG_DAMPER[i],HG_STIFFNESS[i]);}

        RCLCPP_INFO(node_->get_logger(),"Guiding was initialized");
    }

    /**** START OF HAND-GUIDING ****/

    for(int i=0;i<6;i++)
    {AC_pose[i] = AControl[i].adm_1D_control(HG_AC_desX[i], 0.0, ft1data[i]);} // Xd(HG: previous target pose), Fd(HG: 0), Fext
    target_pose = AC_pose;
    target_pose[0] = target_pose[0]*1000; // m -> mm
    target_pose[1] = target_pose[1]*1000; // m -> mm
    target_pose[2] = target_pose[2]*1000; // m -> mm

    /**** END OF HAND-GUIDING ****/

    /* Generate target HTM */
    std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
    auto target_rot = YMatrix::fromSpatialAngle(target_ori);
    target_HTM = YMatrix::identity(4);
    target_HTM.insert(0, 0, target_rot);
    target_HTM[0][3] = target_pose[0]; // mm unit
    target_HTM[1][3] = target_pose[1]; // mm unit
    target_HTM[2][3] = target_pose[2]; // mm unit

    /* Inverse kinematics using QP-solver + execution */
    target_angles = solve_IK(target_angles, target_HTM);

    /* Data upload to past */
    pre_control_mode = control_mode; // Store previous control mode for comparison
}
