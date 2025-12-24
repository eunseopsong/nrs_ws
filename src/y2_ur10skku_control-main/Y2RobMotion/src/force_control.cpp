#include "Y2RobMotion/robot_motion.hpp"

/*** Select Force Control Mode ***/
/* 0: Classical Force Controller - static parameter, k=0 at contact*/
/* 1: FAAC Force Controller - variable parameter, MDK variation (Jay's Controller)*/
/* 2: RL Based Force Controller - mass parameter variation */
/* 3: RL Based Force Controller - mass & MD-ratio parameter variation (Jay's idea) */
#define Force_Con_Mode 3

/*** Select Force Control Coordinate ***/
/* 0: Cartesian Coordinate */
/* 1: TCP Coordinate */

#define Force_Con_Coordinate 0

void robot_motion::control_force()
{
    control_mode = "Force";

    if(Force_Con_Mode == 0) force_con_mode = "Classic";
    else if(Force_Con_Mode == 1) force_con_mode = "FAAC";
    else if(Force_Con_Mode == 2) force_con_mode = "RL-Mass";
    else if(Force_Con_Mode == 3) force_con_mode = "RL-MD";

    /* Target position init - accential!!! */
    if(pre_control_mode != control_mode)
    {
        /* Motion initial */
        target_pose = current_pose;
        target_angles = current_angles;
        
        /* admittance contorl parameters */
        FC_AC_desX = current_pose; // Current_pose: mm, rad
        FC_AC_desX[0] = FC_AC_desX[0]/1000; // mm->m, AC input: m, rad
        FC_AC_desX[1] = FC_AC_desX[1]/1000; // mm->m, AC input: m, rad
        FC_AC_desX[2] = FC_AC_desX[2]/1000; // mm->m, AC input: m, rad

        FC_MASS = {2, 2, 2, 0.5, 0.5, 0.5}; // Position(3), Orientation(3)
        FC_DAMPER = {6000, 6000, 6000, 100, 100, 100}; // Position(3), Orientation(3)
        // FC_MASS = {1, 1, 1, 0.05, 0.05, 0.05}; // Position(3), Orientation(3)
        // FC_DAMPER = {3000, 3000, 3000, 10, 10, 10}; // Position(3), Orientation(3)
        FC_STIFFNESS = {2000,2000,2000,100,100,100}; // Position(3), Orientation(3)

        for(int i=0;i<6;i++){AControl[i].adm_1D_MDK(FC_MASS[i],FC_DAMPER[i],FC_STIFFNESS[i]);}

        /* FAAC initialization */
        for(int i=0;i<3;i++){FAAC3step[i]->FAAC_Init(FC_MASS[i],FC_DAMPER[i],FC_STIFFNESS[i]);}

        RCLCPP_INFO(node_->get_logger(),"Guiding was initialized");
    }

    /**** START OF FORCE-CONTROL ****/

    for(int i=0;i<6;i++) // Xd(HG: previous target pose), Fd(HG: 0), Fext
    {
        #if(Force_Con_Mode == 0) // Classical Force Controller
            /* Position - Force control */
            if(i<3)
            {
                /* if it contact with surf. -> k=0 */
                if(fabs(FC_AC_desX[i+6]) > 0.01) {AControl[i].adm_1D_MDK(FC_MASS[i],FC_DAMPER[i],0.0);}
                /* k recovery */
                else{
                    if(AControl[i].adm_MDK_monitor(2) != FC_STIFFNESS[i])
                    {
                        double tau_k = 3.0; // Recovery time (s)
                        double alpha = 1.0 - std::exp(-Control_period_/ tau_k);
                        double target_k = AControl[i].adm_MDK_monitor(2) + alpha * (FC_STIFFNESS[i] - AControl[i].adm_MDK_monitor(2));

                        AControl[i].adm_1D_MDK(FC_MASS[i],FC_DAMPER[i],target_k);
                    }
                }

                /* Force control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], FC_AC_desX[i+6], ft1data[i]);
            }
            /* Orientation - Position control */
            else
            {
                /* Position control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, ft1data[i]);
            }
        #elif(Force_Con_Mode == 1) // FAAC Force Controller
            /* Position - Force control */
            if(i<3)
            {
                /* FAAC flag (if desired force not zero -> flag on -> FAAC control) */
                FAAC_flag[i] = ((fabs(FC_AC_desX[i+6]) > 0.01) || FAAC_flag[i])? true:false;
                FAAC_flag[abs(i-1)] = (FAAC_flag[i])? false:FAAC_flag[abs(i-1)];
                FAAC_flag[abs(i-2)] = (FAAC_flag[i])? false:FAAC_flag[abs(i-2)];
                
                /* In the case of flag true */
                if (FAAC_flag[i]){
                    /* Three Step FAAC MDK update */
                    double Tank_energy = 5; // in here, we fix the tank energy (must be revised)
                    /* FAAC_MDKob_RUN(Tank_energy(J),Ext_Force(N),Dis_Force(N),AC_Pose(m),Act_Pose(m)) */
                    auto TSFAAC_MDK = FAAC3step[i]->FAAC_MDKob_RUN(Tank_energy, ft1data[i], FC_AC_desX[i+6], AC_pose[i], current_pose[i]/1000);
                    AControl[i].adm_1D_MDK(TSFAAC_MDK.Mass,TSFAAC_MDK.Damping,TSFAAC_MDK.Stiffness);
                }
                
                /* Force control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], FC_AC_desX[i+6], ft1data[i]);
            }
            /* Orientation - Position control */
            else
            {
                /* Position control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, ft1data[i]);
            }

        #elif(Force_Con_Mode == 2) // RL Based Mass Variation
            /* Position - Force control */
            if(i<3)
            {
                /* if it contact with surf. -> MDK Variation */
                if(fabs(FC_AC_desX[i+6]) > 0.01) 
                {
                    auto RL_mass = policy_mass[i]->run(AC_pose[i], current_pose[i]/1000, FC_AC_desX[i+6], ft1data[i]); // xc, x, Fd, Env_Fext
                    // double MD_Ratio = FC_DAMPER[i]/FC_MASS[i];
                    double MD_Ratio = 3000;
                    AControl[i].adm_1D_MDK(RL_mass,MD_Ratio*RL_mass,0.0);
                }
                /* mdk recovery */
                else{
                    if(AControl[i].adm_MDK_monitor(2) != FC_STIFFNESS[i])
                    {
                        double tau_k = 3.0; // Recovery time (s)
                        double alpha = 1.0 - std::exp(-Control_period_/ tau_k);

                        std::vector<double> target_mdk(3);
                        target_mdk[0] = AControl[i].adm_MDK_monitor(0) + alpha * (FC_MASS[i] - AControl[i].adm_MDK_monitor(0));
                        target_mdk[1] = AControl[i].adm_MDK_monitor(1) + alpha * (FC_DAMPER[i] - AControl[i].adm_MDK_monitor(1));
                        target_mdk[2] = AControl[i].adm_MDK_monitor(2) + alpha * (FC_STIFFNESS[i] - AControl[i].adm_MDK_monitor(2));

                        AControl[i].adm_1D_MDK(target_mdk[0],target_mdk[1],target_mdk[2]);
                    }
                }

                /* Force control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], FC_AC_desX[i+6], ft1data[i]);
            }
            /* Orientation - Position control */
            else
            {
                /* Position control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, ft1data[i]);
            }


        #elif(Force_Con_Mode == 3) // RL Based Mass & MD-Ratio Variation
            /* Position - Force control */
            if(i<3)
            {
                /* if it contact with surf. -> MDK Variation */
                if(fabs(FC_AC_desX[i+6]) > 0.01) 
                {
                    auto policy_md_out 
                    = policy_md[i]->run(AC_pose[i], current_pose[i]/1000, FC_AC_desX[i+6], ft1data[i]); // xc, x, Fd, Env_Fext
                    // double MD_Ratio = FC_DAMPER[i]/FC_MASS[i];
                    double MD_Ratio = 1000;
                    AControl[i].adm_1D_MDK(policy_md_out[0],MD_Ratio*policy_md_out[0]*policy_md_out[1],0.0);
                }
                /* mdk recovery */
                else{
                    if(AControl[i].adm_MDK_monitor(2) != FC_STIFFNESS[i])
                    {
                        double tau_k = 3.0; // Recovery time (s)
                        double alpha = 1.0 - std::exp(-Control_period_/ tau_k);

                        std::vector<double> target_mdk(3);
                        target_mdk[0] = AControl[i].adm_MDK_monitor(0) + alpha * (FC_MASS[i] - AControl[i].adm_MDK_monitor(0));
                        target_mdk[1] = AControl[i].adm_MDK_monitor(1) + alpha * (FC_DAMPER[i] - AControl[i].adm_MDK_monitor(1));
                        target_mdk[2] = AControl[i].adm_MDK_monitor(2) + alpha * (FC_STIFFNESS[i] - AControl[i].adm_MDK_monitor(2));

                        AControl[i].adm_1D_MDK(target_mdk[0],target_mdk[1],target_mdk[2]);
                    }
                }

                /* Force control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], FC_AC_desX[i+6], ft1data[i]);
            }
            /* Orientation - Position control */
            else
            {
                /* Position control using AC */
                // AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, ft1data[i]);
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, 0.0);
            }
        
        #endif
    } 
    target_pose = AC_pose;
    target_pose[0] = target_pose[0]*1000; // m -> mm
    target_pose[1] = target_pose[1]*1000; // m -> mm
    target_pose[2] = target_pose[2]*1000; // m -> mm

    /**** END OF FORCE-CONTROL ****/

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