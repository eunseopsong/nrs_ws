#include "Yoon_force_control.h"

bool Yoon_force_control::adm_1D_MDK(double Mass, double Damping, double Stiffness)
{
    adm_1D_M = Mass;
    adm_1D_D = Damping;
    adm_1D_K = Stiffness;

    return true;
}
bool Yoon_force_control::adm_1D_init(double Perror_init,double Ferror_init,double Sampling_time)
{
    for(int i=0;i<3;i++)
    {
        adm_1D_Perror[i] = Perror_init;
    }

    adm_1D_dt = Sampling_time;
    return true;
}

double Yoon_force_control::adm_1D_control(double xd, double Fd, double Fext)
{
    adm_1D_Ferror[0] = Fd - Fext;

    adm_1D_A = 4*adm_1D_M - 2*adm_1D_dt*adm_1D_D + pow(adm_1D_dt,2)*adm_1D_K;
    adm_1D_B = 2*pow(adm_1D_dt,2)*adm_1D_K - 8*adm_1D_M;
    adm_1D_C = 4*adm_1D_M + 2*adm_1D_dt*adm_1D_D + pow(adm_1D_dt,2)*adm_1D_K;

    xc = xd + (1/adm_1D_C)*(adm_1D_B*adm_1D_Perror[1] + adm_1D_A*adm_1D_Perror[2]
    -pow(adm_1D_dt,2)*(adm_1D_Ferror[0]+2*adm_1D_Ferror[1]+adm_1D_Ferror[2]));

    //parameter update to past
    adm_1D_Perror[0] = xd - xc;
    adm_1D_Perror[2] = adm_1D_Perror[1];
    adm_1D_Perror[1] = adm_1D_Perror[0];

    adm_1D_Ferror[2] = adm_1D_Ferror[1];
    adm_1D_Ferror[1] = adm_1D_Ferror[0];

    // Return the postion output
    return xc;
}


double E_tank_Hand_Guiding::Energy_tank()
{
    // policy of tank energy charging
    if(tank_energy <= tank_Ulim) whi = 1;
    else whi = 0;

    // policy of tank energy dissipatation
    if(Md_dot <= 0) gamma = whi;
    else gamma = 1;

    // total charging energy from damper
    dampE_sum += (whi)*Dd*current_vel*current_vel;
    // total dissipation energy from mass increment
    massCE_sum += (gamma)*0.5*Md_dot*current_vel*current_vel;
    
    // total stored energy
    tank_energy = init_energy + dampE_sum - massCE_sum;

    // Return the tank energy value
    return tank_energy;
}


// -------- Posture playback start ------- //

void DS_power_playback::playback_init(Eigen::Vector3d Current_pos, Eigen::Vector3d Current_ori)
{
    /* 1) Non Real-time inputs set (but must be set, might be changed): */
    Ts = 0.002; // Control loop time (unit: s)

    /* Position */
    PTiniE = 2; // Energy tank init value (unit: J)
    PTankE = PTiniE; // Tank energy initialization
    PTankMax = 10; // Energy tank maximum energy (unit: J)
    PTankMin = 0.1; // Energy tank minimum energy (unit: J)

    /* Orientation */
    OTiniE = 2; // Energy tank init value (unit: J)
    OTankE = OTiniE; // Tank energy initialization
    OTankMax = 5; // Energy tank maximum energy (unit: J)
    OTankMin = 0.1; // Energy tank minimum energy (unit: J)

    /* 2) Parameter initialization */

    /* Position */
    PXr_pre = Current_pos;
    PX_pre = Current_pos;
    PXc_0 = Current_pos; //test

    PU1 << 1,0,0;
    PU1_pre << 1,0,0;
    PU3 << 0,0,1;
    PU3_pre << 0,0,1;

    Vapproach << 0,0,1;
    Vapproach_pre << 0,0,1;

    Pe1 << 1,0,0;
    Pe1_pre << 1,0,0;
    Pe3 << 0,0,1;
    Pe3_pre << 0,0,1;

    PA_0 << 0,0,0;
    PA_1 << 0,0,0;
    PA_2 << 0,0,0;

    PXc_0 = Current_pos;
    PXc_1 = Current_pos;
    PXc_2 = Current_pos;

    /* Orientation */
    OXr_pre = Current_ori;
    OX_pre = Current_ori;

    OA_0 << 0,0,0;
    OA_1 << 0,0,0;
    OA_2 << 0,0,0;

    OXc_0 = Current_ori;
    OXc_1 = Current_ori;
    OXc_2 = Current_ori;

    PB_init_flag = true;
}

void DS_power_playback::PU_calculation()
{
    #if 1 // original
    /*PU1 calculation*/
    if(PXr_dot.norm() != 0) 
    {
        PU1 = PXr_dot/PXr_dot.norm();
        PU1_pre = PU1;
    }
    else {PU1 = PU1_pre;}
    /*PU3 calculation start*/
    PZ_ori = PRtz; // Tool Z direction used in PU3 (if EE heading to surf -> PRtz = [0,0,1])
    PU1_rot_axis = PU1.cross(PZ_ori);
    if(PU1_rot_axis.norm() != 0)
    {
        PU1_rot_axis = PU1_rot_axis/PU1_rot_axis.norm();
        PU1_rot_mrx = AA_ref.angle_axis_representation(PU1_rot_axis,90*(PI/180));// 90degree rotation
        PU3 = PU1_rot_mrx*PU1;
        PU3 = PU3/PU3.norm();
        PU3_pre = PU3;
    }
    else
    {PU3 = PU3_pre;}
    #endif

    #if 0 // 2024 NIST experimnet
    /*PU1 calculation*/
    // if(PXr_dot.norm() != 0) 
    // {
    //     PU1 = PXr_dot/PXr_dot.norm();
    //     PU1_pre = PU1;
    // }
    // else {PU1 = PU1_pre;}
    // /*PU3 calculation start*/
    // PZ_ori = PRtz; // Tool Z direction used in PU3 (if EE heading to surf -> PRtz = [0,0,1])
    // PU1_rot_axis = PU1.cross(PZ_ori);
    // if(PU1_rot_axis.norm() != 0)
    // {
    //     PU1_rot_axis = PU1_rot_axis/PU1_rot_axis.norm();
    //     PU1_rot_mrx = AA_ref.angle_axis_representation(PU1_rot_axis,90*(PI/180));// 90degree rotation
    //     PU3 = PU1_rot_mrx*PU1;
    //     PU3 = PU3/PU3.norm();
    //     PU3_pre = PU3;
    // }
    // else
    // {PU3 = PU3_pre;}

    if(PFd.norm() != 0)
    {
        int contact_dir = 1; //0:x, 1:y, 2:z

        if(contact_dir == 0) // x-directional contact
        {
            PU3 << 0.609,-0.796,-0.0224;
            PU1 << 0,0,1; //Exp1
        }
        else if(contact_dir == 1)
        {
            PU3 << -0.703,-0.711,-0.0026;
            PU1 << 0,0,1; //Exp1s
        }
        else
        {
            PU3 << 0,0,1;
            PU1 << 1,0,0; //Exp1s
        }

        PU1_pre = PU1;
        PU3_pre = PU3;
    }
    else
    {
        PU1 << P_Tool_Rot(0,0),P_Tool_Rot(1,0),P_Tool_Rot(2,0);
        PU3 << P_Tool_Rot(0,2),P_Tool_Rot(1,2),P_Tool_Rot(2,2);
    }

    #endif

    #if 0 // revised at 25.01.07 (static-xyz contact, dynamic-z contact situation possible)
    if(PXr_dot.norm() != 0) 
    {
        /* PU1 calculation */
        PU1 = PXr_dot/PXr_dot.norm();
        PU1_pre = PU1;

        /* PU3 calculation */
        PZ_ori = PRtz; // Tool Z direction used in PU3 (if EE heading to surf -> PRtz = [0,0,1])
        PU1_rot_axis = PU1.cross(PZ_ori);
        if(PU1_rot_axis.norm() != 0)
        {
            PU1_rot_axis = PU1_rot_axis/PU1_rot_axis.norm();
            PU1_rot_mrx = AA_ref.angle_axis_representation(PU1_rot_axis,90*(PI/180));// 90 degree rotation
            PU3 = PU1_rot_mrx*PU1;
            PU3 = PU3/PU3.norm();
            PU3_pre = PU3;
        }
        else
        {PU3 = PU3_pre;}


        if((PFd.norm() != 0) && (PFext.norm() > Fext_threshold)) {Vapproach = PU3;}
        else {Vapproach = -PU1;}

    }
    else
    {
        if(PFd.norm() != 0) // Xd가 0이며, Fd 는 0이 아닐때
        {   
            
            PU3 << fabs(Vapproach(0)),fabs(Vapproach(1)),fabs(Vapproach(2));
            if(PU3.maxCoeff() == PU3(2)) {VpreU1 << 1,0,0;}
            else {VpreU1 << 0,0,1;}

            /* PU3 calculation */
            PU3 = Vapproach;

            /* PU1 calculation */
            VrotU3 = PU3.cross(VpreU1);
            VrotU3 = VrotU3/VrotU3.norm();

            PU3_rot_mrx = AA_ref.angle_axis_representation(VrotU3,90*(PI/180));// 90degree rotation
            PU1 = PU3_rot_mrx*PU3;

            PU1_pre = PU1;
            PU3_pre = PU3;
        }
        else
        {
            PU1 = PU1_pre;
            PU3 = PU3_pre;
        }
        
    }
    #endif
    
    /*PU2 calculation*/
    PU2 = PU3.cross(PU1);

    /*PU calculation*/
    PU << PU1,PU2,PU3;
}


int DS_power_playback::playback_start(DS_power_PB_RTinput RTinput)
{
    if(PB_init_flag == true)
    {
        /* STEP0) RTinput upload */
    
        /* Position */
        PXr = RTinput.PXr;
        PX = RTinput.PX;
        PRtx = RTinput.PRtx;
        PRtz = RTinput.PRtz;
        PFext = RTinput.PFext;
        P_Tool_Rot = RTinput.P_Tool_Rot;
        PFd(2) = RTinput.PFd;
        PXr_dot = (PXr - PXr_pre)/Ts;
        PX_dot = (PX - PX_pre)/Ts;

        PXr_pre = PXr;
        PX_pre = PX;

        /* Orientation */
        OXr = RTinput.OXr;
        OX = RTinput.OX;
        ORtz = RTinput.ORtz;
        OFext = RTinput.OFext;
        OXr_dot = (OXr - OXr_pre)/Ts;
        OX_dot = (OX - OX_pre)/Ts;

        OXr_pre = OXr;
        OX_pre = OX;


        /* STEP1) Task generation */

        /* 1) Spring behavior task fr calculation */

        /*--Position--*/
        /* PU calculation */
        PU_calculation();
        /* RamdaK matrix generation */
        PRamK_mrx << PRamK[0],0,0, 0,PRamK[1],0, 0,0,PRamK[2];
        /* PKd calculation */
        PKd = PU*PRamK_mrx*PU.transpose(); 
        /*Pfr calculation*/
        Pfr = (1/PRamD[0])*PKd*(PXr-PX); // stiffness control mode

        /*--Orientation--*/
        /*OU1 calculation*/
        if(OXr_dot.norm() != 0) OU1 = OXr_dot/OXr_dot.norm();
        else OU1 << 1,0,0;
        /*OU3 calculation start*/
        // OZ_ori << 0,0,1; // Z plus direction used in OU3
        OZ_ori = ORtz; // Tool Z direction used in PU3 (if EE heading to surf -> PRtz = [0,0,1])
        OU1_rot_axis = OU1.cross(OZ_ori);
        if(OU1_rot_axis.norm() != 0)
        {
            OU1_rot_axis = OU1_rot_axis/OU1_rot_axis.norm();
            OU1_rot_mrx = AA_ref.angle_axis_representation(OU1_rot_axis,90*(PI/180));// 90degree rotation
            OU3 = OU1_rot_mrx*OU1;
            OU3 = OU3/OU3.norm();
        }
        else OU3<<1,0,0;
        /*OU3 calculation end*/
        /*OU2 calculation*/
        OU2 = OU3.cross(OU1);

        /*OU calculation*/
        OU << OU1,OU2,OU3;
        /*RamdaK matrix generation*/
        ORamK_mrx << ORamK[0],0,0, 0,ORamK[1],0, 0,0,ORamK[2];
        /*OKd calculation*/
        OKd = OU*ORamK_mrx*OU.transpose();
        /*Ofr calculation*/
        Ofr = (1/ORamD[0])*OKd*(OXr-OX);

        /* 2) Force generation task fd calculation */
        /*--Position--*/
        PFd_hat = PFd(2)*PU3;
        Pfn = -(1/PRamD[0])*PFd_hat;
        /*--Orientation--*/
        OFd = RTinput.OFd*OU3;
        Ofn = -(1/ORamD[0])*OFd;

        /* 3) Total task fx calculation */
        /*--Position--*/
        Pfx = Pfr + Pfn;
        /*--Orientation--*/
        Ofx = Ofr + Ofn;

        /* STEP2) Calculate state-varying damping matrix */

        /*--Position--*/

        /* 1) PRamD matrix generation */
        PRamD_mrx << PRamD[0],0,0, 0,PRamD[1],0, 0,0,PRamD[2];

        /* 2) PDd matrix calculation */
        PDd = PU*PRamD_mrx*PU.transpose(); // State-varing damping matrix

        /* 3) PMd matrix calculation */
        /* RamdaK matrix generation */
        PRamM_mrx << PRamM[0],0,0, 0,PRamM[1],0, 0,0,PRamM[2];
        /* PKd calculation */
        PMd = PU*PRamM_mrx*PU.transpose();

        /*--Orientation--*/

        /* 1) OQ matrix generation */
        /* Oe1 calculation */
        #if 0
        if(Ofx.norm() != 0) Oe1 = Ofx/Ofx.norm();
        else Oe1 << 1,0,0;
        #endif

        if(OXr_dot.norm() != 0) Oe1 = OXr_dot/OXr_dot.norm();
        else Oe1 << 1,0,0;
        /* Oe3 calculation */
        Oe1_rot_axis = Oe1.cross(OZ_ori);
        if(Oe1_rot_axis.norm() != 0)
        {
            Oe1_rot_axis = Oe1_rot_axis/Oe1_rot_axis.norm();
            Oe1_rot_mrx = AA_ref.angle_axis_representation(Oe1_rot_axis,90*(PI/180));// 90degree rotation
            Oe3 = Oe1_rot_mrx*Oe1;
        }
        else Oe3<<1,0,0;
        /* Oe2 calculation */
        Oe2 = Oe3.cross(Oe1);
        /* OQ matrix calculation */
        OQ << Oe1, Oe2, Oe3;

        /* 2) ORamD matrix generation */
        ORamD_mrx << ORamD[0],0,0,
                        0,ORamD[1],0,
                        0,0,ORamD[2];

        /* 3) ODd matrix calculation */
        ODd = OQ*ORamD_mrx*OQ.transpose(); // State-varing damping matrix

        /* 4) OMd matrix calculation */
        /*RamdaK matrix generation*/
        ORamM_mrx << ORamM[0],0,0, 0,ORamM[1],0, 0,0,ORamM[2];
        /*PKd calculation*/
        OMd = OU*ORamM_mrx*OU.transpose();

        /* STEP3) Calculate energy tank */

        /*--Position--*/
        /* Calculate pd */
        Ppd = PX_dot.transpose()*PDd*PX_dot;
        /* Calculate pr */
        Ppr = PRamD[0]*PX_dot.transpose()*Pfr;
        /* Calculate pn */
        Ppn = PRamD[0]*PX_dot.transpose()*Pfn;

        /* Tank alpha calculation (For smooth charging) */
        if(PTankE < PTankMin) PTAl = 1;
        else if((PTankE <= PTankMax)&&(PTankE >= PTankMin))
        {PTAl = 0.5*(1+cos(PI*(PTankE-PTankMin)/(PTankMax-PTankMin)));}
        else PTAl = 0;
        /* Tank Beta r calculation (For smooth charging) */
        if((PTankE-Ppr < PTankMin)&&(Ppr > 0)) PTBr = 0;
        else if((PTankE-Ppr > PTankMax)&&(Ppr < 0)) PTBr = 0;
        else PTBr = 1;
        /* Tank Beta n calculation (For smooth charging) */
        if((PTankE-Ppn < PTankMin)&&(Ppn > 0)) PTBn = 0;
        else if((PTankE-Ppn > PTankMax)&&(Ppn < 0)) PTBn = 0;
        else PTBn = 1;

        /* Calculate pd integral */
        Ppd_integ += PTAl*Ppd;
        /* Calculate pr integral */
        Ppr_integ += PTBr*Ppr;
        /* Calculate pn integral */
        Ppn_integ += PTBn*Ppn;

        /* Energy tank value update */
        PTankE = PTiniE + Ppd_integ - Ppr_integ - Ppn_integ;

        /*--Orientation--*/

        /* Calculate pd */
        Opd = OX_dot.transpose()*ODd*OX_dot;
        /* Calculate pr */
        Opr = ORamD[0]*OX_dot.transpose()*Ofr;
        /* Calculate pn */
        Opn = ORamD[0]*OX_dot.transpose()*Ofn;

        /* Tank alpha calculation (For smooth charging) */
        if(OTankE < OTankMin) OTAl = 1;
        else if((OTankE <= OTankMax)&&(OTankE >= OTankMin))
        {OTAl = 0.5*(1+cos(PI*(OTankE-OTankMin)/(OTankMax-OTankMin)));}
        else OTAl = 0;
        /* Tank Beta r calculation (For smooth charging) */
        if((OTankE < OTankMin)&&(Opr > 0)) OTBr = 0;
        else if((OTankE > OTankMax)&&(Opr < 0)) OTBr = 0;
        else OTBr = 1;
        /* Tank Beta n calculation (For smooth charging) */
        if((OTankE < OTankMin)&&(Opn > 0)) OTBn = 0;
        else if((OTankE > OTankMax)&&(Opn < 0)) OTBn = 0;
        else OTBn = 1;

        /* Calculate pd integral */
        Opd_integ += OTAl*Opd;
        /* Calculate pr integral */
        Opr_integ += OTBr*Opr;
        /* Calculate pn integral */
        Opn_integ += OTBn*Opn;

        /* Energy tank value update */
        OTankE = OTiniE + Opd_integ - Opr_integ - Opn_integ;

        /* STEP4) Reshape the task */

        /* Position */
        /* PBeta r dat calculation*/
        if(Ppr < 0) PTBr_dat = 1;
        else PTBr_dat = PTBr;
        /* PBeta n dat calculation*/
        if(Ppn < 0) PTBn_dat = 1;
        else PTBn_dat = PTBn;

        /* Reshape the Ptask */
        Pfx = PTBr_dat*Pfr + PTBn_dat*Pfn;

        /* Orientation */
        /* OBeta r dat calculation*/
        if(Opr < 0) OTBr_dat = 1;
        else OTBr_dat = OTBr;
        /* OBeta n dat calculation*/
        if(Opn < 0) OTBn_dat = 1;
        else OTBn_dat = OTBn;

        /* Reshape the Otask */
        Ofx = OTBr_dat*Ofr + OTBn_dat*Ofn;


        /* STEP5) Solve the differential equation with tustin method */

        /* Position */
        /*Pdif_U calculation*/
        Pdif_U = 4*PMd - 2*Ts*PDd;
        /*Pdif_V calculation*/
        Pdif_V = -8*PMd;
        /*Pdif_W calculation*/
        Pdif_W = 4*PMd + 2*Ts*PDd;

        /* PA_0 calculation */
        PA_0 = PFext + (PRamD[0])*Pfx;
        // PA_0 = PDd*Pfx;

        /* Xc calculation */
        PXc_0 = Pdif_W.inverse()*(pow(Ts,2)*(PA_2+2*PA_1+PA_0)-Pdif_U*PXc_2-Pdif_V*PXc_1); // Final output

        /* Update to past */
        #if 1 // original version
        PXc_2 = PXc_1;
        PXc_1 = PXc_0;
        #endif 
        #if 0 // Test version
        PXc_2 = PXc_1;
        PXc_1 = PX;
        #endif 
        
        PA_2 = PA_1;
        PA_1 = PA_0;

        /* Orientation */
        /*Odif_U calculation*/
        Odif_U = 4*OMd - 2*Ts*ODd;
        /*Pdif_V calculation*/
        Odif_V = -8*OMd;
        /*Pdif_W calculation*/
        Odif_W = 4*OMd + 2*Ts*ODd;

        /* PA_0 calculation */
        OA_0 = OFext + ODd*Ofx;

        /* Xc calculation */
        OXc_0 = Odif_W.inverse()*(pow(Ts,2)*(OA_2+2*OA_1+OA_0)-Odif_U*OXc_2-Odif_V*OXc_1); // Final output

        /* Update to past */
        OXc_2 = OXc_1;
        OXc_1 = OXc_0;
        
        OA_2 = OA_1;
        OA_1 = OA_0;

        return 1;
    }
    else return 0;
}

Fuzzy_adaptive_k::Fuzzy_adaptive_k(const char Exper_Env[])
{
    // In here, sigma == ehta
    if (strcmp(Exper_Env,"Fs_Cf") == 0) // Flat surface & Constant force
    {
        FEDMM = 22.5; // Force Error dot Min. Max.
        FEMM = 0.55; // Force Error Min. Max.
        DKMM = 300; // Delta K Min. Max.
        DEMM = 0.003; // Delta Etha Min. Max.
    }
    else if (strcmp(Exper_Env,"Ss_Cf") == 0) // Sloped surface & Constant force
    {
        FEDMM = 23; // Force Error dot Min. Max.
        FEMM = 0.7; // Force Error Min. Max.
        DKMM = 800; // Delta K Min. Max.
        DEMM = 0.006; // Delta Etha Min. Max.
    }
    else if (strcmp(Exper_Env,"Ss_Vf") == 0) // Sloped surface & Variable force
    {
        FEDMM = 18; // Force Error dot Min. Max.
        FEMM = 0.86; // Force Error Min. Max.
        DKMM = 1000; // Delta K Min. Max.
        DEMM = 0.006; // Delta Etha Min. Max.
    }
    else if (strcmp(Exper_Env,"Cs_Cf") == 0) // Curved surface & Constant force
    {
        FEDMM = 18; // Force Error dot Min. Max.
        FEMM = 0.82; // Force Error Min. Max.
        DKMM = 800; // Delta K Min. Max.
        DEMM = 0.014; // Delta Etha Min. Max.
    }
    else if (strcmp(Exper_Env,"Cs_Vf") == 0) // Curved surface & Variable force
    {
        FEDMM = 15; // Force Error dot Min. Max.
        FEMM = 0.82; // Force Error Min. Max.
        DKMM = 300; // Delta K Min. Max.
        DEMM = 0.016; // Delta Etha Min. Max.
    }
    else if (strcmp(Exper_Env,"GE") == 0) // Grinding Experiment
    {
        FEDMM = 22.5; // Force Error dot Min. Max.
        FEMM = 0.55; // Force Error Min. Max.
        DKMM = 500; // Delta K Min. Max.
        DEMM = 0.005; // Delta Etha Min. Max.
    }
    else
    { 
        fprintf(stderr, "Invalid Exper_Env option: %s\n", Exper_Env);
        exit(EXIT_FAILURE);
    }
    
}

double Fuzzy_adaptive_k::FAAC_DelK_Cal(double F_Err, double FDot_Err)
{
    double FAAC_Nor_input1 = FDot_Err; // Force Error Dot input
    double FAAC_Nor_input2 = F_Err; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= FEDMM) {FAAC_Nor_input1 = FEDMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -FEDMM) {FAAC_Nor_input1 = -FEDMM;}

    if (FAAC_Nor_input2 >= FEMM) {FAAC_Nor_input2 = FEMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -FEMM) {FAAC_Nor_input2 = -FEMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-FEDMM,FEDMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-FEMM,FEMM,-FAAC_MinMax,FAAC_MinMax);
    
    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= FAAC_array_size[0]) {index1 = FAAC_array_size[0];}
    if(index2 >= FAAC_array_size[1]) {index2 = FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = FAAC_DelK_array[index1][index2];

    // Step3 : Output denormalization
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DKMM,DKMM);
}

double Fuzzy_adaptive_k::FAAC_DelSigma_Cal(double F_Err, double FDot_Err)
{
    double FAAC_Nor_input1 = FDot_Err; // Force Error Dot input
    double FAAC_Nor_input2 = F_Err; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= FEDMM) {FAAC_Nor_input1 = FEDMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -FEDMM) {FAAC_Nor_input1 = -FEDMM;}

    if (FAAC_Nor_input2 >= FEMM) {FAAC_Nor_input2 = FEMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -FEMM) {FAAC_Nor_input2 = -FEMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-FEDMM,FEDMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-FEMM,FEMM,-FAAC_MinMax,FAAC_MinMax);

    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= FAAC_array_size[0]) {index1 = FAAC_array_size[0];}
    if(index2 >= FAAC_array_size[1]) {index2 = FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = FAAC_DelEtha_array[index1][index2];

    // Step3 : Output denormalization - DelK & DelEtha 
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DEMM,DEMM);
}

double Fuzzy_adaptive_k::FAAC_Normalize(double Input,double Input_Min,double Input_Max,double FAAC_Min,double FAAC_Max)
{
    double Gamma, Normalized_out;
    Gamma = ((FAAC_Max)-(FAAC_Min))/(Input_Max-Input_Min);
    Normalized_out = ((FAAC_Max)+(FAAC_Min))/2 + Gamma*(Input - (Input_Max+Input_Min)/2);
    return Normalized_out;
}


Fuzzy_adaptive_md::Fuzzy_adaptive_md(const char Exper_Env[], double Init_Md, double Init_Dd, double Init_Kd, double Ts, double HPF_cf, double MdDdR_threshold)
{
    // HPF_cf : Cut-off frequency of High-Pass Filter (Unit: Hz)
    // MdDdR_threshold : Threshold value for Mass & MassDamper Ratio (Unit : N)

    /* Parameters used in Kd variation */
    F_ExtThr = 2; // Threshold value for external force (Unit: N)
    Kd_recov_dist = 0.03; // Recovery distance for Kd (Unit: m)

    /* Desired Mass Limit (Very important) */
    FAAC_Md_Limit[0] = 0.5; // Lower limit of Mass (Unit: kg)
    FAAC_Md_Limit[1] = 5; // Upper limit of Mass (Unit: kg)

    /* Desired MDR Limit (Very important) */
    FAAC_MDR_Limit[0] = 1; // Lower limit of MDR (Multiplication of Md & Dd)
    FAAC_MDR_Limit[1] = 20; // Upper limit of Mass (Multiplication of Md & Dd)

    /* Min. & Max. setting used in fuzzification */
    if (strcmp(Exper_Env,"cu") == 0) // Flat surface & Constant force
    {
        /* Crisp input limitation */
        FEDMM = 255.0; // Force Error dot Min. Max. (previous 255)
        FEMM = 5.5; // Force Error Min. Max.
        HPFFMM = 1.0; // High-Pass Filtered Measured Force Min. Max.

        /* Crisp output limitation */
        DMMM = 0.5; // Delta M Min. Max. (0.5 이상 금지)
        DMDRatio_MM = 0.5; // Delta MDRatio Min. Max.
    }
    else
    { 
        fprintf(stderr, "Invalid Exper_Env option: %s\n", Exper_Env);
        exit(EXIT_FAILURE);
    }

    Init_Kd_ = Init_Kd;
    Updated_Md = Init_Md;
    Updated_MDR = 1.0; // Initial value (Fix)
    Init_MDR = Init_Dd/Init_Md;
    FAAC_Ts = Ts;
    FAAC_HPF_cf = HPF_cf;
    FAAC_MdDdR_threshold = MdDdR_threshold;
    Zh0 = Eigen::Vector3d::Zero();
    Zm = Eigen::Vector3d::Zero();
}

double Fuzzy_adaptive_md::FAAC_DelM_Cal(double F_Err, double FDot_Err)
{
    double FAAC_Nor_input1 = FDot_Err; // Force Error Dot input
    double FAAC_Nor_input2 = F_Err; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= FEDMM) {FAAC_Nor_input1 = FEDMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -FEDMM) {FAAC_Nor_input1 = -FEDMM;}

    if (FAAC_Nor_input2 >= FEMM) {FAAC_Nor_input2 = FEMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -FEMM) {FAAC_Nor_input2 = -FEMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-FEDMM,FEDMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-FEMM,FEMM,-FAAC_MinMax,FAAC_MinMax);
    
    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= FAAC_array_size[0]) {index1 = FAAC_array_size[0];}
    if(index2 >= FAAC_array_size[1]) {index2 = FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = FAAC_DelM_array[index1][index2];

    // Step3 : Output denormalization
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DMMM,DMMM);
}

double Fuzzy_adaptive_md::FAAC_DelMDRatio_Cal(double F_Err, double F_H)
{
    double FAAC_Nor_input1 = F_H; // High-Pass Filtered Measured Force input
    double FAAC_Nor_input2 = F_Err; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= HPFFMM) {FAAC_Nor_input1 = HPFFMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -HPFFMM) {FAAC_Nor_input1 = -HPFFMM;}

    if (FAAC_Nor_input2 >= FEMM) {FAAC_Nor_input2 = FEMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -FEMM) {FAAC_Nor_input2 = -FEMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-HPFFMM,HPFFMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-FEMM,FEMM,-FAAC_MinMax,FAAC_MinMax);
    
    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= FAAC_array_size[0]) {index1 = FAAC_array_size[0];}
    if(index2 >= FAAC_array_size[1]) {index2 = FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = FAAC_DelMDRatio_array[index1][index2];

    // Step3 : Output denormalization
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DMDRatio_MM,DMDRatio_MM);
}

double Fuzzy_adaptive_md::FAAC_HPFInput_Cal(double F_ext, double HPF_cf, double HPF_Ts, double MdDdR_threshold)
{

    double HPF_wc = 2 * M_PI * HPF_cf;

    HPF_Fext_in[0] = F_ext;

    HPF_Fext_out[0] = (1 / (HPF_wc * HPF_Ts + 2)) * (2 * HPF_Fext_in[0] - 2 * HPF_Fext_in[1] - (HPF_wc * HPF_Ts - 2) * HPF_Fext_out[1]);

    HPF_Fext_out[1] = HPF_Fext_out[0];
    HPF_Fext_in[1] = HPF_Fext_in[0];

    double Fuzzy2_HPF_Fext = HPF_Fext_out[0];

    Fuzzy2_HPF_Fext = (std::abs(Fuzzy2_HPF_Fext) >= MdDdR_threshold) ? (Fuzzy2_HPF_Fext - std::copysign(MdDdR_threshold, Fuzzy2_HPF_Fext)) : 0;

    return Fuzzy2_HPF_Fext;
}


double Fuzzy_adaptive_md::FAAC_Normalize(double Input,double Input_Min,double Input_Max,double FAAC_Min,double FAAC_Max)
{
    double Gamma, Normalized_out;
    Gamma = ((FAAC_Max)-(FAAC_Min))/(Input_Max-Input_Min);
    Normalized_out = ((FAAC_Max)+(FAAC_Min))/2 + Gamma*(Input - (Input_Max+Input_Min)/2);
    return Normalized_out;
}

void Fuzzy_adaptive_md::FAAC_Kd_variation(double Fd, double Curr_Kd, Eigen::Vector3d &Curr_posi, double* Output_Kd)
{
    Zm = Curr_posi;

    if(Fd>=0.01)
    {
        FAAC_Kd = 0;
        Zh0 = Zm;
    }
    else
    {
        Eigen::Vector3d Zh0MZm = Zh0 - Zm;
        if((Init_Kd_ > Curr_Kd) && (Kd_recov_dist > Zh0MZm.norm()))
        {
            double Kdh = Curr_Kd*exp(-(FAAC_Ts/2)*Zh0MZm.norm()/fabs(Kd_recov_dist-Zh0MZm.norm()));
            FAAC_Kd = Init_Kd_ - Kdh;
        }
        else {FAAC_Kd = Init_Kd_;}
    }

    *Output_Kd = FAAC_Kd;
}

void Fuzzy_adaptive_md::FAAC_MD_MainCal(double Fd, double F_ext, double* Cal_Md, double* Cal_Dd)
{
    /* Input parameter generation */
    FAAC_Fe = Fd - F_ext;
    FAAC_Fe_dot = (FAAC_Fe - FAAC_Fe_pre)/FAAC_Ts;
    FAAC_Fe_pre = FAAC_Fe;

    /* Fuzzy input generation */
    double F_H = FAAC_HPFInput_Cal(F_ext, FAAC_HPF_cf, FAAC_Ts, FAAC_MdDdR_threshold);

    /* Delta M calculation */
    double DelM = FAAC_DelM_Cal(FAAC_Fe, FAAC_Fe_dot);

    /* Delta MDRatio calculation */
    double DelMDR = FAAC_DelMDRatio_Cal(FAAC_Fe, F_H);

    /* Mass & MassDamper Ratio update */
    if (Fd >= 0.01)
    {
        Updated_Md += DelM;
        Updated_MDR += DelMDR;
    }

    /* Mass saturation */
    Updated_Md = (Updated_Md >= FAAC_Md_Limit[1]) ? FAAC_Md_Limit[1] : Updated_Md;
    Updated_Md = (Updated_Md <= FAAC_Md_Limit[0]) ? FAAC_Md_Limit[0] : Updated_Md;

    /* MDR saturation */
    Updated_MDR = (Updated_MDR >= FAAC_MDR_Limit[1]) ? FAAC_MDR_Limit[1] : Updated_MDR;
    Updated_MDR = (Updated_MDR <= FAAC_MDR_Limit[0]) ? FAAC_MDR_Limit[0] : Updated_MDR;

    /* Output */
    *Cal_Md = Updated_Md;
    *Cal_Dd = Updated_MDR * Init_MDR * Updated_Md;
}