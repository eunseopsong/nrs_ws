#include "Y2RobMotion/robot_motion.hpp"

/* Color macro */
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[37m"
#define WHITE "\033[38m"
#define RESET "\033[0m"


/* State Monitoring */
void robot_motion::state_monitoring()
{
    if(monitoring_flag){
        printf(RED "[%s - %dms] - Control mode: %s(%s)" RESET "\n",
            robot_name.c_str(),static_cast<int>(Control_period_*1000),control_mode.c_str(),force_con_mode.c_str());
        printf("(Ctl_modes: Idling, Position, Guiding, Force)\n");
        printf("------------------------------------------------------------------------------\n");

        /* Current angles */
        printf(GREEN "<C-Ang> j1:%.2f, j2:%.2f, j3:%.2f, j4:%.2f, j5:%.2f, j6:%.2f, j7:%.2f" RESET "\n",
        current_angles[0],current_angles[1],current_angles[2],current_angles[3],current_angles[4],current_angles[5],current_angles[6]);

        /* Target angles */
        printf(BLUE "<T-Ang> j1:%.2f, j2:%.2f, j3:%.2f, j4:%.2f, j5:%.2f, j6:%.2f, j7:%.2f" RESET "\n",
        target_angles[0],target_angles[1],target_angles[2],target_angles[3],target_angles[4],target_angles[5],target_angles[6]);

        printf("------------------------------------------------------------------------------\n");
        
        #if 0
        /* Current angular velocity */
        printf(GREEN "<C-AVel> j1:%+.2f, j2:%+.2f, j3:%+.2f, j4:%+.2f, j5:%+.2f, j6:%+.2f, j7:%+.2f" RESET "\n",
        current_angvel[0],current_angvel[1],current_angvel[2],current_angvel[3],current_angvel[4],current_angvel[5],current_angvel[6]);

        /* Target angular velocity */
        printf(BLUE "<T-AVel> j1:%+.2f, j2:%+.2f, j3:%+.2f, j4:%+.2f, j5:%+.2f, j6:%+.2f, j7:%+.2f" RESET "\n",
        target_angvel[0],target_angvel[1],target_angvel[2],target_angvel[3],target_angvel[4],target_angvel[5],target_angvel[6]);

        printf("------------------------------------------------------------------------------\n");
        #endif

        /* Current Pose */
        printf(GREEN "<C-Posture> x:%.2f, y:%.2f, z%.2f, wx:%.2f, wy:%.2f, wz:%.2f" RESET "\n",
        current_pose[0],current_pose[1],current_pose[2],current_pose[3],current_pose[4],current_pose[5]);

        /* Target Pose */
        printf(BLUE "<T-Posture> x:%.2f, y:%.2f, z%.2f, wx:%.2f, wy:%.2f, wz:%.2f" RESET "\n",
        target_pose[0],target_pose[1],target_pose[2],target_pose[3],target_pose[4],target_pose[5]);

        printf("------------------------------------------------------------------------------\n");
        
        #if 0
        /* Current cartesian velocity */
        printf(GREEN "<C-LinVel> x:%+.2f, y:%+.2f, z%+.2f, wx:%+.2f, wy:%+.2f, wz:%+.2f" RESET "\n",
        current_carvel[0],current_carvel[1],current_carvel[2],current_carvel[3],current_carvel[4],current_carvel[5]);

        /* Target cartesian velocity */
        printf(BLUE "<T-LinVel> x:%+.2f, y:%+.2f, z%+.2f, wx:%+.2f, wy:%+.2f, wz:%+.2f" RESET "\n",
        target_carvel[0],target_carvel[1],target_carvel[2],target_carvel[3],target_carvel[4],target_carvel[5]);

        printf("------------------------------------------------------------------------------\n");
        #endif

        /* Measured force/torque */
        printf(GREEN "<C-F/T> Fx:%+.2f, Fy:%+.2f, Fz%+.2f, Mx:%+.2f, My:%+.2f, Mz:%+.2f" RESET "\n",
            ft1data[0],ft1data[1],ft1data[2],ft1data[3],ft1data[4],ft1data[5]);

        /* Desired force/torque */
        printf(BLUE "<T-F/T> Fx:%+.2f, Fy:%+.2f, Fz%+.2f, Mx:%+.2f, My:%+.2f, Mz:%+.2f" RESET "\n",
            FC_AC_desX[6],FC_AC_desX[7],FC_AC_desX[8],0.0,0.0,0.0);
        
        printf("------------------------------------------------------------------------------\n");
        
        /* Admittance control Mass */
        printf(BLUE "<AC-Mass> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        AControl[0].adm_MDK_monitor(0),AControl[1].adm_MDK_monitor(0),AControl[2].adm_MDK_monitor(0),
        AControl[3].adm_MDK_monitor(0),AControl[4].adm_MDK_monitor(0),AControl[5].adm_MDK_monitor(0));

        printf(MAGENTA "<AC-Damper> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        AControl[0].adm_MDK_monitor(1),AControl[1].adm_MDK_monitor(1),AControl[2].adm_MDK_monitor(1),
        AControl[3].adm_MDK_monitor(1),AControl[4].adm_MDK_monitor(1),AControl[5].adm_MDK_monitor(1));

        printf(GREEN "<AC-Spring> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        AControl[0].adm_MDK_monitor(2),AControl[1].adm_MDK_monitor(2),AControl[2].adm_MDK_monitor(2),
        AControl[3].adm_MDK_monitor(2),AControl[4].adm_MDK_monitor(2),AControl[5].adm_MDK_monitor(2));

        printf("------------------------------------------------------------------------------\n");
        /* Admittance control output pose */ 
        printf(BLUE "<AC OUT>  x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        AC_pose[0],AC_pose[1],AC_pose[2],AC_pose[3],AC_pose[4],AC_pose[5]);


        printf("==============================================================================\n");
    }
}
