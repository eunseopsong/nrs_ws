#include "Yoon_path.h"

double sign(double input)
{
    if(input >= 0) {return 1;}
    else {return -1;}
}

int Yoon_path::Single_blended_path(double Tar_pos[], double Tar_vel[], double Waiting_time[],int Path_length)
{
    // Step1 : Travel time generation
    this->Travel_time = this->Tm*2;
    for(int i=0;i<Path_length;i++)
    {
        if(Waiting_time[i] == 0)
        {
            if(Tar_vel[i] != 0) {this->Travel_time = this->Travel_time + fabs(Tar_pos[i]/Tar_vel[i]);}
            else
            {
                printf("Target velocity must not be zero");
                return -1;
            }
        }
        else // waiting mode
        {
            this->Travel_time = this->Travel_time + Waiting_time[i];
        }
    }

    // Step2 : Time profile generation
    this->Y_array_size = (int)(this->Travel_time/this->Ts);
    this->time_profile = Dynamic1D::LinSpaced(this->Y_array_size,0,this->Travel_time-this->Ts);
    

    // Step3 : Interpolation
    this->Interpolated = DynamicMatrix::Zero(this->Y_array_size,2);

    int path_counter = 0;
    double path_time;
    if(Tar_vel[0] != 0) path_time = fabs(Tar_pos[0]/Tar_vel[0]);
    else path_time = Waiting_time[0];

    for(int i=0;i<this->Y_array_size;i++)
    {
        if(this->time_profile(i)<this->Tm) // initial waiting time
        {
            this->Interpolated(i,0) = this->time_profile(i);
            this->Interpolated(i,1) = 0;
        }
        else if(this->time_profile(i) < this->Travel_time - this->Tm) // Moving section
        {
            this->Interpolated(i,0) = this->time_profile(i);
            this->Interpolated(i,1) = Tar_vel[path_counter];
            if((this->time_profile(i)-this->Tm >= path_time)&&(path_counter<Path_length-1))
            {
                if(Tar_vel[path_counter+1] != 0) path_time = path_time + fabs(Tar_pos[path_counter+1]/Tar_vel[path_counter+1]);
                else path_time = path_time + Waiting_time[path_counter+1];
                path_counter ++;
            }
            
        }
        else
        {
            this->Interpolated(i,0) = this->time_profile(i);
            this->Interpolated(i,1) = 0;
        }
    }
    printf("%d path point loaded \n",path_counter+1);

    // Step4 : Velocity profiling
    unsigned int PA = (int)(this->Acc_time/this->Ts); // Points for acceleration
    this->Final_vel = DynamicMatrix::Zero(this->Travel_time/Ts,2);
    this->Final_pos = DynamicMatrix::Zero(this->Travel_time/Ts,2);
    
    for(int i=1;i<this->Y_array_size;i++)
    {
        if(i<=PA)
        {
            this->Final_vel(i,0) = this->time_profile(i);
            this->Final_vel(i,1) = this->Final_vel(i-1,1) + (this->Interpolated(i,1)-this->Interpolated(0,1))/i;
        }
        else
        {
            this->Final_vel(i,0) = this->time_profile(i);
            this->Final_vel(i,1) = this->Final_vel(i-1,1) + (this->Interpolated(i,1)-this->Interpolated(i-PA,1))/PA;
        }

        // transmit to postion profile
        this->Final_pos(i,0) = this->time_profile(i);
        this->Final_pos(i,1) = this->Final_pos(i-1,1) + this->Final_vel(i,1)*this->Ts;
    }

    printf("Single_blended_path was done\n");
    return (int)(this->Travel_time/this->Ts);
}

int Yoon_path::Single_blended_force(double Exe_time[], double Tar_force[], double Waiting_time[],int Path_length)
{
    // Step1 : Travel time generation
    this->Travel_time = this->Tm*2;
    for(int i=0;i<Path_length;i++)
    {
        if(Waiting_time[i] == 0)
        {this->Travel_time = this->Travel_time + fabs(Exe_time[i]);}
        else // waiting mode
        {this->Travel_time = this->Travel_time + Waiting_time[i];}
    }

    // Step2 : Time profile generation
    this->Y_array_size = (int)(this->Travel_time/this->Ts);
    this->time_profile = Dynamic1D::LinSpaced(this->Y_array_size,0,this->Travel_time-this->Ts);
    

    // Step3 : Interpolation
    this->Interpolated = DynamicMatrix::Zero(this->Y_array_size,2);

    int path_counter = 0;
    double path_time = fabs(Exe_time[0]);

    for(int i=0;i<this->Y_array_size;i++)
    {
        if(this->time_profile(i)<this->Tm) // initial waiting time
        {
            this->Interpolated(i,0) = this->time_profile(i);
            this->Interpolated(i,1) = 0;
        }
        else if(this->time_profile(i) < this->Travel_time - this->Tm) // Moving section
        {
            this->Interpolated(i,0) = this->time_profile(i);
            this->Interpolated(i,1) = Tar_force[path_counter];
            if((this->time_profile(i)-this->Tm >= path_time)&&(path_counter<Path_length-1))
            {
                path_time = path_time + fabs(Exe_time[path_counter+1]);
                path_counter ++;
            }
        }
        else
        {
            this->Interpolated(i,0) = this->time_profile(i);
            this->Interpolated(i,1) = 0;
        }
    }
    printf("%d path point loaded \n",path_counter+1);

    // Step4 : Velocity profiling
    unsigned int PA = (int)(this->Acc_time/this->Ts); // Points for acceleration
    this->Final_force = DynamicMatrix::Zero(this->Travel_time/Ts,2);
    
    for(int i=1;i<this->Y_array_size;i++)
    {
        if(i<=PA)
        {
            this->Final_force(i,0) = this->time_profile(i);
            this->Final_force(i,1) = this->Final_force(i-1,1) + (this->Interpolated(i,1)-this->Interpolated(0,1))/i;
        }
        else
        {
            this->Final_force(i,0) = this->time_profile(i);
            this->Final_force(i,1) = this->Final_force(i-1,1) + (this->Interpolated(i,1)-this->Interpolated(i-PA,1))/PA;
        }
    }

    printf("Single_blended_force was done\n");
    return (int)(this->Travel_time/this->Ts);
}

bool Yoon_path::PTP_6D_path_init(double Init_ABSpos[6], double Tar_ABSpos[6], double travel_time)
{
    double Linear_travel_time = travel_time;

    double Tar_pos[] = {0.0}; // Target position array, unit: m (this value is applied as absolute value)
    double Tar_vel[] = {0.0}; // Target velocity array, unit: m/s (the direction is controled by cmd velocity)
    double Waiting_time[] = {0.0}; // Waiting time, unit : s, 0이 아닌 값을 넣으려면 동일 array 위치의 pos와 vel은 0 

    for(int i = 0;i<6;i++)
    {
        this->starting_pos[i] = Init_ABSpos[i]; // initial pos update
        Tar_pos[0] = Tar_ABSpos[i]-Init_ABSpos[i];
        Tar_vel[0] = Tar_pos[0]/Linear_travel_time;
        this->generated_path_num[i] = Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));

        if(this->generated_path_num[i] != -1)
        {
            if(i == 0) this->Final_px = this->Final_pos;
            else if(i == 1) this->Final_py = this->Final_pos;
            else if(i == 2) this->Final_pz = this->Final_pos;
            else if(i == 3) this->Final_rx = this->Final_pos;
            else if(i == 4) this->Final_ry = this->Final_pos;
            else if(i == 5) this->Final_rz = this->Final_pos;
        }
        else
        {
            printf("error at PTP_6D_path_init \n");
            return false;
        }

    }
    
    this->PTP_6D_path_init_flag = true;
    this->PTP_6D_path_counter = 0;
    printf("PTP_6D_path_init was done \n");
    return true;
    
}
bool Yoon_path::PTP_6D_path_exe(double path_out[6])
{
    
    if(this->PTP_6D_path_init_flag == true)
    {
        if(this->PTP_6D_path_counter < this->generated_path_num[0])
        {
            path_out[0] = this->starting_pos[0] + this->Final_px(this->PTP_6D_path_counter,1);
            path_out[1] = this->starting_pos[1] + this->Final_py(this->PTP_6D_path_counter,1);
            path_out[2] = this->starting_pos[2] + this->Final_pz(this->PTP_6D_path_counter,1);
            path_out[3] = this->starting_pos[3] + this->Final_rx(this->PTP_6D_path_counter,1);
            path_out[4] = this->starting_pos[4] + this->Final_ry(this->PTP_6D_path_counter,1);
            path_out[5] = this->starting_pos[5] + this->Final_rz(this->PTP_6D_path_counter,1);

            this->PTP_6D_path_counter = this->PTP_6D_path_counter + 1;
            return true;
        }
        else
        {
            for(int i=0;i<6;i++) {this->generated_path_num[i] = -1;}
            this->PTP_6D_path_counter = 0;
            this->PTP_6D_path_init_flag = false;
            return false;
        }
        
    }
    else return false;
}

bool Yoon_path::MultiP_6D_path_init(Eigen::MatrixXd Tar_point, double _Tar_vel[], double Waiting_time[],int Point_num)
{
    // if Point_num & # of Tar_point row is 3; 
    // -> then, size of array of Tar_vel,Waiting_time must be 2 (point_num - 1);

    double Init_ABSpos[6] = {Tar_point(0,0),Tar_point(0,1),Tar_point(0,2),Tar_point(0,3),Tar_point(0,4),Tar_point(0,5)};
    double Tar_pos[Point_num-1] = {0,}; // Target position array, unit: m (this value is applied as absolute value)
    double Tar_vel[Point_num-1] = {0,}; 
    double Travel_dist = 0;
    double Travel_time = 0;

    for(int i = 0;i<6;i++)
    {
        for(int j = 1; j<Point_num; j++)
        {
            Tar_pos[j-1] = fabs(Tar_point(j,i)-Tar_point(j-1,i));

            Travel_dist = sqrt(pow(Tar_point(j,0)-Tar_point(j-1,0),2)+pow(Tar_point(j,1)-Tar_point(j-1,1),2)+pow(Tar_point(j,2)-Tar_point(j-1,2),2));
            if(Tar_pos[j-1] != 0){
                Travel_time = Travel_dist/_Tar_vel[j-1]; // cartesian travel time 
                Tar_vel[j-1] = Tar_pos[j-1]/Travel_time; // calculate each axis velocity
            }
            else Tar_vel[j-1] = 0;
            
            if(Tar_point(j,i)-Tar_point(j-1,i) >= 0) Tar_vel[j-1] = fabs(Tar_vel[j-1]);
            else Tar_vel[j-1] = -fabs(Tar_vel[j-1]);
        }

        this->Mstarting_pos[i] = Init_ABSpos[i]; // initial pos update

        this->Mgenerated_path_num[i] = Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));

        if(this->Mgenerated_path_num[i] != -1)
        {
            if(i == 0) this->MFinal_px = this->Final_pos;
            else if(i == 1) this->MFinal_py = this->Final_pos;
            else if(i == 2) this->MFinal_pz = this->Final_pos;
            else if(i == 3) this->MFinal_rx = this->Final_pos;
            else if(i == 4) this->MFinal_ry = this->Final_pos;
            else if(i == 5) this->MFinal_rz = this->Final_pos;
        }
        else
        {
            printf("error at MultiP_6D_path_init \n");
            return false;
        }

    }

    this->MPTP_6D_path_init_flag = true;
    this->MPTP_6D_path_counter = 0;
    printf("MultiP_6D_path_init was done \n");
    return true;
}
bool Yoon_path::MultiP_path_exe(double path_out[6])
{
    if(this->MPTP_6D_path_init_flag == true)
    {
        if(this->MPTP_6D_path_counter < this->Mgenerated_path_num[0])
        {
            path_out[0] = this->Mstarting_pos[0] + this->MFinal_px(this->MPTP_6D_path_counter,1);
            path_out[1] = this->Mstarting_pos[1] + this->MFinal_py(this->MPTP_6D_path_counter,1);
            path_out[2] = this->Mstarting_pos[2] + this->MFinal_pz(this->MPTP_6D_path_counter,1);
            path_out[3] = this->Mstarting_pos[3] + this->MFinal_rx(this->MPTP_6D_path_counter,1);
            path_out[4] = this->Mstarting_pos[4] + this->MFinal_ry(this->MPTP_6D_path_counter,1);
            path_out[5] = this->Mstarting_pos[5] + this->MFinal_rz(this->MPTP_6D_path_counter,1);

            this->MPTP_6D_path_counter = this->MPTP_6D_path_counter + 1;
            return true;
        }
        else
        {
            for(int i=0;i<6;i++) {this->Mgenerated_path_num[i] = -1;}
            this->MPTP_6D_path_counter = 0;
            this->MPTP_6D_path_init_flag = false;
            return false;
        }
        
    }
    else return false;
}

bool Yoon_path::PPB_path_init(Eigen::MatrixXd Tar_point, double _Tar_vel[], double _Des_force[], double Waiting_time[],int Point_num)
{
    // if Point_num & # of Tar_point row is 3; 
    // -> then, size of array of Tar_vel,Waiting_time must be 2 (point_num - 1);

    double Init_ABSpos[7] = {Tar_point(0,0),Tar_point(0,1),Tar_point(0,2),Tar_point(0,3),Tar_point(0,4),Tar_point(0,5),(double)0.0};
    double Tar_pos[Point_num-1] = {0,}; // Target position array, unit: m (this value is applied as absolute value)
    double Tar_vel[Point_num-1] = {0,};
    double Exe_time[Point_num-1] = {0,};
    double Tar_force[Point_num-1] = {0,}; 
    double Travel_dist = 0;
    double Travel_time[Point_num-1] = {0,};

    for(int i = 0;i<7;i++) // 6 - posture, 1- force
    {
        /* Tar_pos(relative), Exe_time generation */
        for(int j = 1; j<Point_num; j++)
        {
            if(i < 6) // Posture
            { 
                Tar_pos[j-1] = fabs(Tar_point(j,i)-Tar_point(j-1,i));

                Travel_dist = sqrt(pow(Tar_point(j,0)-Tar_point(j-1,0),2)+pow(Tar_point(j,1)-Tar_point(j-1,1),2)+pow(Tar_point(j,2)-Tar_point(j-1,2),2));
                if(Tar_pos[j-1] != 0){
                    Travel_time[j-1] = Travel_dist/_Tar_vel[j-1]; // cartesian travel time 
                    Waiting_time[j-1] = 0;
                    Tar_vel[j-1] = Tar_pos[j-1]/Travel_time[j-1]; // calculate each axis velocity
                }
                else 
                {
                    /* if pos was overlapped, waiting time will be applied using "Defualt_WaitingT" */
                    Travel_time[j-1] = Defualt_WaitingT;
                    Waiting_time[j-1] = Defualt_WaitingT;
                    Tar_vel[j-1] = 0;
                    printf("Vel_zero i: %d, j: %d \n",i,j);
                }
                
                if(Tar_point(j,i)-Tar_point(j-1,i) >= 0) Tar_vel[j-1] = fabs(Tar_vel[j-1]);
                else Tar_vel[j-1] = -fabs(Tar_vel[j-1]);
            }
            else // Force
            {
                Exe_time[j-1] = Travel_time[j-1];
                Tar_force[j-1] = _Des_force[j-1];
            }
        }
        
        this->PPBstarting_pos[i] = Init_ABSpos[i]; // initial pos + force update (actually force init must be "0")

        if(i < 6)
        {this->PPBgenerated_path_num[i] = Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));}
        else
        {this->PPBgenerated_path_num[i] = Single_blended_force(Exe_time,Tar_force,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));}

        if(this->PPBgenerated_path_num[i] != -1)
        {
            if(i == 0) this->PPBFinal_px = this->Final_pos;
            else if(i == 1) this->PPBFinal_py = this->Final_pos;
            else if(i == 2) this->PPBFinal_pz = this->Final_pos;
            else if(i == 3) this->PPBFinal_rx = this->Final_pos;
            else if(i == 4) this->PPBFinal_ry = this->Final_pos;
            else if(i == 5) this->PPBFinal_rz = this->Final_pos;
            else if(i == 6) this->PPBFinal_Force = this->Final_force;

            printf("Gen_path_num: %d\n",this->PPBgenerated_path_num[i]);
        }
        else
        {
            printf("error at PPB_path_init \n");
            return false;
        }

    }

    this->PPB_path_init_flag = true;
    this->PPB_path_counter = 0;
    printf("PPB_path_init was done \n");
    return true;
}
bool Yoon_path::PPB_path_exe(double path_out[])
{
    // The path_out contains 6D pose & force data (Totally 7 data)
    
    if(this->PPB_path_init_flag == true)
    {
        if(this->PPB_path_counter < this->PPBgenerated_path_num[0])
        {
            path_out[0] = this->PPBstarting_pos[0] + this->PPBFinal_px(this->PPB_path_counter,1);
            path_out[1] = this->PPBstarting_pos[1] + this->PPBFinal_py(this->PPB_path_counter,1);
            path_out[2] = this->PPBstarting_pos[2] + this->PPBFinal_pz(this->PPB_path_counter,1);
            path_out[3] = this->PPBstarting_pos[3] + this->PPBFinal_rx(this->PPB_path_counter,1);
            path_out[4] = this->PPBstarting_pos[4] + this->PPBFinal_ry(this->PPB_path_counter,1);
            path_out[5] = this->PPBstarting_pos[5] + this->PPBFinal_rz(this->PPB_path_counter,1);
            path_out[6] = this->PPBstarting_pos[6] + this->PPBFinal_Force(this->PPB_path_counter,1);

            this->PPB_path_counter = this->PPB_path_counter + 1;
            return true;
        }
        else
        {
            for(int i=0;i<7;i++) {this->PPBgenerated_path_num[i] = -1;}
            this->PPB_path_counter = 0;
            this->PPB_path_init_flag = false;
            return false;
        }
        
    }
    else return false;
}

bool Yoon_path::Trape_1D_path_init(double ald_, double vd, double x0_, double xf_)
{
    // Be care full!! Do not set the vd = 0

    /* Step0: global parameter update & set */
    this->x0 = x0_;
    this->xf = xf_;
    this->ald = ald_;
    this->tc = 0;
    /* Step1: tf calculation */
    this->t0 = 0;
    this->tf = t0 + fabs(xf-x0)/vd;

    /* Step2: tb,xb calculation & ald recalculation */
    double delT = this->tf - this->t0;

    if(this->ald >- 4*fabs(this->xf-this->x0)/pow(delT,2))
    {
        this->tb = delT/2 - sqrt(pow(this->ald,2)*pow(delT,2) - 4*this->ald*fabs(this->xf-this->x0))/
        (2*this->ald);
        this->xb = sign(this->xf-this->x0)*(1/2)*this->ald*pow(this->tb,2) + this->x0;

        return true;
    }
    else
    {
        this->tb = delT/2;
        this->xb = (this->xf-this->x0)/2;
        this->ald = 4*fabs(this->xf-this->x0)/pow(delT,2);

        return true;
    }
    
}
bool Yoon_path::Trape_1D_path_exe(double* x_out)
{
    if(this->tc <= this->tb)
    {*x_out = sign(this->xf-this->x0)*(1/2)*this->ald*pow(this->tc,2) + this->x0;}

    else if(this->tc <= this->tf - this->tb)
    {*x_out = ((this->xf-2*this->xb)/(this->tf-2*this->tb))*(this->tc-this->tb) + this->xb;}

    else
    {
        if(this->tb == (this->tf - this->t0)/2)
        {*x_out = (this->xf-this->xb)+sign(this->xf-this->x0)*this->ald*this->tb*(this->tc-this->tb)
        -sign(this->xf-this->x0)*(1/2)*this->ald*pow(this->tc-this->tb,2);}
        else
        {*x_out = (this->xf-this->xb)+((this->xf-2*this->xb)/(this->tf-2*this->tb))*(this->tc+this->tb-this->tf)
        -sign(this->xf-this->x0)*(1/2)*this->ald*pow(this->tc+this->tb-this->tf,2);}
    }

    this->tc += this->Ts;

    if(this->tc < this->tf){return true;}
    else {return false;}
}

