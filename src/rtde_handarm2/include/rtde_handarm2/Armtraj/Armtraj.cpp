#include "Armtraj.h"
#include <math.h>
#include <ctime>

FILE *testData, *detData, *detData6;

Armtraj::Armtraj()
{
	printf("============== Applying trajectory planning =================\n");

	testData = fopen("/home/hand/testdata.txt","wt");
	detData = fopen("/home/hand/detdata.txt","wt");
	detData6 = fopen("/home/hand/detdata6.txt","wt");

	Distance.resize(6);
	JointPosStart.resize(6);
	JointPosStop.resize(6);
	TaskPosStart.resize(6);
	TaskPosStop.resize(6);
	JointVelStart.resize(6);
	TaskVelStart.resize(6);
	Dist_cal.resize(6);
	InitDist.resize(6);

	n_origin2.resize(3,4);
	v.resize(4,3);
}

int Armtraj::TaskTraj_lspb_realtime(CArm *A, float CustomTaskMaxVel=0.5, float CustomTaskMaxAcc=0.5)
{
		// Input = Target Task Position xt, Current Task Position xc
		// Output = Desired Task Velocity dxd

	int i;
	Vector3d Dist, Dist_cal, tasktime, Dir, v0, InitDist;
	float aMax = CustomTaskMaxAcc;
	float vMax = CustomTaskMaxVel;
	float tMax=0;

	Dist = A->xt - A->xc;
  for(i=0;i<3;i++){
    if(Dist(i)<0){
			Dist(i)=Dist(i)*-1;
			Dir(i)=-1;
      v0(i)=A->dxd(i)*-1;
		}
    InitDist(i) = v0(i)*v0(i)/aMax/2;

    if(v0(i)>0&&Dist(i)<InitDist(i)){
        Dist_cal(i) = InitDist(i) - Dist(i);
        if(Dist_cal(i) < vMax*vMax/aMax)
            tasktime(i) = 2*sqrt(Dist_cal(i)/aMax) + v0(i)/aMax;
        else
            tasktime(i) = Dist_cal(i)/vMax + (vMax + v0(i))/aMax;
		}
    else if(v0(i)>vMax)
        tasktime(i) = v0(i)/aMax + Dist(i)/vMax - v0(i)*v0(i)/aMax/vMax/2;
    else{
        Dist_cal(i) = Dist(i) + InitDist(i);
        if(Dist_cal(i) < vMax*vMax/aMax)
            tasktime(i) = 2*sqrt(Dist_cal(i)/aMax) - v0(i)/aMax;
        else
            tasktime(i) = Dist_cal(i)/vMax + (vMax - v0(i))/aMax;
		}
    if(tasktime(i)>tMax) tMax=tasktime(i);
	}


  for(i=0;i<3;i++){
    vtop=(Dist(i)+InitDist(i))/tMax;
    if(v0(i)<0)	A->dxd(i) = Dir(i)*(v0(i) + aMax/AHz);
    else if(v0(i)>0&&Dist(i)<InitDist(i)+0.001)   A->dxd(i) = Dir(i)*(v0(i) - aMax/AHz);
    else if(v0(i)<vtop){
			A->dxd(i) = (v0(i) + aMax/AHz);
			if(A->dxd(i)>=vtop)  A->dxd(i) = vtop;
			A->dxd(i) = Dir(i)*A->dxd(i);
		}
    else if(v0(i)>vtop){
			A->dxd(i) = (v0(i) - aMax/AHz);
			if(A->dxd(i)<=vtop)  A->dxd(i) = vtop;
			A->dxd(i) = Dir(i)*A->dxd(i);
		}
    else
			A->dxd(i) = Dir(i)*A->dxd(i);
	}

	return	0;
}

int Armtraj::Traj3(CArm *A)
{
	// Input = Target Joint Angle qt, Current Joint Angle qc
	// Output = Desired Joint Angle qd

	int i;
    
	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						//Maxerror=error[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
		}

		MaxFinalTime = 3*Distance[num]/(2*JointMaxVel);
		a[0] = 0.0;
		a[1] = 0.0;
		a[2] = 3.0/(MaxFinalTime*MaxFinalTime);
		a[3] = (-2.0)/(MaxFinalTime*MaxFinalTime*MaxFinalTime);

		SEQ=1;
	}

	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;
	s = a[0] + a[1]*TaskTime +a[2]*TaskTime*TaskTime + a[3]*TaskTime*TaskTime*TaskTime;
  for(i=0;i<6;i++)
  	A->qd(i) = JointPosStart[i] + Dir[i]*Distance[i]*s;
	
  if(MaxFinalTime<=TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
  }
	return	0;
}

int Armtraj::Traj5(CArm *A)
{
		// Input = Target Joint Angle qt, Current Joint Angle qc
		// Output = Desired Joint Angle qd

	int i;
    
	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
	}

		MaxFinalTime = 15*Distance[num]/(8*JointMaxVel);
		a[0] = 0.0;
		a[1] = 0.0;
		a[2] = 0.0;
		a[3] = 10.0/(MaxFinalTime*MaxFinalTime*MaxFinalTime);
		a[4] = -15.0/(MaxFinalTime*MaxFinalTime*MaxFinalTime*MaxFinalTime);
		a[5] = 6/(MaxFinalTime*MaxFinalTime*MaxFinalTime*MaxFinalTime*MaxFinalTime);

		SEQ=1;
	}

	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;
	s = a[0] + a[1]*TaskTime +a[2]*TaskTime*TaskTime + a[3]*TaskTime*TaskTime*TaskTime + a[4]*TaskTime*TaskTime*TaskTime*TaskTime + a[5]*TaskTime*TaskTime*TaskTime*TaskTime*TaskTime;
  for(i=0;i<6;i++)
  	A->qd(i) = JointPosStart[i] + Dir[i]*Distance[i]*s;
	
  if(MaxFinalTime<=TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
  }
	return	0;
}

int Armtraj::Traj_rspb(CArm *A)
{
		// Input = Target Joint Angle qt, Current Joint Angle qc
		// Output = Desired Joint Angle qd

	int i;
    
	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
		}
		if(MaxDistance>JointMaxVel*JointMaxVel/JointMaxAcc)
		    MaxFinalTime = (MaxDistance/JointMaxVel)+(JointMaxVel/JointMaxAcc);
		else
		    MaxFinalTime = 2*sqrt(MaxDistance/JointMaxAcc);

		for(i=0;i<6;i++){
			t1[i]=(MaxFinalTime - sqrt(MaxFinalTime*MaxFinalTime-4*Distance[i]/JointMaxAcc))/2;
			t2[i]=(MaxFinalTime + sqrt(MaxFinalTime*MaxFinalTime-4*Distance[i]/JointMaxAcc))/2;
		}

		SEQ=1;
	}

	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;
	
	for(i=0;i<6;i++){
  	if(TaskTime<=t1[i]){
    	A->qd[i] = JointPosStart[i] + Dir[i]*JointMaxAcc*TaskTime*TaskTime/2;
    }
    else if(TaskTime>t1[i]&&TaskTime<t2[i]){
			A->qd[i] = JointPosStart[i] + Dir[i]*JointMaxAcc*t1[i]*(TaskTime - t1[i]/2);
    }
    else if(TaskTime>=t2[i]){
			A->qd[i] = JointPosStop[i] - Dir[i]*(JointMaxAcc*(MaxFinalTime - TaskTime)*(MaxFinalTime - TaskTime))/2;
    }
	}

  if(MaxFinalTime<TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
  }
	return	0;
}
int Armtraj::Traj_lspb(CArm *A)
{
		// Input = Target Joint Angle qt, Current Joint Angle qc
		// Output = Desired Joint Angle qd

	int i;
  double t1,t2;
    
	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
		}
		if(MaxDistance>JointMaxVel*JointMaxVel/JointMaxAcc)
		    MaxFinalTime = (MaxDistance/JointMaxVel)+(JointMaxVel/JointMaxAcc);
		else
		    MaxFinalTime = 2*sqrt(MaxDistance/JointMaxAcc);

    		t1=(MaxFinalTime - sqrt(MaxFinalTime*MaxFinalTime-4*MaxDistance/JointMaxAcc))/2;
		t2=(MaxFinalTime + sqrt(MaxFinalTime*MaxFinalTime-4*MaxDistance/JointMaxAcc))/2;

		SEQ=1;
	}

	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;
	
	for(i=0;i<6;i++){
    a[i] = Distance[i]/(t1*t2);
  	if(TaskTime<=t1){
    	A->qd[i] = JointPosStart[i] + Dir[i]*a[i]*TaskTime*TaskTime/2;
    }
    else if(TaskTime>t1&&TaskTime<t2){
			A->qd[i] = JointPosStart[i] + Dir[i]*a[i]*t1*(TaskTime - t1/2);
    }
    else if(TaskTime>=t2){
			A->qd[i] = JointPosStop[i] - Dir[i]*(a[i]*(MaxFinalTime - TaskTime)*(MaxFinalTime - TaskTime))/2;
    }
	}

  if(MaxFinalTime<TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
  }
	return	0;
}

int Armtraj::Traj_scurve(CArm *A)
{
		// Input = Target Joint Angle qt, Current Joint Angle qc
		// Output = Desired Joint Angle qd

	int i;
    
	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;
		jerk = JointMaxAcc*JointMaxAcc/(beta*JointMaxVel);

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
		}

		tj1 = JointMaxAcc/jerk;
		ta1 = JointMaxVel/JointMaxAcc - tj1;
		tv = MaxDistance/JointMaxVel - 2*tj1 - ta1;
		
		if(MaxDistance > JointMaxVel*(2*tj1 + ta1)){
   	 MaxFinalTime = 4*tj1 + 2*ta1 +tv;
		}
		else if(MaxDistance <= JointMaxVel*(2*tj1 + ta1) && MaxDistance > JointMaxAcc*4*tj1*tj1){
    	ta1 = (-3*tj1 + sqrt(9*tj1*tj1 - 4*(2*tj1*tj1 - MaxDistance/(jerk*tj1))))/2;
    	beta = JointMaxAcc*JointMaxAcc/(jerk*JointMaxVel);
    	MaxFinalTime = 4*tj1 + 2*ta1;
    	tv = 0;
		}
		else{
  	  tj1 = powf(MaxDistance/(2*jerk),1.0/3.0);
  	  ta1 = 0;
  	  tv = 0;
  	  MaxFinalTime = 4*tj1;
  	  beta = JointMaxAcc*JointMaxAcc/(jerk*JointMaxVel);
		}
  	tup = 2*tj1 + ta1;
		tm = tup + tv;
    
    for(int i=0;i<6;++i){
   	  vel[i] = Distance[i]/tm;
      tj[i] = (tup - sqrt(tup*tup - 4*vel[i]/jerk))/2;
      ta[i] = tup - 2*tj[i];
    	acc[i] = jerk*tj[i];
  	}

		SEQ=1;
	}

	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;
	
	for(i=0;i<6;i++){
		if (TaskTime < tj[i])
    {
    	A->qd(i) = JointPosStart[i] + Dir[i]*jerk*TaskTime*TaskTime*TaskTime/6;
		}
    else if (tj[i] <= TaskTime && TaskTime < tj[i] + ta[i])
		{
    	A->qd(i) = JointPosStart[i] + Dir[i]*jerk*tj[i]*tj[i]*tj[i]/6 + Dir[i]*(TaskTime - tj[i])*acc[i]*(TaskTime - tj[i]/2) - Dir[i]*acc[i]*(TaskTime - tj[i])*(TaskTime - tj[i])/2;
    }
    else if (tj[i] + ta[i] <= TaskTime && TaskTime < 2*tj[i] + ta[i])
    {   
    	A->qd(i) = JointPosStart[i] + Dir[i]*vel[i]*(2*tj[i] + ta[i])/2 + Dir[i]*
jerk*(2*tj[i] + ta[i] - TaskTime)*(2*tj[i] + ta[i] - TaskTime)*(2*tj[i] + ta[i] - TaskTime)/6 - Dir[i]*vel[i]*(2*tj[i] + ta[i] - TaskTime);
    }
    else if (2*tj[i] + ta[i] <= TaskTime && TaskTime <= 2*tj[i] + ta[i] + tv)
    {    
    	A->qd(i) = JointPosStart[i] + Dir[i]*vel[i]*(2*tj[i] + ta[i])/2 + Dir[i]*vel[i]*(TaskTime - 2*tj[i] - ta[i]);
    }
    else if (2*tj[i] + ta[i] + tv <= TaskTime && TaskTime < MaxFinalTime - tj[i] - ta[i])
    {    
    	A->qd(i) = JointPosStart[i] + Dir[i]*vel[i]*(2*tj[i] + ta[i])/2 + Dir[i]*vel[i]*tv + Dir[i]*vel[i]*(TaskTime - 2*tj[i] - ta[i] - tv) - Dir[i]*jerk*(TaskTime - 2*tj[i] - ta[i] - tv)*(TaskTime - 2*tj[i] - ta[i] - tv)*(TaskTime - 2*tj[i] - ta[i] - tv)/6;
    }
    else if (MaxFinalTime - tj[i] - ta[i] <= TaskTime && TaskTime < MaxFinalTime - tj[i])
    {    
    	A->qd(i) = JointPosStop[i] - Dir[i]*jerk*tj[i]*tj[i]*tj[i]/6 - Dir[i]*(MaxFinalTime - tj[i] - TaskTime)*acc[i]*(MaxFinalTime - tj[i]/2 - TaskTime) + Dir[i]*acc[i]*(MaxFinalTime - tj[i] - TaskTime)*(MaxFinalTime - tj[i] - TaskTime)/2;
    }
    else if (MaxFinalTime - tj[i] <= TaskTime && TaskTime <= MaxFinalTime)
    {    
    	A->qd(i) = JointPosStop[i] - Dir[i]*jerk*(MaxFinalTime -TaskTime)*(MaxFinalTime -TaskTime)*(MaxFinalTime -TaskTime)/6;
    }
	}

  if(MaxFinalTime<=TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
  }
	return	0;
}

int Armtraj::Traj_rspb_c(CArm *A, float CustomJointMaxVel, float CustomJointMaxAcc)
{
	// Input = Target Joint Angle qt, Current Joint Angle qc, Maximum Joint Velocity CustomJointMaxVel, Maximum Joint Acceleration CustomJointMaxAcc
	// Output = Desired Joint Angle qd

	int							i;
	JointMaxVel = CustomJointMaxVel;
	JointMaxAcc = CustomJointMaxAcc;

	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;
		jerk = JointMaxAcc*JointMaxAcc/(beta*JointMaxVel);

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
		}

		if(MaxDistance>JointMaxVel*JointMaxVel/JointMaxAcc)
		    MaxFinalTime = (MaxDistance/JointMaxVel)+(JointMaxVel/JointMaxAcc);
		else
		    MaxFinalTime = 2*sqrt(MaxDistance/JointMaxAcc);

		for(i=0;i<6;i++){
			t1[i]=(MaxFinalTime - sqrt(MaxFinalTime*MaxFinalTime-4*Distance[i]/JointMaxAcc))/2;
			t2[i]=(MaxFinalTime + sqrt(MaxFinalTime*MaxFinalTime-4*Distance[i]/JointMaxAcc))/2;
		}

		SEQ=1;
	}
	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;

    for(i=0;i<6;i++){
  	if(TaskTime<=t1[i]){
    	A->qd[i] = JointPosStart[i] + Dir[i]*JointMaxAcc*TaskTime*TaskTime/2;
    }
    else if(TaskTime>t1[i]&&TaskTime<t2[i]){
			A->qd[i] = JointPosStart[i] + Dir[i]*JointMaxAcc*t1[i]*(TaskTime - t1[i]/2);
    }
    else if(TaskTime>=t2[i]){
			A->qd[i] = JointPosStop[i] - Dir[i]*(JointMaxAcc*(MaxFinalTime - TaskTime)*(MaxFinalTime - TaskTime))/2;
        }
    }
    if(MaxFinalTime<=TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
    }

	return	0;
}

int Armtraj::Traj_scurve_c(CArm *A,float CustomJointMaxVel, float CustomJointMaxAcc, float Custombeta)
{
		// Input = Target Joint Angle qt, Current Joint Angle qc
		// Output = Desired Joint Angle qd

	int i;
	JointMaxVel = CustomJointMaxVel;
	JointMaxAcc = CustomJointMaxAcc;
	beta = Custombeta;
    
	if(SEQ==0)
	{
    for(i=0;i<6;i++)
        A->qd(i)=A->qc(i);

		TaskIteration = 0;
		TaskTime = (float)(TaskIteration)/(float)AHz;
		jerk = JointMaxAcc*JointMaxAcc/(beta*JointMaxVel);

		for(i=0;i<6;i++){
				Dir[i] = 1;
		    Distance[i] = A->qt(i) - A->qc(i);
				JointPosStart[i]=A->qc(i);
				JointPosStop[i]=A->qt(i);

		    if(Distance[i]<0){
		        Distance[i]=Distance[i]*-1;
		        Dir[i]=-1;
				}
		    if(Distance[i]>MaxDistance){
		        MaxDistance=Distance[i];
						num = i;
				}
		}
		if(MaxDistance == 0){
			return	1;
		}

		tj1 = JointMaxAcc/jerk;
		ta1 = JointMaxVel/JointMaxAcc - tj1;
		tv = MaxDistance/JointMaxVel - 2*tj1 - ta1;
		
		if(MaxDistance > JointMaxVel*(2*tj1 + ta1)){
   	 MaxFinalTime = 4*tj1 + 2*ta1 +tv;
		}
		else if(MaxDistance <= JointMaxVel*(2*tj1 + ta1) && MaxDistance > JointMaxAcc*4*tj1*tj1){
    	ta1 = (-3*tj1 + sqrt(9*tj1*tj1 - 4*(2*tj1*tj1 - MaxDistance/(jerk*tj1))))/2;
    	beta = JointMaxAcc*JointMaxAcc/(jerk*JointMaxVel);
    	MaxFinalTime = 4*tj1 + 2*ta1;
    	tv = 0;
		}
		else{
  	  tj1 = powf(MaxDistance/(2*jerk),1.0/3.0);
  	  ta1 = 0;
  	  tv = 0;
  	  MaxFinalTime = 4*tj1;
  	  beta = JointMaxAcc*JointMaxAcc/(jerk*JointMaxVel);
		}
  	tup = 2*tj1 + ta1;
		tm = tup + tv;
    
    for(int i=0;i<6;++i){
   	  vel[i] = Distance[i]/tm;
      tj[i] = (tup - sqrt(tup*tup - 4*vel[i]/jerk))/2;
      ta[i] = tup - 2*tj[i];
    	acc[i] = jerk*tj[i];
  	}

		SEQ=1;
	}

	TaskIteration++;
	TaskTime = (float)(TaskIteration)/(float)AHz;
	
	for(i=0;i<6;i++){
		if (TaskTime < tj[i])
    {
    	A->qd(i) = JointPosStart[i] + Dir[i]*jerk*TaskTime*TaskTime*TaskTime/6;
		}
    else if (tj[i] <= TaskTime && TaskTime < tj[i] + ta[i])
		{
    	A->qd(i) = JointPosStart[i] + Dir[i]*jerk*tj[i]*tj[i]*tj[i]/6 + Dir[i]*(TaskTime - tj[i])*acc[i]*(TaskTime - tj[i]/2) - Dir[i]*acc[i]*(TaskTime - tj[i])*(TaskTime - tj[i])/2;
    }
    else if (tj[i] + ta[i] <= TaskTime && TaskTime < 2*tj[i] + ta[i])
    {   
    	A->qd(i) = JointPosStart[i] + Dir[i]*vel[i]*(2*tj[i] + ta[i])/2 + Dir[i]*
jerk*(2*tj[i] + ta[i] - TaskTime)*(2*tj[i] + ta[i] - TaskTime)*(2*tj[i] + ta[i] - TaskTime)/6 - Dir[i]*vel[i]*(2*tj[i] + ta[i] - TaskTime);
    }
    else if (2*tj[i] + ta[i] <= TaskTime && TaskTime <= 2*tj[i] + ta[i] + tv)
    {    
    	A->qd(i) = JointPosStart[i] + Dir[i]*vel[i]*(2*tj[i] + ta[i])/2 + Dir[i]*vel[i]*(TaskTime - 2*tj[i] - ta[i]);
    }
    else if (2*tj[i] + ta[i] + tv <= TaskTime && TaskTime < MaxFinalTime - tj[i] - ta[i])
    {    
    	A->qd(i) = JointPosStart[i] + Dir[i]*vel[i]*(2*tj[i] + ta[i])/2 + Dir[i]*vel[i]*tv + Dir[i]*vel[i]*(TaskTime - 2*tj[i] - ta[i] - tv) - Dir[i]*jerk*(TaskTime - 2*tj[i] - ta[i] - tv)*(TaskTime - 2*tj[i] - ta[i] - tv)*(TaskTime - 2*tj[i] - ta[i] - tv)/6;
    }
    else if (MaxFinalTime - tj[i] - ta[i] <= TaskTime && TaskTime < MaxFinalTime - tj[i])
    {    
    	A->qd(i) = JointPosStop[i] - Dir[i]*jerk*tj[i]*tj[i]*tj[i]/6 - Dir[i]*(MaxFinalTime - tj[i] - TaskTime)*acc[i]*(MaxFinalTime - tj[i]/2 - TaskTime) + Dir[i]*acc[i]*(MaxFinalTime - tj[i] - TaskTime)*(MaxFinalTime - tj[i] - TaskTime)/2;
    }
    else if (MaxFinalTime - tj[i] <= TaskTime && TaskTime <= MaxFinalTime)
    {    
    	A->qd(i) = JointPosStop[i] - Dir[i]*jerk*(MaxFinalTime -TaskTime)*(MaxFinalTime -TaskTime)*(MaxFinalTime -TaskTime)/6;
    }
	}

  if(MaxFinalTime<=TaskTime){
		TaskIteration = 0;
		SEQ=0;

		return	1;
  }
	return	0;
}


int Armtraj::Traj_rspb_velocity(CArm *A)
{
        // Input = Target Joint Angle qt, Current Joint Angle qc, Current Joint Velocity dqc
        // Output = Desired Joint Angle qd

    int    i;
    int j;
    int k;

    JointMaxVel = 0.1;
    JointMaxAcc = 0.1;

    if(A->Traj_seq[0]==0)
    {        
   		A->qd=A->qc;
        A->dqd=A->dqc;
        
        MaxFinalTime = 0;
        MaxDistance = 0;
        TaskIteration = 0;
        TaskTime = (float)(TaskIteration)/(float)AHz;
        Distance = A->qt - A->qc;
        JointPosStart=A->qc;
        JointPosStop=A->qt;
        JointVelStart=A->dqc;

        for(i=0;i<6;i++){
            Dir[i] = 1;
            if (Distance[i] < 0) {
                Distance[i] = Distance[i] * -1;
                Dir[i] = -1;
                JointVelStart[i] = JointVelStart[i] * -1;
            }
            else {
                Dir[i] = 1;
            }
            InitDist[i] = JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc);
          	if(JointVelStart[i]>0 && Distance[i]<InitDist[i]){
          		Dist_cal[i] = -Distance[i] + InitDist[i];
          		if(Dist_cal[i]<JointMaxVel*JointMaxVel/JointMaxAcc)
              	ti[i] = 2*sqrt(Dist_cal[i]/JointMaxAcc)+JointVelStart[i]/JointMaxAcc;
          		else
              	ti[i] = Dist_cal[i]/JointMaxVel + (JointMaxVel + JointVelStart[i])/JointMaxAcc;
        		}
          	else if(JointVelStart[i]>JointMaxVel)
          		ti[i] = (Distance[i]-InitDist[i])/JointMaxVel+JointVelStart[i]/JointMaxAcc;
          	else{
            	Dist_cal[i] = Distance[i] + InitDist[i];
            	if(Dist_cal[i]<JointMaxVel*JointMaxVel/JointMaxAcc)
                ti[i] = 2*sqrt(Dist_cal[i]/(JointMaxAcc))-JointVelStart[i]/JointMaxAcc;
            	else
                ti[i] = Dist_cal[i]/JointMaxVel + (JointMaxVel - JointVelStart[i])/JointMaxAcc;
            }
            if(ti[i]>MaxFinalTime){
                MaxFinalTime=ti[i];
                num = i;
            }
        }
        if(MaxFinalTime == 0){
            return    1;
        }

        for(i=0;i<6;i++){
            if(JointVelStart[i]<0 || Distance[i]>JointVelStart[i]*MaxFinalTime-JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)){
          if(((JointVelStart[i]-JointMaxAcc*MaxFinalTime)*(JointVelStart[i]-JointMaxAcc*MaxFinalTime)-4*JointMaxAcc*(Distance[i]+JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)-JointVelStart[i]*MaxFinalTime))>0){
            t1[i]=(-JointVelStart[i]+JointMaxAcc*MaxFinalTime-sqrt((JointVelStart[i]-JointMaxAcc*MaxFinalTime)*(JointVelStart[i]-JointMaxAcc*MaxFinalTime)-4*JointMaxAcc*(Distance[i]+JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)-JointVelStart[i]*MaxFinalTime)))/(2*JointMaxAcc);
         }
         else{
                 t1[i]=(-JointVelStart[i]+JointMaxAcc*MaxFinalTime)/(2*JointMaxAcc);
         }
         t2[i]=MaxFinalTime-t1[i]-JointVelStart[i]/JointMaxAcc;
       }
       else if(Distance[i]>JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)){
            t1[i]=(Distance[i]+JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)-JointVelStart[i]*MaxFinalTime)/(JointVelStart[i]-JointMaxAcc*MaxFinalTime);
            t2[i]=MaxFinalTime+t1[i]-JointVelStart[i]/JointMaxAcc;
            t1[i]=-t1[i];
        }
        else{
            if((JointVelStart[i]+JointMaxAcc*MaxFinalTime)*(JointVelStart[i]+JointMaxAcc*MaxFinalTime)-4*JointMaxAcc*(-Distance[i]+JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)+JointVelStart[i]*MaxFinalTime)>0){
                t1[i]=(JointVelStart[i]+JointMaxAcc*MaxFinalTime-sqrt((JointVelStart[i]+JointMaxAcc*MaxFinalTime)*(JointVelStart[i]+JointMaxAcc*MaxFinalTime)-4*JointMaxAcc*(-Distance[i]+JointVelStart[i]*JointVelStart[i]/(2*JointMaxAcc)+JointVelStart[i]*MaxFinalTime)))/(2*JointMaxAcc);
            }
            else{
                t1[i]=(JointVelStart[i]+JointMaxAcc*MaxFinalTime)/(2*JointMaxAcc);
            }
            t2[i]=MaxFinalTime-t1[i]+JointVelStart[i]/JointMaxAcc;
            t1[i]=-t1[i];
            t2[i]=-t2[i];
        }
        }
        A->Traj_seq[0]=A->Traj_seq[0]+1;
    }
    TaskIteration++;
    TaskTime = (float)(TaskIteration)/(float)AHz;

    for(i=0;i<6;i++){
        if(TaskTime<=fabs(t1[i])){
            if (t1[i] >= 0) {
                A->qd[i] = JointPosStart[i]+Dir[i]*(2*JointVelStart[i]+JointMaxAcc*TaskTime)*TaskTime/2;
                A->dqd[i] = Dir[i]*JointVelStart[i]+Dir[i]*JointMaxAcc*TaskTime;
            }
            else {
                A->qd[i] = JointPosStart[i]+Dir[i]*(2*JointVelStart[i]-JointMaxAcc*TaskTime)*TaskTime/2;
                A->dqd[i] = Dir[i]*JointVelStart[i]-Dir[i]*JointMaxAcc*TaskTime;
            }
        }
        else if(TaskTime>fabs(t1[i])&&TaskTime<fabs(t2[i])){
                A->qd[i] = JointPosStart[i]+Dir[i]*(2*JointVelStart[i]+JointMaxAcc*t1[i])*fabs(t1[i])/2+Dir[i]*(TaskTime-fabs(t1[i]))*(JointVelStart[i]+JointMaxAcc*t1[i]);
                A->dqd[i] = Dir[i]*JointVelStart[i]+Dir[i]*JointMaxAcc*t1[i];
        }
        else if (TaskTime >= fabs(t2[i])) {
            if (t2[i] < 0) {
                A->qd[i] = JointPosStop[i] + Dir[i] * (JointMaxAcc*(MaxFinalTime - TaskTime)*(MaxFinalTime - TaskTime))/2;
                A->dqd[i] = -Dir[i] * JointMaxAcc * (MaxFinalTime - TaskTime);
            }
            else {
                A->qd[i] = JointPosStop[i] - Dir[i] * (JointMaxAcc*(MaxFinalTime - TaskTime)*(MaxFinalTime - TaskTime)) / 2;
                A->dqd[i] = Dir[i] * JointVelStart[i] + Dir[i] * (JointMaxAcc*t1[i] - JointMaxAcc * (TaskTime - t2[i]));
            }
        }
    }
    if(MaxFinalTime<=TaskTime){
        TaskIteration = 0;
        A->Traj_seq[0]=0;

        return    1;
    }
}