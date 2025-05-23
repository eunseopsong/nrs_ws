/****************************************************
  * 
  * ArmGide Main file (ArmTrajectory.cpp) for 6-DOF ARM
  *
  * Created on 2016. 9. 7.
  * Created by Gitae
  *
****************************************************/
#include 	"ArmGuide.h"

ArmGuide::ArmGuide()
{
	printf("============== Applying Arm guiding =================\n");
}
ArmGuide::~ArmGuide()
{
	printf("============== Delete Arm guiding =================\n");
}
int ArmGuide::ArmForceGuide4(AKfun *K, CArm *A)
{ //force guide with velocity command
	
//Force and Torque Limitless
	/*
	for(int i=0;i<3;i++){				//force
		if(A->FMc(i)>20)
			A->fd(i)=20;
		else if(A->FMc(i)<-20)
			A->fd(i)=-20;
		else 
			A->fd(i)=A->FMc(i);
	}

	for(int i=0;i<3;i++){				//moment
		if(A->FMc(i+3)>2)
			A->td(i)=2;
		else if(A->FMc(i+3)<-2)
			A->td(i)=-2;
		else
			A->td(i)=A->FMc(i+3);
	}*/

//Force and Torque Limitness

	for(int i=0;i<3;i++){				//force
		if(A->FMc(i)>20)
			A->fd(i)=20-0.7;
		else if(A->FMc(i)<-20)
			A->fd(i)=-20+0.7;
		else if(A->FMc(i)>0.7)
			A->fd(i)=A->FMc(i)-0.7;
		else if(A->FMc(i)<-0.7)
			A->fd(i)=A->FMc(i)+0.7;
		else if(A->FMc(i)>-0.7&&A->FMc(i)<0.7)
			A->fd(i)=0;
	}
	for(int i=0;i<3;i++){				//moment
		if(A->FMc(i+3)>2)
			A->td(i)=2-0.03;
		else if(A->FMc(i+3)<-2)
			A->td(i)=-2+0.03;
		else if(A->FMc(i+3)>0.03)
			A->td(i)=A->FMc(i+3)-0.03;
		else if(A->FMc(i+3)<-0.03)
			A->td(i)=A->FMc(i+3)+0.03;
		else if(A->FMc(i+3)>-0.03&&A->FMc(i+3)<0.03)
			A->td(i)=0;
	}


	Matrix3d Re;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++)
			Re(i,j) = A->Tc(i,j);
	}

	for(int i=0;i<3;i++){	
		A->Fbase(i)=A->FMc(i)*3;
		A->Tbase(i)=A->FMc(i+3)*3;
	}
	A->Fbase=Re*A->Fbase;
	A->Tbase=Re*A->Tbase;
	//A->Tbase=Re*A->dxd;

////////////////////////////////////////////////////////////////////////////////
/*Matrix3d Cph = MatrixXd::Zero(3,3);
	Vector3d absdx;
	absdx=A->dx;
	for(int i=0;i<3;i++){
		if(A->dx(i)<0) absdx(i)=-1*absdx(i);
		Cph(i,i)=60*exp(-5*absdx(i));
		if(Cph(i,i)<20) Cph(i,i)=20;
	}
	A->dxd = (A->fd - Cph*A->dx)*0.008/Mph + A->dx;
*/
////////////////////////////////////////////////////////////////////////////////

//Initial Calculation for algorithms 
	Vector3d l, cal_td;
	//l << 0.10, 0.10, 0.02;
	l << 0.05, 0.05, 0.02;
	cal_td << A->td(0)/l(0), A->td(1)/l(1), A->td(2)/l(2);

	double absdx, absfd, absdth, abscaltd;
	absdx=sqrt(A->dxd(0)*A->dxd(0)+A->dxd(1)*A->dxd(1)+A->dxd(2)*A->dxd(2));
	absfd=sqrt(A->fd(0)*A->fd(0)+A->fd(1)*A->fd(1)+A->fd(2)*A->fd(2));
	absdth=sqrt(A->dth(0)*A->dth(0)+A->dth(1)*A->dth(1)+A->dth(2)*A->dth(2));
	abscaltd=sqrt(cal_td(0)*cal_td(0)+cal_td(1)*cal_td(1)+cal_td(2)*cal_td(2));
	

//==============================================================================

//Position and Orientation Weighting
/*	double td_x, td_y, abscaltdw;
	td_x=cal_td(1)-A->fd(0);
	if((A->fd(0)*td_x)<0) td_x=0;
	td_y=cal_td(0)+A->fd(1);
	if((A->fd(1)*td_y)>0) td_y=0;
	abscaltdw= sqrt(td_x*td_x+td_y*td_y+cal_td(2)*cal_td(2));

	if(absfd>0.01||abscaltdw>0.01){
		double w;
		w = absfd/3/(absfd/3+abscaltdw/3);
		A->fd = A->fd*w;
		cal_td = cal_td*(1-w);
	}
*/
//==============================================================================
//Static Damping
/*
	Matrix3d Cph = MatrixXd::Identity(3,3);
	Cph=100*Cph;
	double Mph;
	Mph=20;
	A->dx = A->dxd;
	A->dxd = (A->fd - Cph*A->dx)*0.008/Mph + A->dx;


	Matrix3d Coh = MatrixXd::Identity(3,3);
	Coh=15*Coh;
	double Moh;
	Moh=3;
	A->dth = A->dthd;
	A->dthd = (cal_td - Coh*A->dth)*0.008/Moh + A->dth;
*/
//======*========================================================================
//Variable Damping
	double Cph_gain; 
	Matrix3d Cph = MatrixXd::Identity(3,3);
	double Mph;
	Mph=8;

	if(absdx<0.01) absdx=0.01;
	Cph_gain=2/absdx;
	if(Cph_gain>13) Cph_gain=13;
	if(Cph_gain<8) Cph_gain=8;

	if(absfd<0.1&&absdx>0.01) Cph_gain=Cph_gain*2;
	Cph=Cph_gain*Cph;
	//Cph=15*Cph;

	A->dx = A->dxd;
	A->dxd = (A->fd - Cph*A->dx)*0.002/Mph + A->dx;



	double Coh_gain; 
	Matrix3d Coh = MatrixXd::Identity(3,3);
	double Moh;
	Moh=3;

	if(absdth<0.01) absdth=0.01;
	Coh_gain=2/absdth;
	if(Coh_gain>20) Coh_gain=20;
	if(Coh_gain<15) Coh_gain=15;

	if(abscaltd<0.1&&absdth>0.01) Coh_gain=Coh_gain*5;
	Coh=Coh_gain*Coh;

	A->dth = A->dthd;
	A->dthd = (cal_td - Coh*A->dth)*0.002/Moh + A->dth;
	//A->C(0)=Cph_gain;
 	//A->C(1)=Coh_gain;
//////////////////////////////////////////////////////////////////////////////

//Linear path guide

/*
	int filternum=10;
	static int n=0;
	static float filter[250][6], FMc[6], FMc_pre[6];

	filternum=ceil(absdx*500);
  if(filternum>250) filternum=250;
	for (int i=0; i<6; i++){
		FMc[i]=A->FMc(i)/filternum;
		for (int j=0; j<filternum-1; j++){
			int seq=n-j;
			if(seq<0) seq=seq+250;
			FMc[i]=FMc[i]+filter[n-j][i]/filternum;
		}
		filter[n][i]=A->FMc(i);
	}
	n++;
	if(n==250) n=0;

	Cph(0,0)=Cph_gain;
	Cph(1,1)=Cph_gain*3;
	Cph(2,2)=Cph_gain*3;

	Vector3d FDirection;
	//FDirection << 1,1,-1;
	FDirection << FMc[0],FMc[1],FMc[2];
	//FDirection << A->fd(0), A->fd(1), A->fd(2);
	Matrix3d Rp;

	float alpha=atan2(FDirection(1),FDirection(0));
	Matrix3d Ralpha = MatrixXd::Zero(3,3);
	Ralpha(0,0)=cos(alpha);
	Ralpha(0,1)=-sin(alpha);
	Ralpha(1,0)=sin(alpha);
	Ralpha(1,1)=cos(alpha);
	Ralpha(2,2)=1;
	float beta=-atan2(FDirection(2),sqrt(FDirection(0)*FDirection(0)+FDirection(1)*FDirection(1)));
	Matrix3d Rbeta = MatrixXd::Zero(3,3);
	Rbeta(0,0)=cos(beta);
	Rbeta(0,2)=sin(beta);
	Rbeta(2,0)=-sin(beta);
	Rbeta(2,2)=cos(beta);
	Rbeta(1,1)=1;
	Rp = Ralpha*Rbeta;

	A->dx = A->dxd;
	A->dxd = (A->fd - Rp*Cph*Rp.inverse()*A->dx)*0.008/Mph + A->dx;
*/
/*
	Coh(0,0)=Coh_gain;
	Coh(1,1)=Coh_gain*3;
	Coh(2,2)=Coh_gain*3;

	Vector3d TDirection;
	FDirection << FMc[3],FMc[4],FMc[5];
	//FDirection << A->td(0), A->td(1), A->td(2);
	Matrix3d Ro;

	float Talpha=atan2(TDirection(1),TDirection(0));
	Matrix3d TRalpha = MatrixXd::Zero(3,3);
	TRalpha(0,0)=cos(Talpha);
	TRalpha(0,1)=-sin(Talpha);
	TRalpha(1,0)=sin(Talpha);
	TRalpha(1,1)=cos(Talpha);
	TRalpha(2,2)=1;
	float Tbeta=-atan2(TDirection(2),sqrt(TDirection(0)*TDirection(0)+TDirection(1)*TDirection(1)));
	Matrix3d TRbeta = MatrixXd::Zero(3,3);
	TRbeta(0,0)=cos(Tbeta);
	TRbeta(0,2)=sin(Tbeta);
	TRbeta(2,0)=-sin(Tbeta);
	TRbeta(2,2)=cos(Tbeta);
	TRbeta(1,1)=1;
	Ro = TRalpha*TRbeta;

	A->dth = A->dthd;
	A->dthd = (cal_td - Ro*Coh*Ro.inverse()*A->dth)*0.008/Moh + A->dth;
*/

//////////////////////////////////////////////////////////////////////////////


	K->Jacobian(A);
	MatrixXd JJt;
	JJt=A->J*A->J.transpose();
	A->J_t=A->J.transpose()*JJt.inverse();

	VectorXd dxdg(6);

	A->dxd_base=Re*A->dxd;
	A->dthd_base=Re*A->dthd;
	dxdg << A->dxd_base/500, A->dthd_base/500;
	//dxdg << 0, 0, 0, dthd_base/125;   //Orientation Only
	//dxdg << A->dxd_base/500, 0, 0, 0;    //Position Only

	A->dqd=A->J_t*dxdg;
	A->dqd=A->dqd*500;



	for(int i=0;i<6;i++){
		float tmp_dqd = A->dqd(i);
		if(isnan(tmp_dqd)){
			std::cout <<"NAN!!!!!!!!!!!!!!!!!" << std::endl;
			A->dqd(i)=0;
		}
	}

}

int ArmGuide::ArmForceGuide_position(AKfun *K, CArm *A)
{ //force guide with velocity command

	for(int i=0;i<3;i++){				//force
		if(A->FMc(i)>20)
			A->fd(i)=20-0.7;
		else if(A->FMc(i)<-20)
			A->fd(i)=-20+0.7;
		else if(A->FMc(i)>0.7)
			A->fd(i)=A->FMc(i)-0.7;
		else if(A->FMc(i)<-0.7)
			A->fd(i)=A->FMc(i)+0.7;
		else if(A->FMc(i)>-0.7&&A->FMc(i)<0.7)
			A->fd(i)=0;
	}
	for(int i=0;i<3;i++){				//moment
		if(A->FMc(i+3)>2)
			A->td(i)=2-0.03;
		else if(A->FMc(i+3)<-2)
			A->td(i)=-2+0.03;
		else if(A->FMc(i+3)>0.03)
			A->td(i)=A->FMc(i+3)-0.03;
		else if(A->FMc(i+3)<-0.03)
			A->td(i)=A->FMc(i+3)+0.03;
		else if(A->FMc(i+3)>-0.03&&A->FMc(i+3)<0.03)
			A->td(i)=0;
	}


	Matrix3d Re;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++)
			Re(i,j) = A->Tc(i,j);
	}

	for(int i=0;i<3;i++){	
		A->Fbase(i)=A->FMc(i)*3;
		A->Tbase(i)=A->FMc(i+3)*3;
	}
	A->Fbase=Re*A->Fbase;
	A->Tbase=Re*A->Tbase;
	//A->Tbase=Re*A->dxd;

//Initial Calculation for algorithms 
	Vector3d l, cal_td;
	//l << 0.10, 0.10, 0.02;
	l << 0.05, 0.05, 0.02;
	cal_td << A->td(0)/l(0), A->td(1)/l(1), A->td(2)/l(2);

	double absdx, absfd, absdth, abscaltd;
	absdx=sqrt(A->dxd(0)*A->dxd(0)+A->dxd(1)*A->dxd(1)+A->dxd(2)*A->dxd(2));
	absfd=sqrt(A->fd(0)*A->fd(0)+A->fd(1)*A->fd(1)+A->fd(2)*A->fd(2));
	absdth=sqrt(A->dth(0)*A->dth(0)+A->dth(1)*A->dth(1)+A->dth(2)*A->dth(2));
	abscaltd=sqrt(cal_td(0)*cal_td(0)+cal_td(1)*cal_td(1)+cal_td(2)*cal_td(2));
	
//======*========================================================================
//Variable Damping
	double Cph_gain; 
	Matrix3d Cph = MatrixXd::Identity(3,3);
	double Mph;
	Mph=8;

	if(absdx<0.01) absdx=0.01;
	Cph_gain=2/absdx;
	if(Cph_gain>13) Cph_gain=13;
	if(Cph_gain<8) Cph_gain=8;

	if(absfd<0.1&&absdx>0.01) Cph_gain=Cph_gain*2;
	Cph=Cph_gain*Cph;
	//Cph=15*Cph;

	A->dx = A->dxd;
	A->dxd = (A->fd - Cph*A->dx)*0.002/Mph + A->dx;



	double Coh_gain; 
	Matrix3d Coh = MatrixXd::Identity(3,3);
	double Moh;
	Moh=3;

	if(absdth<0.01) absdth=0.01;
	Coh_gain=2/absdth;
	if(Coh_gain>20) Coh_gain=20;
	if(Coh_gain<15) Coh_gain=15;

	if(abscaltd<0.1&&absdth>0.01) Coh_gain=Coh_gain*5;
	Coh=Coh_gain*Coh;

	A->dth = A->dthd;
	A->dthd = (cal_td - Coh*A->dth)*0.002/Moh + A->dth;

	K->Jacobian(A);
	MatrixXd JJt;
	JJt=A->J*A->J.transpose();
	A->J_t=A->J.transpose()*JJt.inverse();

	VectorXd dxdg(6);

	A->dxd_base=Re*A->dxd;
	A->dthd_base=Re*A->dthd;
	dxdg << A->dxd_base/500, A->dthd_base/500;
	//dxdg << 0, 0, 0, dthd_base/125;   //Orientation Only
	//dxdg << A->dxd_base/500, 0, 0, 0;    //Position Only

	A->xd = A->xc + A->dx*0.002 + 0.5*(A->dxd-A->dx)*0.002;
	A->thd = A->thc + A->dth*0.002 + 0.5*(A->dthd-A->dth)*0.002;

	K->EulerAngle2Rotation(A->Rd, A->thd);
	for(int i=0;i<3;i++){
		A->Td(i,3) = A->xd(i);
		for(int j=0;j<3;j++)
			A->Td(i,j) = A->Rd(i,j);
	}
	K->InverseK_min(A);


	//A->dqd=A->J_t*dxdg;
	//A->dqd=A->dqd*500;


	for(int i=0;i<6;i++){
		float tmp_qd = A->qd(i);
		if(isnan(tmp_qd)){
			std::cout <<"NAN!!!!!!!!!!!!!!!!!" << std::endl;
			A->qd(i)=A->qc(i);
		}
	}

}

int ArmGuide::MovingAverageFilter(double *x, int length, int filternum)
{
	//Max length 10, Max filter number 500
	static int FirstRun=0;
	static double xbuf[500][10], avg[10];
	
	if(FirstRun==0){
		for (int i=0; i<length; i++){
			for (int j=0; j<filternum; j++)		xbuf[j][i]=x[i];
			avg[i]=x[i];
		}
		FirstRun=1;
	}


	for (int i=0; i<length; i++){
		avg[i]=avg[i]+(x[i]-xbuf[0][i])/filternum;

		for (int j=0; j<filternum-1; j++)
			xbuf[j][i]=xbuf[j+1][i];

		xbuf[filternum-1][i]=x[i];

		x[i]=avg[i];
	}
}

int ArmGuide::LowpassFilter(double *x, int length, double alpha)
{
	//Max length 10
	static double xbuf[10];

	for (int i=0; i<length; i++)
	{
		xbuf[i]= alpha*xbuf[i] + (1-alpha)*x[i];
		x[i]=xbuf[i];
	}

}
