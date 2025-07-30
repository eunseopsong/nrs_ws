//////////////////////////////////////////////////////////////////////////
// include files
//////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <cmath>

// Sensor data include
#include "ros/ros.h"
#include "ft_ros_tracker/vive_ft_msg.h"

// OpenGL include
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

//////////////////////////////////////////////////////////////////////////
// static constants, types, macros, variables
//////////////////////////////////////////////////////////////////////////
#define RADPERDEG 0.0174533
#define SCALE 40.0
#define AVG_SIZE 25
#define Radius 25 //Unit: mm
//////////////////////////////////////////////////////////////////////////
// global variables
//////////////////////////////////////////////////////////////////////////
// double ft1[AVG_SIZE]={0.0,},ft2[AVG_SIZE]={0.0,},ft3[AVG_SIZE]={0.0,},ft4[AVG_SIZE]={0.0,},ft5[AVG_SIZE]={0.0,},ft6[AVG_SIZE]={0.0,};
// double f_sum1,f_sum2,f_sum3,f_sum4,f_sum5,f_sum6;

double contact_p[3] = {0};
double arrowstart_p[3] = {0};
int spin = 0;
GLfloat xAngle =0 , yAngle=0 , zAngle=0;
//////////////////////////////////////////////////////////////////////////
// static function prototypes
//////////////////////////////////////////////////////////////////////////
// void    FinalizeApp  ( void );
// void    TransmitData ( void );

// FILE* fp_ftsensor;
//////////////////////////////////////////////////////////////////////////
// global functions
//////////////////////////////////////////////////////////////////////////

void init_light(void);							// 광원 초기화
// Callback Functions
void DoDisplay(void);						// 처음 윈도우를 열 때, 윈도우 위치를 옮길 때, 윈도우 크기를 조절할 때, 
											//앞윈도우에 가려져 안보이던 뒤 윈도우가 활성화될때, glutPPostRedisplay()함수에 의해 이벤트 큐에 플레그 될때
void DoReshape(int width, int height);		// 처음 윈도우를 열 때, 윈도우 위치를 옮길 때, 윈도우 크기를 조절할 때
void DoMouse(int button, int state, int x, int y);
void DoKeyboard(unsigned char key, int x, int y);
void DoTimer_animation(int value);
void PilotView(GLfloat roll, GLfloat pitch, GLfloat yaw);
void spindisplay(void);

void DrawSensor(float fingertip_radius, float sensor_height);
void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2, GLdouble D);
void DrawAxes(GLdouble length);

// void EqualArray(double A[], double B[], int vector_length);
// void MatrixMultiply(double x[][8], double y[], double z[]);
void Centroid(double cp[]);

typedef struct canblock
{
	double force[3];
	double moment[3];
	//double Bias[8];  //센서 값 계산 위함 -> 셀이 8개
} CanStruct;
double fMag	= 0;		// force Magnitude
//////////////////////////////////////////////////////////////////////////
// Share with Thread 1
//////////////////////////////////////////////////////////////////////////
// int SET_ZERO=1;		//초기에 센서값을 zero로 설정, zero setting flag
CanStruct Recvblock;	//전역변수로 설정

//////////////////////////////////////////////////////////////////////////
// static functions
//////////////////////////////////////////////////////////////////////////

void forceCallback(const ft_ros_tracker::vive_ft_msg::ConstPtr& msg)
{
    ROS_INFO("Received Force - Fx: %f, Fy: %f, Fz: %f", msg->Fx, msg->Fy, msg->Fz);

    Recvblock.force[0] = msg->Fx;
    Recvblock.force[1] = msg->Fy;
    Recvblock.force[2] = msg->Fz;
}

void momentCallback(const ft_ros_tracker::vive_ft_msg::ConstPtr& msg)
{
    ROS_INFO("Received Moment - Mx: %f, My: %f, Mz: %f", msg->Mx, msg->My, msg->Mz);

    Recvblock.moment[0] = msg->Mx;
    Recvblock.moment[1] = msg->My;
    Recvblock.moment[2] = msg->Mz;
}
//////////////////////////////////////////////////////////////////////////
/**
  Main entry point of the application.
*/
//////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "ft_ros_sensor"); 
    ros::NodeHandle nh;
    ros::Subscriber force_sub = nh.subscribe("vive_force", 10, forceCallback);
    ros::Subscriber moment_sub = nh.subscribe("vive_moment", 10, momentCallback);


	static GLfloat spin = 0.0;
	// int count = 0;

	//  int i, j = 0;	// for문

	// 	for (i=0; i<8; i++)
	// 		Recvblock.Bias[i] = 0;		// Bias값 초기화
	// =========================
	// printf(" >>>> aaa <<<<\n");

	// GLUT 초기화
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("OpenGL Application");

    // OpenGL 설정 함수 호출
    init_light();

    // 디스플레이 및 이벤트 콜백 함수 설정
    glutDisplayFunc(DoDisplay);
    glutReshapeFunc(DoReshape);
    glutMouseFunc(DoMouse);
    glutKeyboardFunc(DoKeyboard);
    glutTimerFunc(10, DoTimer_animation, 0);

    // 이벤트 루프 시작
    glutMainLoop();

	ros::spin();
	return 0;
}



/*
 *	광원 초기화
 */
void init_light(void)
{
	//TransmitData();

	static GLfloat lit_amb[4]={1.0, 1.0, 1.0, 0.0};	// 주변광의 강도
	static GLfloat lit_dif[4]={1.0, 1.0, 1.0, 0.0}; // 분산광의 강도
	static GLfloat lit_spc[4]={1.0, 1.0, 1.0, 0.0};	// 반사광의 강도
	static GLfloat lit_pos[4]={6.0, 6.0, 9.0, 0.5};// 광원의 위치
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL); 

	// Light0에 설정
	glLightfv(GL_LIGHT0, GL_AMBIENT, lit_amb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lit_dif);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lit_spc);
	glLightfv(GL_LIGHT0, GL_POSITION, lit_pos);

	// 광원을 사용하겠다
	
	glEnable(GL_LIGHTING);
}

/*
 *	화면변경시의 처리
 */
void DoDisplay(void)
{
	static GLfloat mat_amb[4]={0.2, 0.2, 0.2, 0.0};	// 주변광에 대한 반사값
	static GLfloat mat_dif[4]={0.4, 0.4, 0.4, 0.0};	// 분산광에 대한 반사값
	static GLfloat mat_spc[4]={0.2, 0.2, 0.2, 0.0};	// 반사광에 대한 반사값
	static GLfloat mat_emi[4]={0.0, 0.0, 0.2, 0.0};	// 발광값
	float mat_shi=60.0;	// 광택
	//GLfloat mat_emi2[4]={1.0, 0.0, 0.0, 0.2};	// 발광값
	
	// 힘 화살표 그리기
	float mat_ambient2[4] = {0.2, 0, 0, 1.0};
	float mat_diffuse2[4] = {0.7, 0, 0, 1.0};
	float mat_specular2[4] = {0.50, 0.50, 0.50, 1.0};
	float mat_shininess2 = 32;

	float mat_ambient3[] = {0.2, 0, 0, 1.0};
	float mat_diffuse3[] = {1, 1, 0.5, 1.0};
	float mat_specular3[] = {0.5, 0.5, 0.5, 1.0};
	float mat_shininess3 = 32;

	GLfloat tip_r = 0;				
	GLfloat sensor_h = 0;				
	int i = 0; // for 문
	
	// 화면과 디스플레이버퍼를 제거
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	// 질감을 설정
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_amb);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_spc);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emi);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shi);
	
	// 시점 대신 애초에 회전된 좌표계에 센서를 그림
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	Arrow(0.0, 0, 0, 0.0, 0, 0, 1.0);
	glPushMatrix();
	glTranslatef(0.0, -0.2, 0.0);
	glRotatef(-120, 1.0, 0.0, 0.0);
	glRotatef(45, 0, 0, 1.0);
	PilotView(xAngle, yAngle,zAngle);
	
	// === 센서 그리기 === //
	tip_r = 25.0 / 2 / SCALE;	
	sensor_h = 15.0 / SCALE;				
	glColor3f(1.0, 1.0, 1.0);
	DrawSensor(tip_r, sensor_h);

	glColor3f(0.5, 0, 0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular2);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess2);
	
	for (i =0 ; i < 3; i++) 
	{
		arrowstart_p[i]= arrowstart_p[i] / SCALE;		// arrow의 scale은 따로 하는게 좋음. [mm] -> [GL]
		contact_p[i]= contact_p[i] / SCALE;				// [mm] -> [GL]
	}
	//printf("C : %lf %lf %lf \n", contact_p[0], contact_p[1], contact_p[2]);
	//printf("A: %lf %lf %lf \n", arrowstart_p[0], arrowstart_p[1], arrowstart_p[2]);
	if (fMag > 5)
		Arrow(arrowstart_p[0], arrowstart_p[1], arrowstart_p[2], contact_p[0], contact_p[1], contact_p[2], 0.04);

	glPushMatrix();
	glLoadIdentity();
	glTranslatef(-0.6, 0.7, 0);
	glRotatef(-120, 1.0, 0.0, 0.0);
	glRotatef(45, 0, 0, 1.0);
	PilotView(xAngle, yAngle,zAngle);
	// x-y-z 좌표축
	
	DrawAxes(0.4);
	glPopMatrix();
	//Arrow(0.0, 0, 0.0, 0, 0, 1.0);
	//Arrow(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
	//Arrow(0.0, 0.0, 0.0, 0, 1.0, 0.0);
	glPopMatrix();

	//PilotView(xAngle, yAngle, zAngle);

	// 화면을 갱신
	glutSwapBuffers();
	//printf("display\n");

}
void DoReshape(int width, int height)
{
	// 표시범위를 설정
	glViewport(0, 0, width, height);

	//// 투영방법 설정
	// glMatrixMode(GL_PROJECTION);
	// glLoadIdentity();
	// gluPerspective(60.0, (GLfloat) width/ (GLfloat) height, 1, 20.0);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//gluLookAt(0.0,0.0,0.0,0.0,0.0,-1.0,0.0,1.0,0.0);
	//gluLookAt(0.0,0.0,1.0,0.0,0.0,-1.0,0.0,1.0,0.0);

	// 광원 설정
	init_light();
	//printf("reshape\n");
}
void PilotView(GLfloat roll, GLfloat pitch, GLfloat yaw)
{
	glRotatef(roll, 0.0, 0.0, 1.0);
	glRotatef(pitch, 0.0, 1.0, 0.0);
	glRotatef(yaw, 1.0, 0.0, 0.0);
	//glTranslatef(-x, -y, -z);
}
void spindisplay(void)
{
	//화면 갱신내용//
	spin = spin + 0.05;
	if (spin > 360.0)
		spin = spin - 360.0;

	glutPostRedisplay();	

}
void DoTimer_animation(int value)
{
	// double f[3] = {2.0, 0.0, 1.0};		// force
	// double m[3] = {0.0, -3.5, 1.0};		// moment
	double f[3] = {Recvblock.force[0], Recvblock.force[1], Recvblock.force[2]};		// force
	double m[3] = {Recvblock.moment[0], Recvblock.moment[1], Recvblock.moment[2]};  // moment
	
	double Fmax = 5;		// maximum torque를 넣어서 화살표의 길이를 normalize 할 필요가 있음
	int i = 0;				//for 문
	static double theta = 0;		// 임시로
	int abs = 0;

	// EqualArray(f, Recvblock.force , 3);
	// EqualArray(m, Recvblock.moment, 3);
	// printf("%d, %d, %d,%d, %d, %d\n", f[0], f[1], f[2], m[0], m[1], m[2] );
	// Centroid(f[0], f[1], f[2], m[0], m[1], m[2], Radius, contact_p);    //FingerTip. real radius!!
	Centroid(contact_p);

	//좌표계 GL에 맞게 변형
	f[2]=-f[2];
	contact_p[2]=-contact_p[2];
	
	fMag= sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
	for( i=0; i < 3; i++){

	
		arrowstart_p[i] = contact_p[i] + 12.5 * fabs(fMag/Fmax) * (-1)*f[i]/fMag;		// fMag/gamma : 화살표의 길이

	// DEBUG -> 죄다 nan 나옴!!!!!!!!!
	// printf("C : %.3lf %.3lf %.3lf \n", contact_p[0], contact_p[1], contact_p[2]);
	//printf("Cnorm : %lf \n", sqrt(contact_p[0]*contact_p[0]+contact_p[1]*contact_p[1]+contact_p[2]*contact_p[2]));
	// printf("P : %.3lf %.3lf %.3lf \n", arrowstart_p[0], arrowstart_p[1], arrowstart_p[2]);
	// printf("f : %lf, %lf, %lf \n", f[0], f[1], f[2]);
	// printf("fnorm : %lf \n", fMag);
	
	glutPostRedisplay();
	glutTimerFunc(10, DoTimer_animation, 1);

	printf("arrow: %d, %d, %d \n", arrowstart_p[0], arrowstart_p[1], arrowstart_p[2]); // arrowstart_p[1] 만 나옴
}
}

void DoMouse(int button, int state, int x, int y)
{
	switch(button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
			glutIdleFunc(spindisplay);		//화면을 갱신해야할 때 호출!
		break;

	case GLUT_MIDDLE_BUTTON:
			if (state == GLUT_DOWN)
				glutIdleFunc(NULL);
			break;

	default:
		break;
	}
}
void DoKeyboard(unsigned char key, int x, int y)
{
	int increase_angle = 5;
	switch(key) 
	{
	// case 'r':
	// 	// Set zero
	// 	SET_ZERO = 1;
	// 	break;
	case 'a':
		yAngle += increase_angle;
		break;
	case 'd':
		yAngle -= increase_angle;
		break;
	case 'w':
		xAngle += increase_angle;
		break;
	case 's':
		xAngle -= increase_angle;
		break;
	case 'q':
		zAngle += increase_angle;
		break;
	case 'e':
		zAngle -= increase_angle;
		break;
	case 'z':
		xAngle = 0;
		yAngle = 0;
		zAngle = 0;
		break;
	case 27:  // esc의 ASCII 코드 값
		exit(0);
		break;
	}

	// char info[128]; 
	// sprintf(info, &quot; x=%.1f, y=%.1f, z=%.1f&quot;, xAngle, yAngle, zAngle); 
	// glutSetWindowTitle(info);    
	glutPostRedisplay();
}
void DrawSensor(float fingertip_radius, float sensor_height)
{
	GLfloat tip_r = (GLfloat) fingertip_radius;				// 그래픽상의 크기
	GLfloat cyl_h = (GLfloat) sensor_height;
	GLUquadric *cylinder, *sphere, *quadObj;

	glMatrixMode(GL_MODELVIEW);
	// glLoadIdentity();
	glPushMatrix();
//	glTranslatef(0.0, 0.0, -15.0); //화면 안쪽으로 좌표계 이동
	//glRotatef((GLfloat) -90, 1, 0, 0);	// z축이 위로가게.

	// 구를 그린다
	sphere = gluNewQuadric();
	gluSphere(sphere, tip_r, 20, 16);

	glColor3ub(100, 100, 100);	// gray
	glTranslatef(0, 0, -cyl_h);
	cylinder = gluNewQuadric();
	gluCylinder(cylinder, tip_r, tip_r, cyl_h, 20, 16);

	quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, tip_r, 32, 1);
    gluDeleteQuadric(quadObj);


	glPopMatrix();

}
void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2, GLdouble D)
{
  double x=(x2-x1);
  double y=(y2-y1);
  double z=(z2-z1);
  double L=sqrt(x*x+y*y+z*z);
  GLUquadricObj *quadObj;
	
	glPushMatrix ();

      glTranslated(x1,y1,z1);

      if((x!=0.)||(y!=0.)) {
        glRotated(atan2(y,x)/RADPERDEG,0.,0.,1.);
        glRotated(atan2(sqrt(x*x+y*y),z)/RADPERDEG,0.,1.,0.);
      } else if (z<0){
        glRotated(180,1.,0.,0.);
      }

      glTranslatef(0,0,L-4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, 2*D, 32, 1);
      gluDeleteQuadric(quadObj);

      glTranslatef(0,0,-L+4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, D, D, L-4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, D, 32, 1);
      gluDeleteQuadric(quadObj);

    glPopMatrix ();

}
void DrawAxes(GLdouble length)
{
	GLdouble diameter = 0.01;
	glColor3f(1, 0, 0);						// Red
    Arrow(0,0,0, length,0,0, diameter);		// x-axis
	glColor3f(0, 1, 0);						// Green
    Arrow(0,0,0, 0,length,0, diameter);		// y-axis
	glColor3f(0, 0, 1);						// Blue
    Arrow(0,0,0, 0,0,-length, diameter);		// z-axis

}
void Centroid(double cp[])
{
	// 반지름 단위 : [mm]
	// 힘 [N]
	// 모멘트, [N.mm]

	// double Radius = 25;  // 반지름

	double f[3] = {Recvblock.force[0], Recvblock.force[1], Recvblock.force[2]};		// force
	double m[3] = {Recvblock.moment[0], Recvblock.moment[1], Recvblock.moment[2]}; 


	double a =f[0]*m[0]+f[1]*m[1]+f[2]*m[2];
	double sigma = (m[0]*m[0] + m[1]*m[1] + m[2]*m[2]) - Radius*Radius*(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
	double b = sqrt(sigma*sigma + 4*Radius*Radius*a*a);
	double c = sigma + b;
	double K = 0;
	double g= 0;
	double d[3] = {0};
	double e[3] = {0};
	
	if (a > 0)
		K = (-1)*sqrt(c)/(sqrt(2.f)*Radius);
	else
		K =  sqrt(c)/(sqrt(2.f)*Radius);

	d[0] = f[1]*m[2]-f[2]*m[1];
	d[1]= -f[0]*m[2]+f[2]*m[0];
	d[2] = f[0]*m[1]-f[1]*m[0];

	e[0] =  K*K*m[0] + K*d[0] + a*f[0];
	e[1] = K*K*m[1] + K*d[1] + a*f[1];
	e[2] = K*K*m[2] + K*d[2] + a*f[2];

	g = K*(K*K + f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);

	//centroid = e/g
	cp[0] = e[0]/g;
	cp[1] = e[1]/g;
	cp[2] = e[2]/g;
}

// ========================== Matrix Computation Functions ===================================//
// void EqualArray(double A[], double B[], int vector_length)
// {
// 	// A = B. Put values of B into A
// 	int i =0;
// 	for (i=0 ; i <vector_length; i++)
// 		A[i] = B[i];
// }
// void MatrixMultiply(double x[][8], double y[], double z[])
// {
// 	int i, j, k;

// 	for(i=0; i < 6; i++)
// 	{
// 		z[i] = 0;
// 		for(k=0; k< 8; k++)
// 			// same as z[i][j] = z[i][j] + x[i][k] * y[k][j];
// 			z[i] += x[i][k] * y[k];
// 	}
// }

