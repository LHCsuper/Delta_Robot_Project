#ifndef inverse_kin_h
#define inverse_kin_h


//#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <mujoco/mujoco.h>

class inverse_kin 
{
private: 

const double OUT_OF_RANGE_ANGLE_LOW = -0.5235987756;
const double OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268;

//公共参数
// 角度常量（弧度制）
const double PI   = 3.14159265358979323846;
const double DEG30 = PI / 6.0;   // 30°
const double DEG60 = PI / 3.0;   // 60°
// geometry of the robot
const double r = 72;// end effector triangle radius
const double R = 218; // base triangle radius
const double alpha = 0.16634; 
const double z0 = -13; // end effector triangle offset in z direction
const double z1 = -112;  // base triangle offset in z direction
const double L = 405; // length of the upper arms
const double l = 930; // length of the forearms

//代入公式的参数

double c      = z0 - z1;       // c = z0 - z1

// a_i, b_i （三组）
// ==============================================
double a1 = R * cos(alpha) - (sqrt(3.0) / 2.0) * r;
double a2 = (sqrt(3.0) / 2.0) * r - R * sin(DEG30 + alpha);
double a3 = -R * sin(DEG30 - alpha);
double b1 = R * sin(alpha) - 0.5 * r;
double b2 = -0.5 * r + R * cos(DEG30 + alpha);
double b3 = r - R * cos(DEG30 - alpha);

double u1 = 1.0;
double v1 = 0.0;
double u2 = -0.5;
double v2 = sqrt(3.0) / 2.0;
double u3 = -0.5;
double v3 = -sqrt(3.0) / 2.0;


// vectors for motor joints in base frame {B} (from base frame to the point where motors are mounted)
double B_1[3] = {-R*cos(alpha), -R*sin(alpha), z1};
double B_2[3] = {R*sin(DEG30+alpha), -R*cos(DEG30+alpha), z1};
double B_3[3] = {R*sin(DEG30-alpha), R*cos(DEG30-alpha), z1};

// vectors for effector joints Pi in effector frame {P}
double P_1[3] = {-0.5*sqrt(3)*r, -0.5*r, z0};
double P_2[3] = {0.5*sqrt(3)*r, -0.5*r, z0};
double P_3[3] = {0, r, z0};

// list for the base vertices
// double Bvx[4] = {-sb / 2, 0, sb / 2, -sb / 2};
// double Bvy[4] = {-wb, ub, -wb, -wb};
// double Bvz[4] = {0, 0, 0, 0};

public:

int calculations(double position_data[3], double fi_array[3]);
void printBodyPosition(const mjModel* m, const mjData* d, const char* body_name);
};

#endif