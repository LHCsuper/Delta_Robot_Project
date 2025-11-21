#ifndef inverse_kin_h
#define inverse_kin_h


//#include <Arduino.h>
#include <math.h>
#include <stdio.h>

class inverse_kin 
{
private: 

const double OUT_OF_RANGE_ANGLE_LOW = -0.5235987756;
const double OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268;

// geometry of the robot
double sb = 375;  // length of the Side of the Base triangle in mm (sb)
double sp = 76; // length of the Side of the effector (Plate) triangle in mm (sp)
double Length = 200;  // length of the biceps in mm (L)
double length = 281;  // length of the forearm in mm (l)
double wb = 1.0 / 3 * sqrt(3) / 2 * sb;  // distance from the center of the Base to the side of the triangle
double ub = 2 * wb;  // distance from the center of the Base to the side of the triangle
double wp = 1.0 / 3 * sqrt(3) / 2 * sp;  // distance from the center of the effector (Plate) to the vertex of the triangle
double up = 2 * wp;  // distance from the center of the effector (Plate) to the vertex of the triangle
double a = wb - up;
double b = sp / 2 - sqrt(3) / 2 * wb;
double c = wp - 1.0 / 2 * wb;



// vectors for motor joints in base frame {B} (from base frame to the point where motors are mounted)
double B_1[3] = {0, -wb, 0};
double B_2[3] = {sqrt(3) / 2 * wb, 1.0 / 2 * wb, 0};
double B_3[3] = {-sqrt(3) / 2 * wb, 1.0 / 2 * wb, 0};

// vectors for effector joints Pi in effector frame {P}
double P_1[3] = {0, -up, 0};
double P_2[3] = {sp / 2, wp, 0};
double P_3[3] = {-sp / 2, wp, 0};

// list for the base vertices
double Bvx[4] = {-sb / 2, 0, sb / 2, -sb / 2};
double Bvy[4] = {-wb, ub, -wb, -wb};
double Bvz[4] = {0, 0, 0, 0};

public:

int calculations(double position_data[3], double fi_array[3]);
};

#endif