#include <iostream>
#include "inverse_kin.h"
inline double square(double x)
{
    return x * x;
}


int inverse_kin :: calculations(double position_data[3], double fi_array[3])
{
    double x = position_data[0];
    double y = position_data[1];
    double z = position_data[2];
    //Constants used to calculate Inverse Kinematics
    double Ei[3];
    Ei[0] = 2.0 * L * ( (x + a1) * u1 + (y + b1) * v1 );
    Ei[1] = 2.0 * L * ( (x + a2) * u2 + (y + b2) * v2 );
    Ei[2] = 2.0 * L * ( (x + a3) * u3 + (y + b3) * v3 );
    double Fi[3];
    double Fi_val = 2.0 * L * (z + c);
    Fi[0] = Fi_val;
    Fi[1] = Fi_val;
    Fi[2] = Fi_val;
    double Gi[3];
    Gi[0] = x * x + y * y + z * z
          + a1 * a1 + b1 * b1 + c * c
          + 2.0 * x * a1 + 2.0 * y * b1 + 2.0 * z * c
          + L * L - l * l;
    Gi[1] = x * x + y * y + z * z
          + a2 * a2 + b2 * b2 + c * c
          + 2.0 * x * a2 + 2.0 * y * b2 + 2.0 * z * c
          + L * L - l * l;
    Gi[2] = x * x + y * y + z * z
          + a3 * a3 + b3 * b3 + c * c
          + 2.0 * x * a3 + 2.0 * y * b3 + 2.0 * z * c
          + L * L - l * l;


    //Calculate motor angles, check if they are within specified range
    for (int i=0; i<3; i++){
      double temp = 2 * atan((-Fi[i] + sqrt(square(Fi[i]) + square(Ei[i]) - square(Gi[i]))) / (Gi[i] - Ei[i]));
      if (temp >= OUT_OF_RANGE_ANGLE_LOW && temp <= OUT_OF_RANGE_ANGLE_HIGH){
        fi_array[i] = temp;        
      }
      else{
        double temp = 2 * atan((-Fi[i] - sqrt(square(Fi[i]) + square(Ei[i]) - square(Gi[i]))) / (Gi[i] - Ei[i]));
        if (temp >= OUT_OF_RANGE_ANGLE_LOW && temp <= OUT_OF_RANGE_ANGLE_HIGH){
          fi_array[i] = temp;
        }
        else{
          fi_array[0] = -100;
          fi_array[1] = -100;
          fi_array[2] = -100;
          return 0; //-1 is error - coordiantes out of range
        }
      }
    }
  return 1;
};